#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from planning_interfaces.srv import PickPlaceService
import time

from planning.ik import IKPlanner  


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # Parameters
        self.declare_parameter('planning_group', 'tmr_arm')
        self.declare_parameter('end_effector_link', 'link_6')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('approach_distance', 0.1)

        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.approach_distance = self.get_parameter('approach_distance').value

        # Callback group
        self.callback_group = ReentrantCallbackGroup()

        # IK Planner (your working approach!)
        self.ik_planner = IKPlanner()
        self.get_logger().info('IK Planner initialized')

        # Joint state tracking
        self.current_joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Action client for trajectory execution
        self.exec_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/tmr_arm_controller/follow_joint_trajectory',  # TM12 controller
            callback_group=self.callback_group
        )
        self.exec_client.wait_for_server()
        self.get_logger().info('Connected to trajectory controller')

        # Service for pick and place
        self.pick_place_srv = self.create_service(
            PickPlaceService,
            'pick_and_place',
            self.pick_place_callback,
            callback_group=self.callback_group
        )

        self.processing = False

        self.get_logger().info('Ready! Using IK-based planning')

    def joint_state_callback(self, msg):
        """Store latest joint state"""
        self.current_joint_state = msg

    async def pick_place_callback(self, request, response):
        """Execute pick and place using IK + motion planning"""
        if self.processing:
            response.success = False
            response.message = "Already processing"
            return response

        if self.current_joint_state is None:
            response.success = False
            response.message = "No joint state available"
            return response

        self.processing = True
        
        try:
            pick_pose = request.pick_pose
            place_pose = request.place_pose
            
            self.get_logger().info('='*60)
            self.get_logger().info('Starting Pick and Place (IK-based)')
            self.get_logger().info(f'Pick:  x={pick_pose.pose.position.x:.3f}, '
                                 f'y={pick_pose.pose.position.y:.3f}, '
                                 f'z={pick_pose.pose.position.z:.3f}')
            self.get_logger().info(f'Place: x={place_pose.pose.position.x:.3f}, '
                                 f'y={place_pose.pose.position.y:.3f}, '
                                 f'z={place_pose.pose.position.z:.3f}')
            self.get_logger().info('='*60)

            # Build job queue
            job_queue = []

            # 1. Compute IK for pick approach
            self.get_logger().info('Step 1/8: Computing IK for pick approach...')
            pick_approach = self.get_approach_pose(pick_pose)
            ik_result = self.ik_planner.compute_ik(
                self.current_joint_state,
                pick_approach.pose.position.x,
                pick_approach.pose.position.y,
                pick_approach.pose.position.z,
                pick_approach.pose.orientation.x,
                pick_approach.pose.orientation.y,
                pick_approach.pose.orientation.z,
                pick_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for pick approach"
                return response
            job_queue.append(ik_result)

            # 2. Compute IK for pick position
            self.get_logger().info('Step 2/8: Computing IK for pick...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                pick_pose.pose.position.x,
                pick_pose.pose.position.y,
                pick_pose.pose.position.z,
                pick_pose.pose.orientation.x,
                pick_pose.pose.orientation.y,
                pick_pose.pose.orientation.z,
                pick_pose.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for pick"
                return response
            job_queue.append(ik_result)

            # 3. Close gripper
            job_queue.append('close_gripper')

            # 4. Retreat
            self.get_logger().info('Step 4/8: Computing IK for pick retreat...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                pick_approach.pose.position.x,
                pick_approach.pose.position.y,
                pick_approach.pose.position.z,
                pick_approach.pose.orientation.x,
                pick_approach.pose.orientation.y,
                pick_approach.pose.orientation.z,
                pick_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for pick retreat"
                return response
            job_queue.append(ik_result)

            # 5. Place approach
            self.get_logger().info('Step 5/8: Computing IK for place approach...')
            place_approach = self.get_approach_pose(place_pose)
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                place_approach.pose.position.x,
                place_approach.pose.position.y,
                place_approach.pose.position.z,
                place_approach.pose.orientation.x,
                place_approach.pose.orientation.y,
                place_approach.pose.orientation.z,
                place_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for place approach"
                return response
            job_queue.append(ik_result)

            # 6. Place position
            self.get_logger().info('Step 6/8: Computing IK for place...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                place_pose.pose.position.x,
                place_pose.pose.position.y,
                place_pose.pose.position.z,
                place_pose.pose.orientation.x,
                place_pose.pose.orientation.y,
                place_pose.pose.orientation.z,
                place_pose.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for place"
                return response
            job_queue.append(ik_result)

            # 7. Open gripper
            job_queue.append('open_gripper')

            # 8. Retreat
            self.get_logger().info('Step 8/8: Computing IK for place retreat...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                place_approach.pose.position.x,
                place_approach.pose.position.y,
                place_approach.pose.position.z,
                place_approach.pose.orientation.x,
                place_approach.pose.orientation.y,
                place_approach.pose.orientation.z,
                place_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for place retreat"
                return response
            job_queue.append(ik_result)

            # Execute all jobs
            success = await self.execute_job_queue(job_queue)
            
            if success:
                self.get_logger().info('='*60)
                self.get_logger().info('✅ Pick and Place Complete!')
                self.get_logger().info('='*60)
                response.success = True
                response.message = "Success"
            else:
                response.success = False
                response.message = "Execution failed"

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            response.success = False
            response.message = f"Exception: {str(e)}"
        finally:
            self.processing = False

        return response

    async def execute_job_queue(self, job_queue):
        """Execute all jobs in the queue"""
        for i, job in enumerate(job_queue):
            self.get_logger().info(f'Executing job {i+1}/{len(job_queue)}...')
            
            if isinstance(job, JointState):
                # Plan to joint configuration
                trajectory = self.ik_planner.plan_to_joints(job)
                if trajectory is None:
                    self.get_logger().error('Planning failed')
                    return False
                
                # Execute trajectory
                if not await self.execute_trajectory(trajectory.joint_trajectory):
                    return False
                    
            elif job == 'close_gripper':
                self.get_logger().info('🤏 Closing gripper...')
                time.sleep(1.0)  # TODO: Implement gripper control
                
            elif job == 'open_gripper':
                self.get_logger().info('✋ Opening gripper...')
                time.sleep(1.0)  # TODO: Implement gripper control
                
        return True

    async def execute_trajectory(self, joint_trajectory):
        """Execute a joint trajectory"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory

        goal_handle = await self.exec_client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result = await goal_handle.get_result_async()
        return result.result.error_code == 0

    def get_approach_pose(self, target_pose):
        """Get approach pose above target"""
        approach = PoseStamped()
        approach.header = target_pose.header
        approach.pose.position.x = target_pose.pose.position.x
        approach.pose.position.y = target_pose.pose.position.y
        approach.pose.position.z = target_pose.pose.position.z + self.approach_distance
        approach.pose.orientation = target_pose.pose.orientation
        return approach


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    
    node.executor = executor
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()