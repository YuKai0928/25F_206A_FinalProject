#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from planning_interfaces.srv import PickPlaceService, MoveToTarget
from std_srvs.srv import SetBool

from planning.ik import IKPlanner  


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # Parameters
        self.declare_parameter('planning_group', 'tmr_arm')
        self.declare_parameter('end_effector_link', 'link_6')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('approach_distance', 0.1)
        self.declare_parameter('goal_tolerance', 0.005)  # Joint goal tolerance in radians (default: 0.01 rad ≈ 0.57°)

        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

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

        # Gripper service client
        self.gripper_client = self.create_client(
            SetBool,
            '/gripper/control',
            callback_group=self.callback_group
        )
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Gripper service not available, will continue anyway')
        else:
            self.get_logger().info('Connected to gripper controller')

        # Service for pick and place
        self.pick_place_srv = self.create_service(
            PickPlaceService,
            'pick_and_place',
            self.pick_place_callback,
            callback_group=self.callback_group
        )

        # Service for move to target
        self.move_to_target_srv = self.create_service(
            MoveToTarget,
            'move_to_target',
            self.move_to_target_callback,
            callback_group=self.callback_group
        )

        self.processing = False

        self.get_logger().info('Ready! Using IK-based planning')

    def joint_state_callback(self, msg):
        """Store latest joint state"""
        self.current_joint_state = msg

    async def move_to_target_callback(self, request, response):
        """Service callback to move to target position"""
        if self.processing:
            response.success = False
            response.message = "Already processing"
            return response

        self.processing = True

        try:
            success = await self.move_to_target(request.target_pose)
            response.success = success
            response.message = "Success" if success else "Failed to reach target"
        except Exception as e:
            self.get_logger().error(f'Exception in move_to_target: {e}')
            response.success = False
            response.message = f"Exception: {str(e)}"
        finally:
            self.processing = False

        return response

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

            # 1. Compute IK for pick approach (two-step: position then orientation)
            self.get_logger().info('Step 1/8: Computing IK for pick approach...')
            pick_approach = self.get_approach_pose(pick_pose)
            position_ik, ik_result = self.move_to_pose_two_step(
                self.current_joint_state,
                pick_approach
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for pick approach"
                return response
            # Add both steps to job queue
            job_queue.append(position_ik)  # First reach position
            job_queue.append(ik_result)     # Then adjust orientation

            # 2. Compute IK for pick position (straight down, maintain approach orientation)
            self.get_logger().info('Step 2/8: Computing IK for pick (descending)...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                pick_pose.pose.position.x,
                pick_pose.pose.position.y,
                pick_pose.pose.position.z,
                pick_approach.pose.orientation.x,  # Maintain approach orientation
                pick_approach.pose.orientation.y,
                pick_approach.pose.orientation.z,
                pick_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for pick"
                return response
            job_queue.append(ik_result)

            # 3. Close gripper
            job_queue.append('close_gripper')

            # 4. Retreat (straight up)
            self.get_logger().info('Step 4/8: Computing IK for pick retreat (ascending)...')
            retreat_ik = self.ik_planner.compute_ik(
                ik_result,
                pick_approach.pose.position.x,
                pick_approach.pose.position.y,
                pick_approach.pose.position.z,
                pick_approach.pose.orientation.x,
                pick_approach.pose.orientation.y,
                pick_approach.pose.orientation.z,
                pick_approach.pose.orientation.w
            )
            if retreat_ik is None:
                response.success = False
                response.message = "IK failed for pick retreat"
                return response
            job_queue.append(retreat_ik)

            # 5. Place approach (two-step: position then orientation)
            self.get_logger().info('Step 5/8: Computing IK for place approach...')
            place_approach = self.get_approach_pose(place_pose)
            position_ik, ik_result = self.move_to_pose_two_step(
                ik_result,
                place_approach
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for place approach"
                return response
            # Add both steps to job queue
            job_queue.append(position_ik)  # First reach position
            job_queue.append(ik_result)     # Then adjust orientation

            # 6. Place position (straight down, maintain approach orientation)
            self.get_logger().info('Step 6/8: Computing IK for place (descending)...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                place_pose.pose.position.x,
                place_pose.pose.position.y,
                place_pose.pose.position.z,
                place_approach.pose.orientation.x,  # Maintain approach orientation
                place_approach.pose.orientation.y,
                place_approach.pose.orientation.z,
                place_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for place"
                return response
            job_queue.append(ik_result)

            # 7. Open gripper
            job_queue.append('open_gripper')

            # 8. Retreat (straight up)
            self.get_logger().info('Step 8/8: Computing IK for place retreat (ascending)...')
            retreat_ik = self.ik_planner.compute_ik(
                ik_result,
                place_approach.pose.position.x,
                place_approach.pose.position.y,
                place_approach.pose.position.z,
                place_approach.pose.orientation.x,
                place_approach.pose.orientation.y,
                place_approach.pose.orientation.z,
                place_approach.pose.orientation.w
            )
            if retreat_ik is None:
                response.success = False
                response.message = "IK failed for place retreat"
                return response
            job_queue.append(retreat_ik)

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
                trajectory = self.ik_planner.plan_to_joints(job, goal_tolerance=self.goal_tolerance)
                if trajectory is None:
                    self.get_logger().error('Planning failed')
                    return False

                # Execute trajectory
                if not await self.execute_trajectory(trajectory.joint_trajectory):
                    return False

            elif job == 'close_gripper':
                self.get_logger().info('🤏 Closing gripper...')
                if not await self.control_gripper(True):
                    self.get_logger().error('Failed to close gripper')
                    return False

            elif job == 'open_gripper':
                self.get_logger().info('✋ Opening gripper...')
                if not await self.control_gripper(False):
                    self.get_logger().error('Failed to open gripper')
                    return False

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

    async def control_gripper(self, close: bool):
        """Control gripper: True to close, False to open"""
        if not self.gripper_client.service_is_ready():
            self.get_logger().warn('Gripper service not available')
            return True  # Continue anyway

        request = SetBool.Request()
        request.data = close

        future = self.gripper_client.call_async(request)
        response = await future

        if response.success:
            action = "closed" if close else "opened"
            self.get_logger().info(f'Gripper {action}: {response.message}')
            return True
        else:
            self.get_logger().error(f'Gripper control failed: {response.message}')
            return False

    def get_approach_pose(self, target_pose):
        """Get approach pose above target"""
        approach = PoseStamped()
        approach.header = target_pose.header
        approach.pose.position.x = target_pose.pose.position.x
        approach.pose.position.y = target_pose.pose.position.y
        approach.pose.position.z = target_pose.pose.position.z + self.approach_distance
        approach.pose.orientation = target_pose.pose.orientation
        return approach

    def move_to_pose_two_step(self, start_joint_state, target_pose):
        """
        Move to target pose in two steps to avoid excessive joint 1 rotation:
        1. Move to target position with intermediate orientation (gripper down)
        2. Adjust to target orientation

        Returns: tuple (position_ik_result, final_ik_result) or (None, None) on failure
        """
        # Step 1: Move to target position with gripper pointing down (default orientation)
        # This uses qx=1.0, qy=0.0, qz=0.0, qw=0.0 which is the default in compute_ik
        self.get_logger().info('  → Step 1: Moving to target position with intermediate orientation...')
        position_ik = self.ik_planner.compute_ik(
            start_joint_state,
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
            1.0, 0.0, 0.0, 0.0  # Gripper pointing down (default orientation)
        )

        if position_ik is None:
            self.get_logger().error('IK failed for position-only movement')
            return None, None

        # Step 2: Adjust orientation to target orientation
        self.get_logger().info('  → Step 2: Adjusting to target orientation...')
        final_ik = self.ik_planner.compute_ik(
            position_ik,  # Start from the position we just reached
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        )

        if final_ik is None:
            self.get_logger().error('IK failed for orientation adjustment')
            return None, None

        return position_ik, final_ik

    async def move_to_target(self, target_pose):
        """Move end effector to target position using IK (two-step approach)"""
        if self.current_joint_state is None:
            self.get_logger().error('No joint state available')
            return False

        try:
            self.get_logger().info('='*60)
            self.get_logger().info('Moving to target position (IK-based, two-step)')
            self.get_logger().info(f'Target: x={target_pose.pose.position.x:.3f}, '
                                 f'y={target_pose.pose.position.y:.3f}, '
                                 f'z={target_pose.pose.position.z:.3f}')
            self.get_logger().info('='*60)

            # Use two-step approach: position first, then orientation
            self.get_logger().info('Computing IK for target (two-step)...')
            position_ik, final_ik = self.move_to_pose_two_step(
                self.current_joint_state,
                target_pose
            )

            if final_ik is None:
                self.get_logger().error('IK failed for target position')
                return False

            # Execute both steps
            for step_num, ik_result in enumerate([position_ik, final_ik], 1):
                step_name = "position" if step_num == 1 else "orientation"
                self.get_logger().info(f'Planning trajectory for {step_name} adjustment...')
                trajectory = self.ik_planner.plan_to_joints(ik_result, goal_tolerance=self.goal_tolerance)
                if trajectory is None:
                    self.get_logger().error(f'Planning failed for {step_name} adjustment')
                    return False

                self.get_logger().info(f'Executing {step_name} adjustment...')
                success = await self.execute_trajectory(trajectory.joint_trajectory)
                if not success:
                    self.get_logger().error(f'Execution failed for {step_name} adjustment')
                    return False

            self.get_logger().info('='*60)
            self.get_logger().info('✅ Move to target complete!')
            self.get_logger().info('='*60)

            return True

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            return False


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