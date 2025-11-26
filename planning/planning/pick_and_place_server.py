#!/usr/bin/env python3

import asyncio
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from planning_interfaces.srv import PickPlaceService  # Custom service definition


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # Parameters
        self.declare_parameter('planning_group', 'tmr_arm')
        self.declare_parameter('end_effector_link', 'link_6')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('approach_distance', 0.1)  # 10cm approach height
        self.declare_parameter('gripper_open_topic', '/gripper/open')
        self.declare_parameter('gripper_close_topic', '/gripper/close')

        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.approach_distance = self.get_parameter('approach_distance').value

        self.get_logger().info(f'End-effector: {self.end_effector_link}')

        # Use ReentrantCallbackGroup to allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Action client
        self.move_client = ActionClient(
            self, 
            MoveGroup, 
            '/move_action',
            callback_group=self.callback_group
        )
        self.move_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup')

        # Gripper publishers (adjust to your gripper interface)
        self.gripper_open_pub = self.create_publisher(
            Trigger.Request, 
            self.get_parameter('gripper_open_topic').value, 
            10
        )
        self.gripper_close_pub = self.create_publisher(
            Trigger.Request,
            self.get_parameter('gripper_close_topic').value,
            10
        )

        # Service for pick and place
        self.pick_place_srv = self.create_service(
            PickPlaceService,  # Custom service (see below)
            'pick_and_place',
            self.pick_place_callback,
            callback_group=self.callback_group
        )

        # Alternative: Subscribe to separate topics for pick/place
        self.create_subscription(
            PoseStamped, 
            '/pick_pose', 
            self.pick_callback, 
            10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            PoseStamped, 
            '/place_pose', 
            self.place_callback, 
            10,
            callback_group=self.callback_group
        )

        self.processing = False
        self.picked_object = False

        self.get_logger().info('Ready! Call /pick_and_place service or publish to /pick_pose and /place_pose')

    # === SERVICE-BASED APPROACH (RECOMMENDED) ===
    
    async def pick_place_callback(self, request, response):
        """Service callback for complete pick and place operation"""
        if self.processing:
            response.success = False
            response.message = "Already processing a motion"
            return response

        self.processing = True
        
        try:
            # 1. Move to approach position (above pick)
            self.get_logger().info('📍 Moving to approach position...')
            approach_pose = self.get_approach_pose(request.pick_pose)
            if not await self.move_to_pose(approach_pose):
                response.success = False
                response.message = "Failed to reach approach position"
                return response

            # 2. Move down to pick position
            self.get_logger().info('⬇️  Moving to pick position...')
            if not await self.move_to_pose(request.pick_pose):
                response.success = False
                response.message = "Failed to reach pick position"
                return response

            # 3. Close gripper
            self.get_logger().info('🤏 Closing gripper...')
            await self.close_gripper()
            self.picked_object = True

            # 4. Move up to retreat position
            self.get_logger().info('⬆️  Retreating...')
            if not await self.move_to_pose(approach_pose):
                response.success = False
                response.message = "Failed to retreat from pick"
                return response

            # 5. Move to approach position above place
            self.get_logger().info('📍 Moving to place approach...')
            place_approach = self.get_approach_pose(request.place_pose)
            if not await self.move_to_pose(place_approach):
                response.success = False
                response.message = "Failed to reach place approach"
                return response

            # 6. Move down to place position
            self.get_logger().info('⬇️  Moving to place position...')
            if not await self.move_to_pose(request.place_pose):
                response.success = False
                response.message = "Failed to reach place position"
                return response

            # 7. Open gripper
            self.get_logger().info('✋ Opening gripper...')
            await self.open_gripper()
            self.picked_object = False

            # 8. Retreat
            self.get_logger().info('⬆️  Retreating from place...')
            if not await self.move_to_pose(place_approach):
                response.success = False
                response.message = "Failed to retreat from place"
                return response

            self.get_logger().info('✅ Pick and place complete!')
            response.success = True
            response.message = "Pick and place successful"

        except Exception as e:
            self.get_logger().error(f'Exception during pick and place: {e}')
            response.success = False
            response.message = f"Exception: {str(e)}"
        finally:
            self.processing = False

        return response

    # === TOPIC-BASED APPROACH (ALTERNATIVE) ===
    
    def pick_callback(self, msg):
        """Simple pick operation via topic"""
        if self.processing:
            self.get_logger().warn('Already processing')
            return
        
        self.executor.create_task(self.pick_sequence(msg))

    def place_callback(self, msg):
        """Simple place operation via topic"""
        if not self.picked_object:
            self.get_logger().warn('No object picked! Call /pick_pose first')
            return
            
        if self.processing:
            self.get_logger().warn('Already processing')
            return
        
        self.executor.create_task(self.place_sequence(msg))

    async def pick_sequence(self, pick_pose):
        """Execute pick sequence"""
        self.processing = True
        try:
            # Approach
            approach = self.get_approach_pose(pick_pose)
            if not await self.move_to_pose(approach):
                return
            
            # Pick
            if not await self.move_to_pose(pick_pose):
                return
            
            # Close gripper
            await self.close_gripper()
            self.picked_object = True
            
            # Retreat
            await self.move_to_pose(approach)
            
            self.get_logger().info('✅ Pick complete!')
        finally:
            self.processing = False

    async def place_sequence(self, place_pose):
        """Execute place sequence"""
        self.processing = True
        try:
            # Approach
            approach = self.get_approach_pose(place_pose)
            if not await self.move_to_pose(approach):
                return
            
            # Place
            if not await self.move_to_pose(place_pose):
                return
            
            # Open gripper
            await self.open_gripper()
            self.picked_object = False
            
            # Retreat
            await self.move_to_pose(approach)
            
            self.get_logger().info('✅ Place complete!')
        finally:
            self.processing = False

    # === HELPER METHODS ===

    def get_approach_pose(self, target_pose):
        """Get approach pose (above target by approach_distance)"""
        approach = PoseStamped()
        approach.header = target_pose.header
        approach.pose = target_pose.pose
        approach.pose.position.z += self.approach_distance
        return approach

    async def move_to_pose(self, target_pose):
        """Move robot to target pose - returns True on success"""
        try:
            self.get_logger().info(f'Moving to: x={target_pose.pose.position.x:.3f}, '
                                 f'y={target_pose.pose.position.y:.3f}, '
                                 f'z={target_pose.pose.position.z:.3f}')

            # Create goal
            goal = MoveGroup.Goal()
            goal.request = MotionPlanRequest()
            goal.request.workspace_parameters.header.frame_id = self.base_frame
            goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
            goal.request.group_name = self.planning_group
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 5.0
            goal.request.max_velocity_scaling_factor = 0.1
            goal.request.max_acceleration_scaling_factor = 0.1

            # Position constraint
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = target_pose.header.frame_id
            pos_constraint.link_name = self.end_effector_link
            pos_constraint.constraint_region.primitives.append(SolidPrimitive())
            pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
            pos_constraint.constraint_region.primitives[0].dimensions = [0.01]
            pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            pos_constraint.weight = 1.0

            # Orientation constraint
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = target_pose.header.frame_id
            ori_constraint.link_name = self.end_effector_link
            ori_constraint.orientation = target_pose.pose.orientation
            ori_constraint.absolute_x_axis_tolerance = 0.1
            ori_constraint.absolute_y_axis_tolerance = 0.1
            ori_constraint.absolute_z_axis_tolerance = 0.1
            ori_constraint.weight = 1.0

            # Add constraints
            goal.request.goal_constraints.append(Constraints())
            goal.request.goal_constraints[0].position_constraints.append(pos_constraint)
            goal.request.goal_constraints[0].orientation_constraints.append(ori_constraint)

            # Planning options
            goal.planning_options = PlanningOptions()
            goal.planning_options.plan_only = False
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True

            # Send goal
            goal_handle = await self.move_client.send_goal_async(goal)

            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False

            result = await goal_handle.get_result_async()

            if result.result.error_code.val == 1:
                self.get_logger().info('✓ Motion success!')
                return True
            else:
                self.get_logger().error(f'Failed with error code: {result.result.error_code.val}')
                return False

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            return False

    async def open_gripper(self):
        """Open gripper - implement based on your gripper interface"""
        self.get_logger().info('Opening gripper...')
        # TODO: Implement your gripper opening logic
        # Examples:
        # - Publish to a topic
        # - Call a service
        # - Send action goal
        # await asyncio.sleep(0.5)  # Simulate gripper action
        time.sleep(0.5)  # Simulate gripper action

    async def close_gripper(self):
        """Close gripper - implement based on your gripper interface"""
        self.get_logger().info('Closing gripper...')
        # TODO: Implement your gripper closing logic
        # await asyncio.sleep(0.5)  # Simulate gripper action
        time.sleep(0.5)  # Simulate gripper action


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