#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from shape_msgs.msg import SolidPrimitive


class SimpleMove(Node):
    def __init__(self):
        super().__init__('simple_move')

        # Parameters
        self.declare_parameter('planning_group', 'tmr_arm')
        self.declare_parameter('end_effector_link', 'link_6')
        self.declare_parameter('base_frame', 'base')

        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value

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

        # Subscribe to target pose
        self.create_subscription(
            PoseStamped, 
            '/target_pose', 
            self.pose_callback, 
            10,
            callback_group=self.callback_group
        )

        self.processing = False

        self.get_logger().info('Ready! Publish to /target_pose to move')

    def pose_callback(self, msg):
        """Move to target pose - only if not already moving"""
        self.get_logger().info(f'📥 Received pose request (processing={self.processing})')

        if self.processing:
            self.get_logger().warn('⏸️  Already processing a motion, skipping this pose')
            return

        self.get_logger().info('✅ Starting motion')
        # Create async task instead of thread
        self.executor.create_task(self.move_to_pose(msg))

    async def move_to_pose(self, target_pose):
        """Move robot to target pose"""
        if self.processing:
            return
            
        self.processing = True
        
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
            pos_constraint.constraint_region.primitives[0].dimensions = [0.01]  # Relaxed
            pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            pos_constraint.weight = 1.0

            # Orientation constraint
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = target_pose.header.frame_id
            ori_constraint.link_name = self.end_effector_link
            ori_constraint.orientation = target_pose.pose.orientation
            ori_constraint.absolute_x_axis_tolerance = 0.1  # Relaxed
            ori_constraint.absolute_y_axis_tolerance = 0.1  # Relaxed
            ori_constraint.absolute_z_axis_tolerance = 0.1  # Relaxed
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
            self.get_logger().info('Sending goal...')
            goal_handle = await self.move_client.send_goal_async(goal)

            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return

            self.get_logger().info('Planning and executing...')
            result = await goal_handle.get_result_async()

            if result.result.error_code.val == 1:
                self.get_logger().info('✓ Success!')
            else:
                self.get_logger().error(f'Failed with error code: {result.result.error_code.val}')

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        finally:
            self.processing = False
            self.get_logger().info('Ready for next pose')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMove()
    executor = MultiThreadedExecutor(num_threads=4)
    
    # Store executor reference in node so callback can use it
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