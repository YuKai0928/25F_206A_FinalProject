#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MotionPlanRequest,
    PlanningOptions,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import copy


class PickPlanner(Node):
    def __init__(self):
        super().__init__('pick_planner')

        # Declare parameters
        self.declare_parameter('robot_name', 'tm12')
        self.declare_parameter('planning_group', 'tmr_arm')
        self.declare_parameter('end_effector_link', 'flange')
        self.declare_parameter('pose_topic', '/target_pose')
        self.declare_parameter('pre_grasp_offset', 0.15)
        self.declare_parameter('post_grasp_offset', 0.20)
        self.declare_parameter('base_frame', 'base')

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.pre_grasp_offset = self.get_parameter('pre_grasp_offset').value
        self.post_grasp_offset = self.get_parameter('post_grasp_offset').value
        self.base_frame = self.get_parameter('base_frame').value

        self.get_logger().info(f'Initializing PickPlanner for {self.robot_name}...')

        # Initialize MoveGroup action client
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action server connected')

        # Subscribe to target pose
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(f'Subscribed to {self.pose_topic}')
        self.get_logger().info('PickPlanner node initialized and ready')

    def pose_callback(self, msg):
        """Callback for receiving target pose in base_link frame"""
        self.get_logger().info(f'Received target pose in frame: {msg.header.frame_id}')

        # Verify the frame is base_link
        if msg.header.frame_id != 'base_link':
            self.get_logger().warn(f'Expected base_link frame but got {msg.header.frame_id}')

        # Execute pick sequence
        self.execute_pick_sequence(msg)

    def execute_pick_sequence(self, target_pose):
        """Execute the complete pick sequence"""
        self.get_logger().info('Starting pick sequence...')

        try:
            # Step 1: Move to pre-grasp pose
            self.get_logger().info('Step 1: Moving to pre-grasp pose')
            pre_grasp_pose = self.calculate_pre_grasp_pose(target_pose)
            if not self.move_to_pose(pre_grasp_pose):
                self.get_logger().error('Failed to reach pre-grasp pose')
                return False

            # Step 2: Move to grasp pose
            self.get_logger().info('Step 2: Moving to grasp pose')
            if not self.move_to_pose(target_pose):
                self.get_logger().error('Failed to reach grasp pose')
                return False

            # Step 3: Close gripper (implement gripper control separately)
            self.get_logger().info('Step 3: Closing gripper')
            self.close_gripper()

            # Step 4: Lift object
            self.get_logger().info('Step 4: Lifting object')
            post_grasp_pose = self.calculate_post_grasp_pose(target_pose)
            if not self.move_to_pose(post_grasp_pose):
                self.get_logger().error('Failed to lift object')
                return False

            self.get_logger().info('Pick sequence completed successfully!')
            return True

        except Exception as e:
            self.get_logger().error(f'Error during pick sequence: {str(e)}')
            return False

    def calculate_pre_grasp_pose(self, target_pose):
        """Calculate pre-grasp pose with offset above target"""
        pre_grasp = PoseStamped()
        pre_grasp.header = target_pose.header
        pre_grasp.pose = target_pose.pose

        # Add offset in Z direction (assuming vertical approach)
        pre_grasp.pose.position.z += self.pre_grasp_offset

        return pre_grasp

    def calculate_post_grasp_pose(self, target_pose):
        """Calculate post-grasp pose to lift object"""
        post_grasp = PoseStamped()
        post_grasp.header = target_pose.header
        post_grasp.pose = target_pose.pose

        # Add offset in Z direction to lift
        post_grasp.pose.position.z += self.post_grasp_offset

        return post_grasp

    def move_to_pose(self, target_pose):
        """Plan and execute motion to target pose using MoveGroup action"""
        try:
            # Create motion plan request
            goal_msg = MoveGroup.Goal()
            goal_msg.request = MotionPlanRequest()
            goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
            goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
            goal_msg.request.group_name = self.planning_group
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1

            # Set pose constraint
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = target_pose.header.frame_id
            position_constraint.link_name = self.end_effector_link
            position_constraint.target_point_offset.x = 0.0
            position_constraint.target_point_offset.y = 0.0
            position_constraint.target_point_offset.z = 0.0

            # Define constraint region
            position_constraint.constraint_region.primitives.append(SolidPrimitive())
            position_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
            position_constraint.constraint_region.primitives[0].dimensions = [0.001]
            position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            position_constraint.weight = 1.0

            # Set orientation constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = target_pose.header.frame_id
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = target_pose.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.01
            orientation_constraint.absolute_y_axis_tolerance = 0.01
            orientation_constraint.absolute_z_axis_tolerance = 0.01
            orientation_constraint.weight = 1.0

            # Add constraints to goal
            goal_msg.request.goal_constraints.append(Constraints())
            goal_msg.request.goal_constraints[0].position_constraints.append(position_constraint)
            goal_msg.request.goal_constraints[0].orientation_constraints.append(orientation_constraint)

            # Set planning options
            goal_msg.planning_options = PlanningOptions()
            goal_msg.planning_options.plan_only = False
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

            # Send goal
            self.get_logger().info('Sending motion goal to MoveGroup...')
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by MoveGroup')
                return False

            self.get_logger().info('Goal accepted, executing...')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result().result
            if result.error_code.val != 1:
                self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
                return False

            self.get_logger().info('Motion completed successfully')
            return True

        except Exception as e:
            self.get_logger().error(f'Error in move_to_pose: {str(e)}')
            return False

    def close_gripper(self):
        """Close gripper to grasp object"""
        # TODO: Implement gripper control based on your gripper hardware
        # This might involve publishing to a gripper action server or topic
        self.get_logger().info('Gripper control not implemented - add your gripper interface here')

        # Example: You might use something like:
        # self.gripper_command_pub.publish(GripperCommand(position=0.0))

        # For now, just wait a bit to simulate gripper closing
        import time
        time.sleep(1.0)

    def open_gripper(self):
        """Open gripper to release object"""
        # TODO: Implement gripper control based on your gripper hardware
        self.get_logger().info('Gripper control not implemented - add your gripper interface here')

        import time
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)

    try:
        pick_planner = PickPlanner()
        rclpy.spin(pick_planner)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
