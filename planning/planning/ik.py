import rclpy
import math
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint, RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


class IKPlanner(Node):
    def __init__(self):
        super().__init__('ik_planner')

        # ---- Clients ----
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        for srv, name in [(self.ik_client, 'compute_ik'),
                          (self.plan_client, 'plan_kinematic_path'),
                          (self.fk_client, 'compute_fk')]:
            while not srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for /{name} service...')

        # ---- Default joint limits for TM12 ----
        self.default_joint_limits = {
            # 'joint_2': (math.radians(-70), math.radians(0)),  # Limit joint_2
            # Add more if needed:
            # 'joint_1': (math.radians(-180), math.radians(180)),
        }

    def compute_ik(self, current_joint_state, x, y, z,
                   qx=1.0, qy=0.0, qz=0.0, qw=0.0):  # Changed default: gripper pointing down
        """
        Compute IK for TM12 robot
        Default quaternion: pointing down (180° rotation around X-axis)
        """
        pose = PoseStamped()
        pose.header.frame_id = 'base'  # TM12 uses 'base' not 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.group_name = 'tmr_arm'  # Changed from 'ur_manipulator'
        ik_req.ik_request.pose_stamped = pose
        ik_req.ik_request.robot_state.joint_state = current_joint_state
        ik_req.ik_request.ik_link_name = 'link_6'  # Changed from 'tool0'
        ik_req.ik_request.timeout = Duration(sec=5, nanosec=0)  # Increased timeout

        future = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('IK service failed.')
            return None

        result = future.result()
        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().error(f'IK failed, code: {result.error_code.val}')
            return None

        self.get_logger().info('IK solution found.')
        return result.solution.joint_state

    def compute_fk(self, joint_state, fk_link_names=['link_6']):
        """
        Compute Forward Kinematics for TM12 robot

        Args:
            joint_state: Current joint configuration
            fk_link_names: List of link names to compute FK for (default: ['link_6'])

        Returns:
            PoseStamped of the end-effector, or None if FK fails
        """
        fk_req = GetPositionFK.Request()
        fk_req.header.frame_id = 'base'
        fk_req.fk_link_names = fk_link_names
        fk_req.robot_state.joint_state = joint_state

        future = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('FK service failed.')
            return None

        result = future.result()
        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().error(f'FK failed, code: {result.error_code.val}')
            return None

        if len(result.pose_stamped) == 0:
            self.get_logger().error('FK returned no poses.')
            return None

        self.get_logger().info('FK solution found.')
        return result.pose_stamped[0]  # Return first pose (link_6)

    def plan_to_joints(self, target_joint_state, start_joint_state=None, custom_joint_limits=None, goal_tolerance=0.01):
        """
        Plan motion to joint configuration for TM12

        Args:
            target_joint_state: Target joint configuration
            start_joint_state: Starting joint state (recommended to avoid ambiguity)
            custom_joint_limits: Optional joint limits for path constraints
            goal_tolerance: Goal tolerance in radians (default: 0.01 rad ≈ 0.57°)
                           Smaller = more precise, Larger = more lenient
        """
        limits = custom_joint_limits if custom_joint_limits else self.default_joint_limits
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'tmr_arm'  # Changed
        req.motion_plan_request.allowed_planning_time = 5.0  # Increased
        req.motion_plan_request.planner_id = "RRTstarkConfigDefault"  # Better planner
        req.motion_plan_request.num_planning_attempts = 10  # More attempts

        # Set explicit start state to avoid planner ambiguity
        if start_joint_state is not None:
            req.motion_plan_request.start_state.is_diff = False
            req.motion_plan_request.start_state.joint_state = start_joint_state
        else:
            # Fallback: let planner use current state in planning scene
            req.motion_plan_request.start_state.is_diff = True

        # velocity and acceleration scaling
        req.motion_plan_request.max_velocity_scaling_factor = 0.2
        req.motion_plan_request.max_acceleration_scaling_factor = 0.2   

        # Apply joint limits as path constraints
        if limits:
            
            
            path_constraints = Constraints()
            
            for joint_name, (min_pos, max_pos) in limits.items():
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = (min_pos + max_pos) / 2.0
                joint_constraint.tolerance_above = max_pos - joint_constraint.position
                joint_constraint.tolerance_below = joint_constraint.position - min_pos
                joint_constraint.weight = 1.0
                
                path_constraints.joint_constraints.append(joint_constraint)
                
                self.get_logger().info(
                    f'Limiting {joint_name}: [{math.degrees(min_pos):.1f}°, {math.degrees(max_pos):.1f}°]'
                )
            
            req.motion_plan_request.path_constraints = path_constraints

        goal_constraints = Constraints()
        for name, pos in zip(target_joint_state.name, target_joint_state.position):
            goal_constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=pos,
                    tolerance_above=goal_tolerance,
                    tolerance_below=goal_tolerance,
                    weight=1.0
                )
            )

        req.motion_plan_request.goal_constraints.append(goal_constraints)
        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Planning service failed.')
            return None

        result = future.result()
        if result.motion_plan_response.error_code.val != 1:
            self.get_logger().error(f'Planning failed with code: {result.motion_plan_response.error_code.val}')
            return None

        self.get_logger().info('Motion plan computed successfully.')
        return result.motion_plan_response.trajectory