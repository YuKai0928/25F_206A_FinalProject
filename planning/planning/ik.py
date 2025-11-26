import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


class IKPlanner(Node):
    def __init__(self):
        super().__init__('ik_planner')

        # ---- Clients ----
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        for srv, name in [(self.ik_client, 'compute_ik'),
                          (self.plan_client, 'plan_kinematic_path')]:
            while not srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for /{name} service...')

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

    def plan_to_joints(self, target_joint_state):
        """
        Plan motion to joint configuration for TM12
        """
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'tmr_arm'  # Changed
        req.motion_plan_request.allowed_planning_time = 5.0  # Increased
        req.motion_plan_request.planner_id = "RRTstarkConfigDefault"  # Better planner
        req.motion_plan_request.num_planning_attempts = 10  # More attempts
        req.motion_plan_request.start_state.is_diff = True

        # velocity and acceleration scaling
        req.motion_plan_request.max_velocity_scaling_factor = 0.2
        req.motion_plan_request.max_acceleration_scaling_factor = 0.2   

        goal_constraints = Constraints()
        for name, pos in zip(target_joint_state.name, target_joint_state.position):
            goal_constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=pos,
                    tolerance_above=0.01,
                    tolerance_below=0.01,
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