#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from planning_interfaces.srv import PickPlaceService, MoveToTarget, ContinuousPickPlace
from std_srvs.srv import SetBool

from planning.ik import IKPlanner
from realsense_cv.slide_detector import SlideDetector

import numpy as np
from scipy.spatial.transform import Rotation as R
import asyncio
import time


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

        # IK Planner
        self.ik_planner = IKPlanner()
        self.get_logger().info('IK Planner initialized')

        # Slide Detector
        self.slide_detector = SlideDetector()
        self.get_logger().info('Slide Detector initialized')

        # Joint state tracking
        self.current_joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1
        )

        # Action client for trajectory execution
        self.exec_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/tmr_arm_controller/follow_joint_trajectory',
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

        # Service for continuous pick and place
        self.continuous_srv = self.create_service(
            ContinuousPickPlace,
            'continuous_pick_place',
            self.continuous_pick_place_callback,
            callback_group=self.callback_group
        )

        self.processing = False

        self.get_logger().info('Ready! Pick-and-Place node initialized')
    def normalize_quaternion(self, quat):
        """
        Normalize a quaternion to ensure it has unit length.

        Args:
            quat: Array-like [x, y, z, w] or geometry_msgs Quaternion
f
        Returns:
            Normalized quaternion as numpy array [x, y, z, w]
        """
        if hasattr(quat, 'x'):
            # geometry_msgs.msg.Quaternion
            q = np.array([quat.x, quat.y, quat.z, quat.w])
        else:
            q = np.array(quat)

        norm = np.linalg.norm(q)
        if norm < 1e-10:
            self.get_logger().warn('Quaternion norm too small, using identity quaternion')
            return np.array([0.0, 0.0, 0.0, 1.0])

        return q / norm

    def print_joint_state(self, prev_js,js):
        self.get_logger().info('prev Joint State:')
        for n,p in zip(prev_js.name,prev_js.position):
            self.get_logger().info(f'  {n}: {p:.4f}')
        self.get_logger().info('new Joint State:')
        for n,p in zip(js.name,js.position):
            self.get_logger().info(f'  {n}: {p:.4f}')

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
            # Normalize target pose quaternion
            target_quat_norm = self.normalize_quaternion(request.target_pose.pose.orientation)
            request.target_pose.pose.orientation.x = target_quat_norm[0]
            request.target_pose.pose.orientation.y = target_quat_norm[1]
            request.target_pose.pose.orientation.z = target_quat_norm[2]
            request.target_pose.pose.orientation.w = target_quat_norm[3]

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

            # Normalize quaternions for pick and place poses
            pick_quat_norm = self.normalize_quaternion(pick_pose.pose.orientation)
            pick_pose.pose.orientation.x = pick_quat_norm[0]
            pick_pose.pose.orientation.y = pick_quat_norm[1]
            pick_pose.pose.orientation.z = pick_quat_norm[2]
            pick_pose.pose.orientation.w = pick_quat_norm[3]

            place_quat_norm = self.normalize_quaternion(place_pose.pose.orientation)
            place_pose.pose.orientation.x = place_quat_norm[0]
            place_pose.pose.orientation.y = place_quat_norm[1]
            place_pose.pose.orientation.z = place_quat_norm[2]
            place_pose.pose.orientation.w = place_quat_norm[3]

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
                self.get_logger().info('Pick and Place Complete!')
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

    async def continuous_pick_place_callback(self, request, response):
        """
        Executes pick-and-place cycles until no more slides are detected.
        """
        
        if self.processing:
            response.success = False
            response.message = "Already processing"
            return response
        
        if self.current_joint_state is None:
            response.success = False
            response.message = "No joint state available"
            return response
        
        self.processing = True
        slides_picked = 0

        # Extract parameters
        pick_scan = request.pick_scan_pose
        place_scan = request.place_scan_pose
        pick_dist = request.pick_distance
        retreat_dist = request.retreat_distance
        place_dist = request.place_distance

        # Normalize quaternions for scan poses
        pick_scan_quat_norm = self.normalize_quaternion(pick_scan.pose.orientation)
        pick_scan.pose.orientation.x = pick_scan_quat_norm[0]
        pick_scan.pose.orientation.y = pick_scan_quat_norm[1]
        pick_scan.pose.orientation.z = pick_scan_quat_norm[2]
        pick_scan.pose.orientation.w = pick_scan_quat_norm[3]

        place_scan_quat_norm = self.normalize_quaternion(place_scan.pose.orientation)
        place_scan.pose.orientation.x = place_scan_quat_norm[0]
        place_scan.pose.orientation.y = place_scan_quat_norm[1]
        place_scan.pose.orientation.z = place_scan_quat_norm[2]
        place_scan.pose.orientation.w = place_scan_quat_norm[3]

        self.get_logger().info('='*60)
        self.get_logger().info('CONTINUOUS PICK-AND-PLACE STARTED')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Pick distance: {pick_dist}m (TODO: TUNE)')
        self.get_logger().info(f'Retreat distance: {retreat_dist}m (TODO: TUNE)')
        self.get_logger().info(f'Place distance: {place_dist}m (TODO: TUNE)')
        self.get_logger().info('='*60)
        
        try:
            while True:
                self.get_logger().info('')
                self.get_logger().info('='*60)
                self.get_logger().info(f'CYCLE {slides_picked + 1}')
                self.get_logger().info('='*60)
                
                # ========================================
                # STEP 1: Move to pick scan pose
                # ========================================
                self.get_logger().info('Step 1: Moving to pick scan pose...')
                if not await self.move_to_target(pick_scan):
                    response.success = False
                    response.message = "Failed to reach pick scan pose"
                    response.slides_picked = slides_picked
                    return response
                
                # Brief pause for camera/marker detection to stabilize
                time.sleep(0.5)
                
                # ========================================
                # STEP 2: Detect slide
                # ========================================
                self.get_logger().info('Step 2: Detecting slide...')
                slide_pose = self.slide_detector.wait_for_slide(timeout=10.0)
                
                if slide_pose is None:
                    self.get_logger().info('No more slides detected - operation complete')
                    break
                
                self.get_logger().info(f'Slide detected at ({slide_pose.pose.position.x:.3f}, '
                                     f'{slide_pose.pose.position.y:.3f}, '
                                     f'{slide_pose.pose.position.z:.3f})')
                
                # ========================================
                # BUILD JOB QUEUE FOR THIS SLIDE
                # ========================================
                self.get_logger().info('Building job queue for this slide...')
                
                job_queue = []
                current_state = self.current_joint_state
                
                # Job 1: Align with slide (XY + orientation, keep current Z from pick_scan_pose)
                self.get_logger().info('  Job 1/9: Computing align pose ...')
                align_pose = self.compute_alignment_pose(slide_pose)
                
                ik_align = self.ik_planner.compute_ik(
                    current_state,
                    align_pose.pose.position.x,
                    align_pose.pose.position.y,
                    align_pose.pose.position.z,
                    align_pose.pose.orientation.x,
                    align_pose.pose.orientation.y,
                    align_pose.pose.orientation.z,
                    align_pose.pose.orientation.w
                )
                if ik_align is None:
                    self.get_logger().error('IK failed for align pose - skipping this slide')
                    continue
                job_queue.append(ik_align)
                current_state = ik_align
                
                # Job 2: Lower for grasp
                self.get_logger().info(f'  Job 2/9: Computing lower ({pick_dist}m) - TODO: TUNE')
                lower_pose = self.offset_pose_z(align_pose, -pick_dist)
                
                ik_lower = self.ik_planner.compute_ik(
                    current_state,
                    lower_pose.pose.position.x,
                    lower_pose.pose.position.y,
                    lower_pose.pose.position.z,
                    lower_pose.pose.orientation.x,
                    lower_pose.pose.orientation.y,
                    lower_pose.pose.orientation.z,
                    lower_pose.pose.orientation.w
                )
                if ik_lower is None:
                    self.get_logger().error('IK failed for lower pose - skipping this slide')
                    continue
                job_queue.append(ik_lower)
                current_state = ik_lower
                
                # Job 3: Close gripper
                self.get_logger().info('  Job 3/9: Close gripper')
                job_queue.append('close_gripper')
                
                # Job 4: Lift after grasp
                self.get_logger().info(f'  Job 4/9: Computing lift ({retreat_dist}m) - TODO: TUNE')
                lift_pose = self.offset_pose_z(lower_pose, retreat_dist)
                
                ik_lift = self.ik_planner.compute_ik(
                    current_state,
                    lift_pose.pose.position.x,
                    lift_pose.pose.position.y,
                    lift_pose.pose.position.z,
                    lift_pose.pose.orientation.x,
                    lift_pose.pose.orientation.y,
                    lift_pose.pose.orientation.z,
                    lift_pose.pose.orientation.w
                )
                if ik_lift is None:
                    self.get_logger().error('IK failed for lift pose - skipping this slide')
                    continue
                job_queue.append(ik_lift)
                current_state = ik_lift
                
                # Job 5: Move to place scan pose
                self.get_logger().info('  Job 5/9: Computing place scan pose...')
                ik_place_scan = self.ik_planner.compute_ik(
                    current_state,
                    place_scan.pose.position.x,
                    place_scan.pose.position.y,
                    place_scan.pose.position.z,
                    place_scan.pose.orientation.x,
                    place_scan.pose.orientation.y,
                    place_scan.pose.orientation.z,
                    place_scan.pose.orientation.w
                )
                if ik_place_scan is None:
                    self.get_logger().error('IK failed for place scan pose - skipping this slide')
                    continue
                job_queue.append(ik_place_scan)
                current_state = ik_place_scan
                
                # Job 6: Align with target (for now, same as place_scan)
                self.get_logger().info('  Job 6/9: Target align (same as scan for now)')
                # Already there from job 5
                
                # Job 7: Lower for place
                self.get_logger().info(f'  Job 7/9: Computing place lower ({place_dist}m) - TODO: TUNE')
                place_lower_pose = self.offset_pose_z(place_scan, -place_dist)
                
                ik_place_lower = self.ik_planner.compute_ik(
                    current_state,
                    place_lower_pose.pose.position.x,
                    place_lower_pose.pose.position.y,
                    place_lower_pose.pose.position.z,
                    place_lower_pose.pose.orientation.x,
                    place_lower_pose.pose.orientation.y,
                    place_lower_pose.pose.orientation.z,
                    place_lower_pose.pose.orientation.w
                )
                if ik_place_lower is None:
                    self.get_logger().error('IK failed for place lower - skipping this slide')
                    continue
                job_queue.append(ik_place_lower)
                current_state = ik_place_lower
                
                # Job 8: Open gripper
                self.get_logger().info('  Job 8/9: Open gripper')
                job_queue.append('open_gripper')
                
                # Job 9: Lift after place
                self.get_logger().info(f'  Job 9/9: Computing final lift ({retreat_dist}m)...')
                final_lift_pose = self.offset_pose_z(place_lower_pose, retreat_dist)
                
                ik_final_lift = self.ik_planner.compute_ik(
                    current_state,
                    final_lift_pose.pose.position.x,
                    final_lift_pose.pose.position.y,
                    final_lift_pose.pose.position.z,
                    final_lift_pose.pose.orientation.x,
                    final_lift_pose.pose.orientation.y,
                    final_lift_pose.pose.orientation.z,
                    final_lift_pose.pose.orientation.w
                )
                if ik_final_lift is None:
                    self.get_logger().error('IK failed for final lift - skipping this slide')
                    continue
                job_queue.append(ik_final_lift)
                
                # ========================================
                # EXECUTE QUEUE
                # ========================================
                self.get_logger().info(f'All IK solutions found! Queue has {len(job_queue)} jobs')
                self.get_logger().info('Executing queue...')
                
                success = await self.execute_job_queue(job_queue)
                
                if success:
                    slides_picked += 1
                    self.get_logger().info(f'Slide {slides_picked} complete!')
                else:
                    self.get_logger().error('Execution failed for this slide')
                
                # Brief pause before next slide
                time.sleep(1.0)
            
            # All done
            self.get_logger().info('='*60)
            self.get_logger().info(f'OPERATION COMPLETE - Successfully picked {slides_picked} slides')
            self.get_logger().info('='*60)
            response.success = True
            response.message = f"Successfully picked {slides_picked} slides"
            response.slides_picked = slides_picked
            
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            import traceback
            traceback.print_exc()
            response.success = False
            response.message = f"Exception after {slides_picked} slides: {str(e)}"
            response.slides_picked = slides_picked
        
        finally:
            self.processing = False
        
        return response

    # MODIFIED: Updated function signature and implementation
    def compute_alignment_pose(self, slide_pose, current_z=None):
        """
        Compute flange pose aligned with slide.
        
        Position: slide XY, keep current Z height (from pick_scan_pose)
        Orientation: Y-axis parallel to slide X-axis, Z-axis points down
        
        Args:
            slide_pose: PoseStamped of detected slide
            current_z: Current Z height (if None, gets from robot TF)
            
        Returns:
            PoseStamped for flange alignment pose
        """
        # ADDED: Get current Z from robot TF if not provided
        if current_z is None:
            try:
                transform = self.ik_planner.tf_buffer.lookup_transform(
                    'base',
                    'link_6',
                    rclpy.time.Time()
                )
                current_z = transform.transform.translation.z
                self.get_logger().info(f'Using current Z height: {current_z:.3f}m')
            except:
                self.get_logger().warn('Cannot get current Z, using slide Z + 0.1m as fallback')
                current_z = slide_pose.pose.position.z + 0.10
        
        # Extract slide orientation
        slide_quat = np.array([
            slide_pose.pose.orientation.x,
            slide_pose.pose.orientation.y,
            slide_pose.pose.orientation.z,
            slide_pose.pose.orientation.w
        ])
        
        slide_rot = R.from_quat(slide_quat)
        slide_x_axis = slide_rot.as_matrix()[:, 0]  # First column = X-axis
        
        # Build flange orientation
        # Z-axis: pointing down (world -Z)
        flange_z = np.array([0, 0, -1])
        
        # Y-axis: aligned with slide X-axis, projected to horizontal
        flange_y = slide_x_axis.copy()
        flange_y[2] = 0.0  # Project to XY plane
        
        norm_y = np.linalg.norm(flange_y)
        if norm_y < 1e-6:
            # Slide is vertical - fall back to world Y
            flange_y = np.array([0, 1, 0])
        else:
            flange_y /= norm_y
        
        # X-axis: Y cross Z (right-hand rule)
        flange_x = np.cross(flange_y, flange_z)
        flange_x /= np.linalg.norm(flange_x)
        
        # Recompute Y to ensure orthogonality
        flange_y = np.cross(flange_z, flange_x)
        
        # Build rotation matrix
        flange_rot_mat = np.column_stack([flange_x, flange_y, flange_z])
        flange_rot = R.from_matrix(flange_rot_mat)
        flange_quat = flange_rot.as_quat()  # [x, y, z, w]

        # Normalize quaternion
        flange_quat_norm = self.normalize_quaternion(flange_quat)

        # Build pose
        align_pose = PoseStamped()
        align_pose.header.frame_id = 'base'
        align_pose.header.stamp = self.get_clock().now().to_msg()
        align_pose.pose.position.x = slide_pose.pose.position.x
        align_pose.pose.position.y = slide_pose.pose.position.y
        align_pose.pose.position.z = current_z  # Use current Z
        align_pose.pose.orientation.x = flange_quat_norm[0]
        align_pose.pose.orientation.y = flange_quat_norm[1]
        align_pose.pose.orientation.z = flange_quat_norm[2]
        align_pose.pose.orientation.w = flange_quat_norm[3]

        return align_pose
    
    def offset_pose_z(self, pose, delta_z):
        """
        Create new pose with Z offset.
        
        Args:
            pose: Original PoseStamped
            delta_z: Z offset in meters (positive = up, negative = down)
            
        Returns:
            New PoseStamped with Z offset applied
        """
        new_pose = PoseStamped()
        new_pose.header = pose.header
        new_pose.pose.position.x = pose.pose.position.x
        new_pose.pose.position.y = pose.pose.position.y
        new_pose.pose.position.z = pose.pose.position.z + delta_z
        new_pose.pose.orientation = pose.pose.orientation
        return new_pose

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
                
                self.print_joint_state(self.current_joint_state, job)
                    
            elif job == 'close_gripper':
                self.get_logger().info('Closing gripper...')
                if not await self.control_gripper(True):
                    self.get_logger().error('Failed to close gripper')
                    return False

            elif job == 'open_gripper':
                self.get_logger().info('Opening gripper...')
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

    async def move_to_target(self, target_pose):
        """Move end effector to target position using IK"""
        if self.current_joint_state is None:
            self.get_logger().error('No joint state available')
            return False

        try:
            self.get_logger().info('='*60)
            self.get_logger().info('Moving to target position (IK-based)')
            self.get_logger().info(f'Target: x={target_pose.pose.position.x:.3f}, '
                                 f'y={target_pose.pose.position.y:.3f}, '
                                 f'z={target_pose.pose.position.z:.3f}')
            self.get_logger().info('='*60)

            # Compute IK for target position
            self.get_logger().info('Computing IK for target...')
            ik_result = self.ik_planner.compute_ik(
                self.current_joint_state,
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w
            )

            if ik_result is None:
                self.get_logger().error('IK failed for target position')
                return False

            # Plan to joint configuration
            self.get_logger().info('Planning trajectory...')
            trajectory = self.ik_planner.plan_to_joints(ik_result)
            if trajectory is None:
                self.get_logger().error('Planning failed')
                return False

            # Execute trajectory
            self.get_logger().info('Executing trajectory...')
            success = await self.execute_trajectory(trajectory.joint_trajectory)

            if success:
                self.get_logger().info('='*60)
                self.get_logger().info('Move to target complete!')
                self.get_logger().info('='*60)

            return success

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    
    node.executor = executor
    executor.add_node(node)
    executor.add_node(node.slide_detector)  # Add SlideDetector to executor for TF callbacks

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
