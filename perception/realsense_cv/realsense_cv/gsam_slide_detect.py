
import sys
sys.path.append('Grounded-SAM-2')

import argparse
import os
import cv2
import json
import torch
import numpy as np
import supervision as sv
import pycocotools.mask as mask_util
from pathlib import Path
from supervision.draw.color import ColorPalette
from utils.supervision_utils import CUSTOM_COLOR_MAP
from PIL import Image as PILImage
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

import math

class GSAMSlideDetectNode(Node):
    def __init__(self):
        super().__init__('gsam_slide_detect_node')

        # Declare parameters
        self.declare_parameter('input_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/camera/edges_overlay')
        self.declare_parameter('service_name', '/detect_slides')
        self.declare_parameter('grounding_model', "IDEA-Research/grounding-dino-tiny")
        self.declare_parameter('text_prompt', "colored box.")
        self.declare_parameter('sam2_checkpoint', "./models/sam2.1_hiera_large.pt")
        self.declare_parameter('sam2_model_config', "configs/sam2.1/sam2.1_hiera_l.yaml")
        self.declare_parameter('force_cpu', False)

        # Get parameters
        self.grounding_model = self.get_parameter('grounding_model').value
        self.text_prompt = self.get_parameter('text_prompt').value
        self.sam2_checkpoint = self.get_parameter('sam2_checkpoint').value
        self.sam2_model_config = self.get_parameter('sam2_model_config').value
        force_cpu = self.get_parameter('force_cpu').value
        input_topic = self.get_parameter('input_image_topic').value
        output_topic = self.get_parameter('output_image_topic').value
        service_name = self.get_parameter('service_name').value

        self.last_image = None
        self.processing_image = False
        self.device = "cuda" if torch.cuda.is_available() and not force_cpu else "cpu"
        self.get_logger().info(f'Using device: {self.device}')

        torch.autocast(device_type=self.device, dtype=torch.bfloat16).__enter__()

        if torch.cuda.is_available() and torch.cuda.get_device_properties(0).major >= 8:
            # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

        # Subscriber to original camera image
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )

        # Publisher for debug/intermediate images
        self.edge_pub = self.create_publisher(Image, output_topic, 10)

        # Create service
        self.service = self.create_service(
            Trigger,
            service_name,
            self.detect_slides_service_callback
        )

        self.get_logger().info(f'GSAM Slide Detect Service initialized at {service_name}')

    def ros_image_to_cv2(self, msg: Image):
        """
        Convert ROS Image message to OpenCV image without using CvBridge
        """
        # Get image dimensions
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        # Convert bytes to numpy array
        if encoding == 'bgr8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif encoding == 'rgb8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        elif encoding == 'mono8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        elif encoding == '16UC1':
            cv_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
        else:
            self.get_logger().error(f'Unsupported encoding: {encoding}')
            return None

        return cv_image

    def image_callback(self, msg: Image):
        ### CV_image = (720, 1280, 3)
        # Convert ROS Image message to OpenCV image (BGR)
        cv_image = self.ros_image_to_cv2(msg)
        if cv_image is None:
            return
        if self.processing_image is False:
            # Store the last image for service processing
            self.last_image = cv_image

    def detect_slides_service_callback(self, request, response):
        """
        Service callback that processes self.last_image and returns detected slides
        """
        if self.last_image is None:
            response.success = False
            response.message = "No image available. Please wait for camera image."
            self.get_logger().warn('Service called but no image available yet')
            return response

        try:
            # Save last_image temporarily for processing
            self.processing_image = True
            # Detect slides
            detected_slides = self.detect_slides(self.last_image, text_prompt=self.text_prompt)

            response.success = True
            response.message = f"Detected slides at slots: {detected_slides}"
            self.get_logger().info(f'Slide detection complete: {detected_slides}')

        except Exception as e:
            response.success = False
            response.message = f"Error during slide detection: {str(e)}"
            self.get_logger().error(f'Slide detection failed: {str(e)}')
            self.processing_image = False
        self.processing_image = False
        return response
    @staticmethod
    def sort_points(points):
        points = sorted(points, key = lambda x: x[0]**2 + x[1]**2)
        p1 = points[0]
        p4 = points[3]
        points = sorted(points[1:3], key = lambda x: abs(x[0]-p1[0])+abs(x[1]-p1[1]))
        p2 = points[0]
        p3 = points[1]
        return [p1, p2, p3, p4]
    @staticmethod
    def yukai_kernel(points,width=15,numSlots=25):
        h, w = (720,1280) # image size
        p1, p2, p3, p4 = points
        #points = sorted(points, key = lambda x: x[0]**2 + x[1]**2)
        #p1 = points[0]
        #p4 = points[3]
        #points = sorted(points[1:3], key = lambda x: abs(x[0]-p1[0])+abs(x[1]-p1[1]))
        #p2 = points[0]
        #p3 = points[1]

        """
        p1----p2
        \.     \
        \      \
        \.     \
        p3-----p4
        """

        kernels = []
        
        for i in range(1,numSlots+1):
            kernel = np.zeros((h, w), dtype=np.uint8)  # black image
            sx = int(p1[0]+(p3[0]-p1[0])*i/(numSlots+1))
            sy = int(p1[1]+(p3[1]-p1[1])*i/(numSlots+1))
            ex = int(p2[0]+(p4[0]-p2[0])*i/(numSlots+1))
            ey = int(p2[1]+(p4[1]-p2[1])*i/(numSlots+1))
            cv2.line(kernel, (sx, sy), (ex, ey), color=255, thickness=width)
            kernel = kernel.astype(np.float32)
            kernels.append(kernel)

        return kernels

    def gsam_mask(self, image, text=None):
        # build SAM2 image predictor
        sam2_checkpoint = self.sam2_checkpoint
        model_cfg = self.sam2_model_config
        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=self.device)
        sam2_predictor = SAM2ImagePredictor(sam2_model)

        # build grounding dino from huggingface
        model_id = self.grounding_model
        processor = AutoProcessor.from_pretrained(model_id)
        grounding_model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(self.device)

        # setup the input image and text prompt for SAM 2 and Grounding DINO
        # VERY important: text queries need to be lowercased + end with a dot

        image = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        sam2_predictor.set_image(np.array(image.convert("RGB")))

        inputs = processor(images=image, text=text, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = grounding_model(**inputs)

        results = processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            # threshold=0.4,
            text_threshold=0.3,
            target_sizes=[image.size[::-1]]
        )

        """
        Results is a list of dict with the following structure:
        [
            {
                'scores': tensor([0.7969, 0.6469, 0.6002, 0.4220], device='cuda:0'), 
                'labels': ['car', 'tire', 'tire', 'tire'], 
                'boxes': tensor([[  89.3244,  278.6940, 1710.3505,  851.5143],
                                [1392.4701,  554.4064, 1628.6133,  777.5872],
                                [ 436.1182,  621.8940,  676.5255,  851.6897],
                                [1236.0990,  688.3547, 1400.2427,  753.1256]], device='cuda:0')
            }
        ]
        """

        # get the box prompt for SAM 2
        input_boxes = results[0]["boxes"].cpu().numpy()

        masks, scores, logits = sam2_predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_boxes,
            multimask_output=False,
        )
        print("generated mask",masks.shape)
        mask = masks[0]  # (H, W) boolean or 0/1 tensor
        mask = mask.astype(np.uint8) * 255
        return mask
    @staticmethod
    def mask_to_rect(mask):
        # Find outer contours on the filled mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rectangles = []

        cnt = max(contours, key=cv2.contourArea)
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)  # tune 0.01–0.04

        # Only keep shapes with *exactly 4 corners*
        if len(approx) == 4:
            rectangles.append(approx)
        # Draw contours on a copy of the original image
        
        return rectangles

    @staticmethod
    def filter_non_parallel(img, points, angle_thrd=10):
        """
        Filter edges to keep only lines parallel to the line (p1, p2)

        Args:
            img: Edge image
            points: List of 4 corner points [p1, p2, p3, p4]
            angle_thrd: Angle threshold in degrees
        """
        # 1) Ensure single-channel 8-bit
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = img.astype(np.uint8)
        lines = cv2.HoughLines(img, 1, np.pi/180, threshold=120)

        if lines is None:
            return np.zeros_like(img).astype(np.bool)

        # Calculate the reference angle from p1 to p2
        p1, p2, _, _ = points
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        # Calculate angle in radians (note: cv2 uses angle from x-axis)
        reference_angle = np.arctan2(dy, dx)

        # Convert to Hough line theta format (angle of normal to the line)
        # Hough theta is the angle of the normal, so we add pi/2
        target_theta = reference_angle + np.pi/2

        # Normalize to [0, pi] range as Hough uses
        while target_theta < 0:
            target_theta += np.pi
        while target_theta >= np.pi:
            target_theta -= np.pi

        print(f"Reference angle (p1->p2): {np.rad2deg(reference_angle):.2f}°")
        print(f"Target theta for Hough: {np.rad2deg(target_theta):.2f}°")

        filtered_lines = []
        for line in lines:
            rho, theta = line[0]

            # Check if angle is close to target_theta (considering wrapping at 0/pi)
            angle_diff = abs(theta - target_theta)
            # Also check the supplementary angle (theta + pi or theta - pi)
            angle_diff_supp = abs(abs(theta - target_theta) - np.pi)

            min_angle_diff = min(angle_diff, angle_diff_supp)

            if min_angle_diff < np.deg2rad(angle_thrd):
                filtered_lines.append((rho, theta))

        print(f"Filtered {len(filtered_lines)} lines out of {len(lines)} total lines")

        output = np.zeros_like(img)

        for rho, theta in filtered_lines:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho

            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))

            cv2.line(output, (x1, y1), (x2, y2), 255, 1)

        cv2.imwrite("filtered_edges.png", output)
        return output.astype(np.bool)

    def detect_slides(self, img, text_prompt=None, slide_thrd=100):
        """
        Main method to detect slides in an image

        Args:
            img_path: Path to the image file
            text_prompt: Text prompt for object detection (uses self.text_prompt if None)
            slide_thrd: Threshold for slide detection

        Returns:
            List of detected slide slot numbers
        """
        if text_prompt is None:
            text_prompt = self.text_prompt

        test_image = "yellow"
        mask = self.gsam_mask(img, text=text_prompt)
        cv2.imwrite(f"{test_image}_mask.png", mask)

        rectangles = self.mask_to_rect(mask)

        overlay = img.copy()
        cv2.drawContours(overlay, rectangles, -1, (0, 0, 255), thickness=5)
        cv2.imwrite(f"{test_image}_contour.jpg", overlay)

        # --------
        # scan slides

        points = []

        for i in range(4):
            pt1 = rectangles[0][i][0]
            points.append(pt1)
        points = self.sort_points(points)
        kernels = self.yukai_kernel(points)


        # 1. Smooth (reduces noise → better edges)
        blur = cv2.GaussianBlur(img, (5, 5), 1.4)

        # 2. Canny edge detection
        low_thresh  = 40
        high_thresh = 120
        edges = cv2.Canny(blur, low_thresh, high_thresh)
        cv2.imwrite(f"{test_image}_edges.png", edges)

        # 3. Apply mask to edges - keep only edges inside the detected box
        edges_masked = cv2.bitwise_and(edges, mask)
        cv2.imwrite(f"{test_image}_edges_masked.png", edges_masked)

        edges_bool = self.filter_non_parallel(edges_masked, points, angle_thrd=10)

        results = []
        for i, kernel in enumerate(kernels):
            kernel_bool = kernel>1e-6
            intersection_count = np.count_nonzero(kernel_bool & edges_bool)
            results.append(intersection_count)

            # Create black and white overlap visualization
            # Shows kernel OR edges (union)
            overlap = kernel_bool | edges_masked
            overlap_path = f"{test_image}_overlap_{i}.png"
            cv2.imwrite(overlap_path, overlap.astype(np.uint8) * 255)

            conv = kernel_bool | edges_bool
            conv_path = f"{test_image}_conv_{i}.png"
            cv2.imwrite(conv_path, conv.astype(np.uint8) * 255)

            self.get_logger().info(f"Slot {i+1}: {intersection_count} edge pixels, saved overlap as {overlap_path}")

        detected_slides = []
        for i, val in enumerate(results):
            is_peak=True
            if i:
                if results[i]<=results[i-1]:
                    is_peak=False
            if i+1<len(results):
                if results[i]<=results[i+1]:
                    is_peak=False
            if is_peak and val>slide_thrd:
                self.get_logger().info(f"Slide detected at slot {i+1}")
                detected_slides.append(i+1)
        
        return detected_slides


def main(args=None):
    rclpy.init(args=args)
    node = GSAMSlideDetectNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
