# Perception Note

## Overview

Goal: get the precise position of a glass slide via Inter Realsense (RGB+D)

## Difficulties

•	RGB image: The interior of the slide is almost invisible; only the edges and specular highlights show up reliably.
•	Depth (RealSense): Most RealSense models struggle with transparent objects. Often:
    •	Depth on the glass surface is missing or noisy.
    •	The sensor “sees through” the glass and returns depth of whatever is behind it

## Plan

- Gaussian Blur to avoid noise
- Edge detection (Canny)
- Search the rectangle with similar ratio
    - findContours -> filter contours:
    - HoughLinesP -> group 4 lines into a quadrilateral
- Rectangle Detection:
    - Opposite sides are roughly parallel.
    - Angles near 90° (if slide is not too tilted).
    - Aspect ratio ≈ (physical slide ratio), e.g., 1in x 3in (from Amazon page)
- 3D position:
    - Depth Camera (Not sure if works transparent)
    - Slide Case: with the information of case size and the index of slide slot, we can get the position of target slide relative to the case.
    - PnP (no depth needed)(same to the cone detect): with the actual size of the slide and camera intrinsic (from RealSense SDK)
- Complementary Strategies:
    - An aruco marker on one side of the glass slide.
    - Color marker on the side of the glass slide.

## Resources:

### Canny Edge Detection

It computes derivatives/apply sobel filters to capture edges. No Neural Network needed!

- Noise reduction (Gaussian blur)
- Gradient computation (Sobel filters)
    - Compute approximate derivatives in x and y directions
    - \theta = \arctan2(G_y, G_x)
- Non-maximum suppression (thinning the edges)
- Hysteresis thresholding (clean up & connect)