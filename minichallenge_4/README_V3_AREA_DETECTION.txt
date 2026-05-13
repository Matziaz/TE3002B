V3 changes:
- The color detector no longer overlays all noisy mask speckles.
- It filters masks using stronger HSV thresholds, blur, morphology open/close, and contour area.
- It selects only the largest valid blob per color.
- It uses a square bounding box and square area to decide whether the color is close/relevant.
- It publishes /detected_color_area as std_msgs/Float32.
- If the selected square area is below min_square_area, the detector publishes unknown.

Main tuning file:
  config/color_detector_params.yaml

Most important parameters to tune on the robot:
  min_contour_area
  min_square_area
  red/yellow/green HSV limits
  dominance_ratio

Recommended test commands:
  ros2 topic hz /video_source/raw
  ros2 topic echo /detected_color
  ros2 topic echo /detected_color_area
  ros2 topic echo /traffic_state
  rqt_image_view  # select /color_debug_image
