# Enhanced Crate Detection Message with Orientation

std_msgs/Header header

# Crate identification
string color              # "blue", "yellow", or "unknown"
float64 confidence        # Detection confidence [0.0 - 1.0]

# Position (in frame specified by header.frame_id)
float64 x                 # X position (meters)
float64 y                 # Y position (meters)
float64 distance          # Distance from camera (meters)
float64 angle             # Horizontal angle from camera (degrees)

# NEW: Orientation information
float64 crate_yaw         # Crate's orientation in image plane (radians, -π to π)
float64 approach_yaw      # Recommended approach angle (radians, -π to π)
float64 grasp_score       # Combined score for grasp planning (0.0 - 1.0)

# NEW: Geometric validation
int32 corner_count        # Number of detected corners (should be 4)
float64 aspect_ratio      # Width/Height ratio (should be ~3.0)

# Image space coordinates (for visualization/debugging)
float64 pixel_x           # Center X in image (pixels)
float64 pixel_y           # Center Y in image (pixels)
float64 bbox_width        # Bounding box width (pixels)
float64 bbox_height       # Bounding box height (pixels)