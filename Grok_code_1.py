import cv2
import numpy as np

# Global variables
map_size = (10000, 10000)  # Map size in pixels (1 km x 1 km at 10 pixels/meter)
map_scale = 10  # Pixels per meter
map_canvas = np.zeros((map_size[0], map_size[1]), dtype=np.uint8)  # Binary map
robot_path = []  # Store robot positions [(x, y), ...]
viewport_size = (640, 480)  # Display window size
center_pixel = (map_size[0] // 2, map_size[1] // 2)  # Map center for robot

# Slider states
pan_x = 0  # Panning offset in pixels
pan_y = 0
zoom_level = 1.0  # Zoom factor (1.0 = original size)

def init_sliders(window_name):
    """Initialize OpenCV trackbars for panning and zooming."""
    global pan_x, pan_y, zoom_level
    # Slider ranges: pan ±viewport_size/2, zoom 0.5 to 2.0
    cv2.namedWindow(window_name)
    cv2.createTrackbar('Pan X', window_name, viewport_size[1] // 2, viewport_size[1], lambda x: None)
    cv2.createTrackbar('Pan Y', window_name, viewport_size[0] // 2, viewport_size[0], lambda x: None)
    cv2.createTrackbar('Zoom', window_name, 50, 100, lambda x: None)  # 0 to 100 maps to 0.5 to 2.0

def get_slider_values(window_name):
    """Read slider values and update global pan/zoom."""
    global pan_x, pan_y, zoom_level
    pan_x = cv2.getTrackbarPos('Pan X', window_name) - viewport_size[1] // 2
    pan_y = cv2.getTrackbarPos('Pan Y', window_name) - viewport_size[0] // 2
    zoom_slider = cv2.getTrackbarPos('Zoom', window_name)
    zoom_level = 0.5 + (zoom_slider / 100.0) * 1.5  # Map 0-100 to 0.5-2.0

def transform_image(binary_image, robot_x, robot_y, robot_yaw, image_scale=0.01):
    """Transform binary image to map coordinates based on robot pose."""
    # Binary image: pixels to meters (image_scale = meters/pixel)
    h, w = binary_image.shape
    robot_pixel_x, robot_pixel_y = w // 2, h - 1  # Robot at center-bottom of image
    map_points = []

    # Get non-zero pixels (objects) in binary image
    points = np.where(binary_image > 0)
    for y, x in zip(points[0], points[1]):
        # Convert pixel coordinates to meters relative to robot
        rel_x = (x - robot_pixel_x) * image_scale
        rel_y = (y - robot_pixel_y) * image_scale
        # Rotate by robot yaw
        cos_yaw, sin_yaw = np.cos(robot_yaw), np.sin(robot_yaw)
        world_x = robot_x + (rel_x * cos_yaw - rel_y * sin_yaw)
        world_y = robot_y + (rel_x * sin_yaw + rel_y * cos_yaw)
        # Convert to map pixels
        map_x = int(world_x * map_scale) + center_pixel[1]
        map_y = int(-world_y * map_scale) + center_pixel[0]  # Negative for image coords
        if 0 <= map_x < map_size[1] and 0 <= map_y < map_size[0]:
            map_points.append((map_y, map_x))
    return map_points

def update_map(binary_image, robot_x, robot_y, robot_yaw):
    """Update map with binary image and robot position."""
    global map_canvas, robot_path
    # Transform binary image points to map
    map_points = transform_image(binary_image, robot_x, robot_y, robot_yaw)
    for y, x in map_points:
        map_canvas[y, x] = 255  # Mark objects as white
    # Update robot path
    map_x = int(robot_x * map_scale) + center_pixel[1]
    map_y = int(-robot_y * map_scale) + center_pixel[0]
    robot_path.append((map_y, map_x))
    # Draw robot path (e.g., as a line)
    if len(robot_path) > 1:
        for i in range(1, len(robot_path)):
            cv2.line(map_canvas, robot_path[i-1], robot_path[i], 128, 1)  # Gray path

def get_viewport():
    """Extract viewport from map based on pan and zoom."""
    # Calculate viewport size based on zoom
    view_w = int(viewport_size[1] / zoom_level)
    view_h = int(viewport_size[0] / zoom_level)
    # Center of viewport (adjusted by pan)
    center_x = center_pixel[1] + pan_x
    center_y = center_pixel[0] + pan_y
    # Extract region
    x1 = max(0, center_x - view_w // 2)
    y1 = max(0, center_y - view_h // 2)
    x2 = min(map_size[1], x1 + view_w)
    y2 = min(map_size[0], y1 + view_h)
    viewport = map_canvas[y1:y2, x1:x2]
    # Resize to display size
    viewport = cv2.resize(viewport, viewport_size[::-1], interpolation=cv2.INTER_NEAREST)
    return viewport

def save_map(filename='map.png'):
    """Save the full map to a file."""
    cv2.imwrite(filename, map_canvas)

# Example usage in while loop
def main():
    window_name = 'Map Viewer'
    init_sliders(window_name)
    
    # Simulated data (replace with your robot's data)
    robot_x, robot_y, robot_yaw = 0, 0, 0
    binary_image = np.zeros((480, 640), dtype=np.uint8)  # Example binary image
    binary_image[200:280, 300:340] = 255  # Example obstacle

    while True:
        # Update map with new data
        update_map(binary_image, robot_x, robot_y, robot_yaw)
        # Get slider values
        get_slider_values(window_name)
        # Display viewport
        viewport = get_viewport()
        cv2.imshow(window_name, viewport)
        
        # Simulate robot movement (replace with actual odometry)
        robot_x += 0.1  # Move 0.1 meter
        robot_y -=.1
        robot_yaw += 0.01  # Rotate 0.01 rad
        
        # Save map on key press 's'
        key = cv2.waitKey(30) & 0xFF
        if key == ord('s'):
            save_map('robot_map.png')
            print("Map saved as robot_map.png")
        elif key == 27:  # ESC to exit
            break
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()