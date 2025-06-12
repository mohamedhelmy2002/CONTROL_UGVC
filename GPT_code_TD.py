import cv2

# Load your large image
image = cv2.imread('im.png')
original_height, original_width = image.shape[:2]

# Display window size
window_width, window_height = 900, 500

# Initial zoom level
zoom = 1.0
zoom_step = 0.1
min_zoom = 0.5
max_zoom = 2.0

# Initial center (start from middle)
x_center = original_width // 2
y_center = original_height // 2

def update_display():
    global zoom, x_center, y_center

    # Calculate the size of the visible area
    display_w = int(window_width / zoom)
    display_h = int(window_height / zoom)

    # Clamp center values to image bounds
    x_center = max(display_w // 2, min(x_center, original_width - display_w // 2))
    y_center = max(display_h // 2, min(y_center, original_height - display_h // 2))

    # Calculate top-left corner for cropping
    x1 = x_center - display_w // 2
    y1 = y_center - display_h // 2
    x2 = x1 + display_w
    y2 = y1 + display_h

    # Crop and resize to window
    cropped = image[y1:y2, x1:x2]
    resized = cv2.resize(cropped, (window_width, window_height))

    cv2.imshow("Viewer", resized)

def on_trackbar_x(val):
    global x_center
    x_center = val
    update_display()

def on_trackbar_y(val):
    global y_center
    y_center = val
    update_display()

# Create window and sliders
cv2.namedWindow("Viewer")

# X and Y sliders (start from middle)
cv2.createTrackbar("X", "Viewer", x_center, original_width, on_trackbar_x)
cv2.createTrackbar("Y", "Viewer", y_center, original_height, on_trackbar_y)

update_display()

while True:
    key = cv2.waitKey(1) & 0xFF

    if key == ord('+') or key == ord('='):
        zoom = min(zoom + zoom_step, max_zoom)
        update_display()
    elif key == ord('-') or key == ord('_'):
        zoom = max(zoom - zoom_step, min_zoom)
        update_display()
    elif key == 27:  # ESC key
        break

cv2.destroyAllWindows()
