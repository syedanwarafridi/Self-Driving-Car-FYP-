import cv2

# Initialize video capture
cap = cv2.VideoCapture('road.mp4')

# Get video resolution
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Video resolution: {width}x{height}")

# Release video capture
cap.release()

