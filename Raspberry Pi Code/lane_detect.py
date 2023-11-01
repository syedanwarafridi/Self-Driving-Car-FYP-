import cv2
import numpy as np

# Initialize video capture
cap = cv2.VideoCapture('road.mp4')

# Define color range for yellow lines
lower_yellow = np.array([18, 94, 140], dtype=np.uint8)
upper_yellow = np.array([48, 255, 255], dtype=np.uint8)

# Define color range for white lines
lower_white = np.array([0, 0, 200], dtype=np.uint8)
upper_white = np.array([180, 25, 255], dtype=np.uint8)

while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blur, 50, 150)

    # Apply ROI mask
    mask = np.zeros_like(edges)
    roi_vertices = [(0, 360), (0, 200), (200, 100), (400, 100), (640, 200), (640, 360)]
    cv2.fillPoly(mask, np.array([roi_vertices]), 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Detect yellow lines
    yellow_mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower_yellow, upper_yellow)
    yellow_edges = cv2.bitwise_and(masked_edges, yellow_mask)
    yellow_lines = cv2.HoughLinesP(yellow_edges, 1, np.pi / 180, 30, maxLineGap=100)

    # Detect white lines
    white_mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower_white, upper_white)
    white_edges = cv2.bitwise_and(masked_edges, white_mask)
    white_lines = cv2.HoughLinesP(white_edges, 1, np.pi / 180, 30, maxLineGap=100)

    # Fit curves to yellow lines
    if yellow_lines is not None:
        for line in yellow_lines:
            x1, y1, x2, y2 = line[0]
            vx, vy, x, y = cv2.fitLine(np.array([(x1, y1), (x2, y2)], dtype=np.int32), cv2.DIST_L2, 0, 0.01, 0.01)
            slope = vy / vx
            intercept = y - slope * x
            y1 = int(360)
            x1 = int((y1 - intercept) / slope)
            y2 = int(200)
            x2 = int((y2 - intercept) / slope)
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Fit curves to white lines
    if white_lines is not None:
        for line in white_lines:
            x1, y1, x2, y2 = line[0]
            vx, vy, x, y = cv2.fitLine(np.array([(x1, y1), (x2, y2)], dtype=np.int32), cv2.DIST_L2, 0, 0.01, 0.01)
            slope = vy / vx  # calculate slope
            intercept = y - slope * x  # calculate intercept
            y1 = 360
            y2 = int(y1 * 0.6)
            x1_ext = int((y1 - intercept) / slope)
            x2_ext = int((y2 - intercept) / slope)
            cv2.line(frame, (x1_ext, y1), (x2_ext, y2), (0, 0, 255), 2)

    # Show the video frame
    cv2.imshow('Video', frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the resources
cap.release()
cv2.destroyAllWindows()

