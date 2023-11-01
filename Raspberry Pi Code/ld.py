import cv2
import numpy as np

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_lines(img, lines, color=[255, 0, 0], thickness=10):
    left_lines = []
    right_lines = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            if slope < 0:
                left_lines.append((slope, intercept))
            else:
                right_lines.append((slope, intercept))
    if len(left_lines) > 0 and len(right_lines) > 0:
        left_slope, left_intercept = np.average(left_lines, axis=0)
        right_slope, right_intercept = np.average(right_lines, axis=0)
        y1 = img.shape[0]
        y2 = int(y1 * 0.6)
        left_x1 = int((y1 - left_intercept) / left_slope)
        left_x2 = int((y2 - left_intercept) / left_slope)
        right_x1 = int((y1 - right_intercept) / right_slope)
        right_x2 = int((y2 - right_intercept) / right_slope)
        cv2.line(img, (left_x1, y1), (left_x2, y2), color, thickness)
        cv2.line(img, (right_x1, y1), (right_x2, y2), color, thickness)

def detect_lanes(frame):
    # convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # apply Gaussian blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # apply Canny edge detection
    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # define region of interest
    imshape = frame.shape
    vertices = np.array([[(0, imshape[0]), (imshape[1] / 2, imshape[0] / 2 + 50), (imshape[1] / 2, imshape[0] / 2 + 50), (imshape[1], imshape[0])]], dtype=np.int32)
    masked_edges = region_of_interest(edges, vertices)

    # apply Hough transform to detect lane lines
    rho = 1
    theta = np.pi / 250
    threshold = 20
    min_line_len = 50
    max_line_gap = 200
    lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]), min_line_len, max_line_gap)

    # draw lane lines on original frame
    line_image = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
    draw_lines(line_image, lines)
    result = cv2.addWeighted(frame, 0.8, line_image, 1, 0)

    return result

# open the video file
cap = cv2.VideoCapture("road.mp4")

# loop through each frame of the video
while True:
    # read the current frame
    ret, frame = cap.read()
    
    # check if there are no more frames
    if not ret:
        break
    
    # detect lanes in the current frame
    result = detect_lanes(frame)
    
    # display the lane detection result
    cv2.imshow("Lane Detection", result)
    
    # wait for a key event and check if 'q' was pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
