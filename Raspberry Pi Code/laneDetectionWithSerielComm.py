import cv2
import numpy as np
import serial

def region_of_interest(img, vertices):
    # Create a mask with the same shape as the input image
    mask = np.zeros_like(img)

    # Fill the region of interest with white color (255)
    cv2.fillPoly(mask, vertices, 255)

    # Apply the mask to the image
    masked_img = cv2.bitwise_and(img, mask)

    return masked_img

def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    # Create a blank image to draw the lines on
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    # Iterate over the output lines and draw them on the blank image
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)

    # Merge the lines image with the original image
    merged_img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    return merged_img

def detect_lanes(image):
    # convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # apply Gaussian blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # apply Canny edge detection
    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # define region of interest
    imshape = image.shape
    vertices = np.array([[(0, imshape[0]), (imshape[1] / 2, imshape[0] / 2 + 50), (imshape[1] / 2, imshape[0] / 2 + 50), (imshape[1], imshape[0])]], dtype=np.int32)
    masked_edges = region_of_interest(edges, vertices)

    # apply Hough transform to detect lane lines
    rho = 1
    theta = np.pi / 180
    threshold = 30
    min_line_len = 100
    max_line_gap = 160
    lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]), min_line_len, max_line_gap)

    # calculate slope of each detected lane line
    left_slopes = []
    right_slopes = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1)
        if slope < 0:
            left_slopes.append(slope)
        else:
            right_slopes.append(slope)

    # check if car is outside lane
    if lines is None:
        # send command to Arduino to steer back into lane
        ser.write(b'steer_left\n')
    else:
        # determine which side the car is going outside the road
        left_count = len(left_slopes)
        right_count = len(right_slopes)
        if left_count > right_count:
            # send command to Arduino to steer right
            ser.write(b'steer_right\n')
        elif right_count > left_count:
            # send command to Arduino to steer left
            ser.write(b'steer_left\n')
        else:
            # send command to Arduino to steer straight
            ser.write(b'steer_straight\n')

    # draw lane lines on original image
    line_image = draw_lines(image, lines)

    return line_image

ser = serial.Serial('/dev/ttyACM0', 9600)
ser.open()


# Set up video capture from the camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error opening the camera")
    exit()

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    if not ret:
        print("Error reading the frame")
        break

    # Process the frame to detect lanes
    processed_frame = detect_lanes(frame)

    # Display the processed frame
    cv2.imshow("Lane Detection", processed_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
ser.close()

