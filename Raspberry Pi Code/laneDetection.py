import cv2

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

