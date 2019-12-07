
import pyrealsense2 as rs
import numpy as np
import time
import cv2

def getBoxParams():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    depth_pixel = np.array((0, 0))
    depth = 0
    # Start streaming
    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # camera centroid offsets
    x_offset = 0.133 # m
    y_offset = 0.094 # m

    angle = 0
    depth_point = np.zeros(3)

    iterations = 50

    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        for i in range(iterations):
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

            # convert image to grayscale image
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
            H, W = gray_image.shape

            # crop image for the object identification
            gray_image = gray_image[0:300, 150:W - 125]
            img = cv2.cvtColor(color_image[0:300, 150:W - 125], cv2.COLOR_BGR2GRAY)
             
            # convert the grayscale image to binary image
            ret,thresh = cv2.threshold(gray_image, 190, 255,cv2.THRESH_BINARY)
            
            # find the contours and draw them
            contours,h = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(gray_image,contours,0,(0,0,255),2)

            # if empty continue
            if not len(contours):
                continue
            # else get the rectangle characteristics
            else:
                # rectangle characteristics
                rotrect = cv2.minAreaRect(contours[0])
                center = rotrect[0]
                angle += rotrect[2]
                side_lengths = rotrect[1] # 0 is width 1 is height

                # get the angle from 0 to -180
                if side_lengths[0] < side_lengths[1]:
                    angle -= 90

                # depth characteristics for the deprojection. These can be used to determine the box location in the camera frame.
                depth_pixel = center
                depth_pixel_flipped = depth_pixel[::-1]
                depth = aligned_depth_frame.get_distance(int(center[1]), int(center[0]))
             
            # draw the centroid
            cv2.circle(gray_image, (int(center[0]), int(center[1])), 5, (0, 0, 0), -1)
            
            # deproject the centroid
            depth_point += rs.rs2_deproject_pixel_to_point(depth_intrin, list(depth_pixel_flipped), depth)
            # adjust centroid based on experimentally determined camera axis offset
            depth_point[0] += x_offset
            depth_point[1] += y_offset

            # display the image
            images = np.vstack((gray_image, img))
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', gray_image)
            cv2.waitKey(1)   

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.waitKey(1000)
        cv2.destroyAllWindows()
        return angle / iterations, depth_point / iterations

# getBoxParams()

