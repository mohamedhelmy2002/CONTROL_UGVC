import time
import numpy as np 
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# need to define 
# calibration 
# plane dimensions 
# in cm 
real_width_cm = 350
real_height_cm = 100
# in pixels
pixel_width_plane = 427
pixel_height_plane = 122
### scale 
scale_x = real_width_cm / pixel_width_plane  # cm per pixel (horizontally)
scale_y = real_height_cm / pixel_height_plane # cm per pixel (vertically)
# area 
area_scale = scale_x * scale_y  # in cm² per pixel²

# shifting
# functions 
import cv2

def draw_vertical_center_line(image, color, thickness=2):
    #image_with_line = image.copy()
    height, width = image.shape[:2]

    start_point = (width // 2, 0)       # Top center
    end_point = (width // 2, height)    # Bottom center

    cv2.line(image, start_point, end_point, color, thickness)

    return image

# control     
# general frames 

#_# :) calibration stage 
def sort_corners(pts): # use with clibration stage to sort the right corner for calibration stage 
    """
    Orders four points in the order:
    Top-Left, Top-Right, Bottom-Right, Bottom-Left

    :param pts: (4,2) NumPy array of points (unordered)
    :return: (4,2) NumPy array of ordered points
    """
    # Step 1: Sort points by Y-coordinate (top two first, bottom two last)
    pts = pts[np.argsort(pts[:, 1])]

    # Step 2: Separate top and bottom points
    top_points = pts[:2]
    bottom_points = pts[2:]

    # Step 3: Sort left-right for both top and bottom
    top_points = top_points[np.argsort(top_points[:, 0])]
    bottom_points = bottom_points[np.argsort(bottom_points[:, 0])]

    # Step 4: Arrange in correct order
    ordered_pts = np.array([top_points[0], top_points[1], bottom_points[1], bottom_points[0]])

    return ordered_pts

## vision 
def nothing(x):
    pass

#fram trackbars windows 
cv2.namedWindow("Calibration")
cv2.createTrackbar("darkness","Calibration",0,255,nothing)
cv2.createTrackbar("contour","Calibration",0,4,nothing)
cv2.createTrackbar("Calibration_Status_Bar","Calibration",0,1,nothing) # if one is still in calibration mode 
#Perspective  trackbars windows 
cv2.namedWindow("Perspective_calb")
cv2.createTrackbar("X_shift","Perspective_calb",0,800,nothing)
cv2.createTrackbar("Y_shift","Perspective_calb",0,800,nothing)
cv2.createTrackbar("center_shift","Perspective_calb",50,250,nothing)

# make connection with CoppeliaSim using ZMQ Remote API
client = RemoteAPIClient()
sim = client.getObject('sim')

# Initialize simulation if not running
if sim.getSimulationState() == sim.simulation_stopped:
    sim.startSimulation()

if client:
    print('Connected to remote API server')
    # initial parameter 
    # define robot 
    robot_handel = sim.getObject('/robot')
    # actuators 
    left_back_handle = sim.getObject('./left_back')
    right_back_handle = sim.getObject('./right_back')
    sj_l_handles = [sim.getObject(f'./sj_l_{i}') for i in range(1, 7)]
    sj_r_handles = [sim.getObject(f'./sj_r_{i}') for i in range(1, 7)]
    # initial actuator velocity 
    sim.setJointTargetVelocity(left_back_handle, 0)
    sim.setJointTargetVelocity(right_back_handle, 0)
    # camera
    cam = sim.getObject('./camera')
    resolusion, fram = sim.getVisionSensorImg(cam)
    
    # start controlling 
    startTime = time.time()
    while time.time()-startTime < 4000:
        # real time parameter
        # get object position and orientation
        positions = sim.getObjectPosition(robot_handel, -1)
        angles = sim.getObjectOrientation(robot_handel, -1)
        linear_velocity, angular = sim.getObjectVelocity(robot_handel)
            
        # control
        # general input
        for handle in sj_l_handles + sj_r_handles:
            sim.setJointForce(handle, 0)
     
        # camera
        img_buf, resX, resY = sim.getVisionSensorCharImage(cam)
        image_array = np.frombuffer(img_buf, dtype=np.uint8).reshape((resY, resX, 3))
        image_array = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
        image_array = cv2.flip(image_array, 0)

        #_#
        image = image_array  
        clear = image_array # take copy for fram without any effect 
        #cv2.imshow("clear",clear)
        cv2.waitKey(1)
        #_# vision only 
        gray_image = cv2.cvtColor(image_array, cv2.COLOR_RGB2GRAY)
        #cv2.imshow("gray_video", gray_image)
        
        #_# VISION 
        #__$ GENERAL

        #_# :) calibration 
        try: # to save the last values of bar and see if it availabe or not at all 
            state_calibration = cv2.getTrackbarPos("Calibration_Status_Bar", "Calibration")
        except: 
            state_calibration = 1
    
        if (state_calibration == 0): # you are in calibration level 
            Lower_value = cv2.getTrackbarPos("darkness", "Calibration")
            mask = cv2.inRange(gray_image, Lower_value, 255) # asphalt filter 
            cv2.imshow("mask", mask)
            # gray_image = cv2.bitwise_and(image_array, image_array, mask=mask) # not necessary 
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contour_num = cv2.getTrackbarPos("contour", "Calibration")
            cv2.drawContours(image_array, contours, contour_num, (0,255,0), 2)
            #for detect specific edges of the shape 
            # Approximate the contour to get corner points
            epsilon = 0.002 * cv2.arcLength(contours[contour_num], True)  # Adjust epsilon for accuracy
            approx = cv2.approxPolyDP(contours[contour_num], epsilon, True)
            # Draw the contour and corner points
            cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
            for point in approx:
                x, y = point[0]
                cv2.circle(image, (x, y), 5, (0, 0, 255), -1)  # Red dots for corners
            try:
                if len(approx) == 4:
                    coreners = approx.reshape(4,2) # to remove extra brackets
                    corener = sort_corners(coreners) # resorting the points to be in correct direction 
                    #print(corener)
                    #_# checking 
                    cv2.circle(image_array, corener[0], 7, (0,0,0), -1) #  black top left
                    cv2.circle(image_array, corener[1], 7, (255,255,255), -1)#  white top right
                    cv2.circle(image_array, corener[2], 7, (0,0,255), -1)# red bottom left
                    cv2.circle(image_array, corener[3], 7, (0,255,0), -1)# green bottom right
                    #_# Perspective transformation 
                    points = np.float32(corener)                    
                    # adding some points to make shift in x and Y view to see all of road easily 
                    x_shift = cv2.getTrackbarPos("X_shift", "Perspective_calb")
                    y_shift = cv2.getTrackbarPos("Y_shift", "Perspective_calb") 
                    center_shift = cv2.getTrackbarPos("center_shift", "Perspective_calb") # to make both views in center of the robot 
                    # new image size is the perspective points
                    plane_dimension = np.array([[0,0], [pixel_width_plane,0], [pixel_width_plane,pixel_height_plane], [0,pixel_height_plane]]) # for plane
                    plane_dimension[:, 0] += x_shift  # Shift X values
                    plane_dimension[:, 1] += y_shift  # Shift Y values
                    size = plane_dimension[[2]].flatten().tolist()
                    new_image = np.float32(plane_dimension)
                    matrix = cv2.getPerspectiveTransform(points, new_image)
                    # draw lines
                    draw_vertical_center_line(image, (0,0,255)) # camera view
                    result = cv2.warpPerspective(image, matrix, (size[0]+round(x_shift*center_shift/100), size[1]))
                    draw_vertical_center_line(result, (0,255,0))  # perspective view
                    cv2.putText(result, f"Detecting Area: width {round(scale_x*result.shape[1], ndigits=2)} cm x high {round(scale_y*result.shape[0], ndigits=2)} cm"
                                , (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8
                                , (0,255,0), 2)                   
                    cv2.imshow("Perspective_calb", result)
                    #cv2.imshow("pres_mask", pres_mask)
                    cv2.imshow("Calibration", image_array)
                    
            except:
                #print(approx)
                print("not four ")
            i, x = 1, 0 # memory for closing unnecessary windows 
            sim.setJointTargetVelocity(left_back_handle, 20)
            sim.setJointTargetVelocity(right_back_handle, 20)
            sim.setJointForce(left_back_handle, 120)
            sim.setJointForce(right_back_handle, 120)

        else: # calibration end >> start moving
            if(i == 1):
                i = 0
                # start moving or not
                cv2.namedWindow("moving")
                cv2.createTrackbar("start_1", "moving", 0, 1, nothing)
                print("end calibration")
                cv2.destroyWindow("Calibration")
                cv2.destroyWindow("Perspective_calb")
                #cv2.destroyWindow("pres_mask")
                cv2.destroyWindow("mask")
            if (x == 0):
                print("Stop")
                x = cv2.getTrackbarPos("start_1", "moving")
            else:
                print("moving")
                if (x == 1): # to run one time
                    cv2.destroyWindow("moving")
                    x = 2
            # start working
            mask = cv2.inRange(gray_image, Lower_value, 255)
            eye_view = cv2.warpPerspective(image, matrix, (size[0]+round(x_shift*center_shift/100), size[1]))
            eye_mask = cv2.inRange(cv2.cvtColor(eye_view, cv2.COLOR_RGB2GRAY), Lower_value, 255) # taking mask of Eye view 
            ## showing data in the main view 
            cv2.putText(eye_view, f"Detecting Area: width {round(scale_x*result.shape[1], ndigits=2)} cm x high {round(scale_y*result.shape[0], ndigits=2)} cm"
                                , (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            ### improving image and apply filters
            binary_img = eye_mask # take copy 
            contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            min_area = round(40*40/area_scale)  # adjust this threshold
            filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

            #cv2.imshow("smoothed", smoothed)
            cv2.imshow("filtered_contours", binary_img)

            cv2.imshow("eye_view", eye_view)
            cv2.imshow("mask", mask)
            #print(Lower_value)
            cv2.imshow("eye_mask", eye_mask)

else:
    print('Failed connecting to remote API server')
    
print('Program ended')
cv2.destroyAllWindows()