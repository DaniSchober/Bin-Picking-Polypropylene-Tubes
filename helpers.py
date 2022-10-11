#from turtle import back
import cv2
import numpy as np
import zivid
import datetime
from datetime import datetime
from datetime import timedelta
from time import sleep
import rtde_control
import rtde_receive
from rtde_control import Path, PathEntry
import rtde_io
from robotiq_gripper_control import RobotiqGripper
import copy
import math
from matplotlib import pyplot as plt
import random
from random import randint

"""
Define important positions
"""
home_pos = [-0.19161405312765759,
 0.13472399156690068,
 0.365305811889139,
 -2.216389383069919,
 -2.214478541184546,
 -0.011795250688296986]

middle_pos = [-0.5892404615419603,
 0.030096186269537212,
 0.0204709361794337,
 -2.2120065495659795,
 -2.205496603905911,
 -0.023032448184135128]

middle_pos_high = [-0.5892404615419603,
 0.030096186269537212,
 0.3604709361794337,
 -2.2120065495659795,
 -2.205496603905911,
 -0.023032448184135128]

above_conveyor_pos = [-0.10178399097655502,
 0.42530727780286376,
 0.36529650939521296,
 -2.2164100615996616,
 -2.2145170665255822,
 -0.011858118701172806]

place_tube_pos = [-0.09530795655436443,
 0.4193777515105637,
 0.2030236559091107,
 -2.2187837262553156,
 -2.209625346523522,
 0.0005215327962423823]




def find_rectangles(img, lowerThreshold = 70, upperThreshold = 200):
    imgGry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  #convert image to grayscale
    ret , thrash = cv2.threshold(imgGry, lowerThreshold , upperThreshold, cv2.CHAIN_APPROX_NONE) #apply threshold to get binary image
    contours , hierarchy = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #find contours in the image
    contours = sorted(contours, key = cv2.contourArea, reverse = True)[1:20] #get the 20 contours with the biggest area

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1] - 5
        if len(approx) < 11 :
            #cv2.drawContours(img, [approx], 0, (255, 100, 0), 5)
            length_approx = str(len(approx))
            rect = cv2.minAreaRect(contour)

            #find center of the rectangle
            center = rect[0]
            img = cv2.circle(img, (int(center[0]),int(center[1])), radius=0, color=(0, 0, 255), thickness=10)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img,[box],0,(0,0,255),2)
    cv2.imshow('shapes', img)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()
    cv2.waitKey(1)


def warm_up_camera(camera, mints = 10):

    warmup_time = timedelta(minutes = mints)
    capture_cycle = timedelta(seconds=5)
    max_capture_time = timedelta(milliseconds=1000)

    print("Getting camera settings")
    suggest_settings_parameters = zivid.capture_assistant.SuggestSettingsParameters(
        max_capture_time=max_capture_time,
        ambient_light_frequency=zivid.capture_assistant.SuggestSettingsParameters.AmbientLightFrequency.none,
    )
    settings = zivid.capture_assistant.suggest_settings(camera, suggest_settings_parameters)

    before_warmup = datetime.datetime.now()

    print(f"Starting warm up for {warmup_time} minutes")
    while (datetime.datetime.now() - before_warmup) < warmup_time:

        before_capture = datetime.datetime.now()
        camera.capture(settings)
        after_capture = datetime.datetime.now()

        duration = after_capture - before_capture

        if duration.seconds <= capture_cycle.seconds:
            sleep(capture_cycle.seconds - duration.seconds)
        else:
            print(
                "Your capture time is longer than your desired capture cycle. \
                Please increase the desired capture cycle."
            )

        remaining_time = warmup_time - (datetime.datetime.now() - before_warmup)
        remaining_time_minutes = remaining_time.seconds // 60
        remaining_time_seconds = remaining_time.seconds % 60
        print(f"Remaining time: {remaining_time_minutes} minutes, {remaining_time_seconds} seconds.")

    print("Warm up completed")


def capture_frame(app, settings, settings_2d, camera):

    #print("Capturing frame")
    frame_3D = camera.capture(settings)
    
    #print("Capturing 2D frame")
    frame_2D = camera.capture(settings_2d)
    image = frame_2D.image_rgba()

    return frame_3D, image

def get_xyz_from_pixels(frame, pixel_x, pixel_y):
    """  
    Example: 
     data_file = "C:/Users/DSOB/OneDrive - Novo Nordisk/Code/Testing Frames/Frame_2706_1.zdf"
     with zivid.Application():
       frame = zivid.Frame(data_file)
       point_cloud = frame.point_cloud()
       xyz = point_cloud.copy_data("xyz")

    """
    #with zivid.Application():
        #frame = zivid.Frame(data_file)
    point_cloud = frame.point_cloud()
    matrix_xyz = point_cloud.copy_data("xyz")
    xyz = matrix_xyz[(pixel_y, pixel_x)]
    while math.isnan(xyz[1]):
        #print("Output was nan")
        pixel_y += 1
        pixel_x += 1
        xyz = matrix_xyz[(pixel_y, pixel_x)]
        #print("New output:", xyz)
    return xyz


def connect_robot(ip = "192.168.2.1"):
    rtde_c = rtde_control.RTDEControlInterface(ip) #IP address found on robot
    rtde_r = rtde_receive.RTDEReceiveInterface(ip)
    rtde_io_set = rtde_io.RTDEIOInterface(ip)
    return rtde_c, rtde_r, rtde_io_set

def move_above_goal(home_pos, middle_pos, rtde_c, vel_J = 3, acc_J = 3):
    path = Path()
    blend = 0.1
    #path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [home_pos[0], home_pos[1], home_pos[2], home_pos[3], home_pos[4], home_pos[5], vel_J, acc_J, blend]))
    path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [middle_pos[0], middle_pos[1], middle_pos[2], middle_pos[3], middle_pos[4], middle_pos[5],  vel_J, acc_J, blend]))
    rtde_c.movePath(path, False)
    return 0

def pick_object(pick_pos, rtde_c, rtde_r, gripper, angle = 0, vel_L = 3, acc_L = 2):
    
    rtde_c.moveL([pick_pos[0], pick_pos[1], pick_pos[2]+0.1, pick_pos[3], pick_pos[4], pick_pos[5]], vel_L, acc_L) # move linear 10 cm above the goal position
    gripper.move(120, speed = 100, force = 0) # open gripper for gripping
    actual_q = rtde_r.getActualQ() # get TCP position
    # check if angle is > 180 degrees, if yes turn in the opposite direction
    if angle > math.pi:
        angle = - (2*math.pi - angle) 
    if angle > math.pi/2:
        angle =  (angle - math.pi)
    if angle < (-math.pi/2):
        angle = (angle + math.pi)
    #print("actual angle:", angle)
    actual_q[5] += angle # set angle around z axis to the determined angle in rad   
    rtde_c.moveJ(actual_q, speed = 3) 
    goal = rtde_r.getActualTCPPose() # get position at after rotation
    goal[2] -= 0.1 

    speed = [0, 0, -0.1, 0, 0, 0]
    rtde_c.moveUntilContact(speed)

    #rtde_c.moveL(goal, vel_L/2, acc_L) # move 10cm linear downwards for gripping
    gripper.move(255, speed = 50, force = 0) # gripping
    sleep(0.5) 
    gripper_position = gripper.get_current_position()
    goal[2] += 0.2 
    rtde_c.moveL(goal, speed = 2, acceleration = 1) # move 10cm linear upwards after gripping
    if (gripper_position > 180):
        #print("Gripper closed")
        gripper_closed = True
    else:
        gripper_closed = False
    return gripper_closed

def bring_object_to_conveyor(middle_pos_high, home_pos, above_conveyor_pos, place_tube_pos, rtde_c, rtde_io, gripper, vel_L = 0.7, acc_L = 0.5, vel_J = 3, acc_J = 2):
    blend = 0.1
    vel_J = 3
    acc_J = 3
    path1 = Path()
    path1.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [middle_pos_high[0], middle_pos_high[1], middle_pos_high[2], middle_pos_high[3], middle_pos_high[4], middle_pos_high[5],  vel_J, acc_J, blend]))
    #path1.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [home_pos[0], home_pos[1], home_pos[2], home_pos[3], home_pos[4], home_pos[5], vel_J, acc_J, 0.08]))
    path1.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [above_conveyor_pos[0], above_conveyor_pos[1], above_conveyor_pos[2], above_conveyor_pos[3], above_conveyor_pos[4], above_conveyor_pos[5], vel_J, acc_J, blend]))
    path1.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [place_tube_pos[0], place_tube_pos[1], place_tube_pos[2], place_tube_pos[3], place_tube_pos[4], place_tube_pos[5], vel_J, acc_J, 0.0]))
    rtde_c.movePath(path1, False)
    gripper.move(0, speed = 100, force = 0) # release tube at conveyor
    path2 = Path()
    path2.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [above_conveyor_pos[0], above_conveyor_pos[1], above_conveyor_pos[2], above_conveyor_pos[3], above_conveyor_pos[4], above_conveyor_pos[5], vel_J, acc_J, blend]))
    path2.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [middle_pos_high[0], middle_pos_high[1], middle_pos_high[2], middle_pos_high[3], middle_pos_high[4], middle_pos_high[5], vel_J, acc_J, 0.0]))
    rtde_c.movePath(path2, False)
    return 0

def read_transform(transform_file):
    """Read transformation matrix from a YAML file.
    Args:
        transform_file: Path to the YAML file.
    Returns:
        transform: Transformation matrix.
    """
    file_storage = cv2.FileStorage(str(transform_file), cv2.FILE_STORAGE_READ)
    transform = file_storage.getNode("PoseState").mat()
    file_storage.release()

    return transform


def find_gripping_points1(img):
    img_cut = img[130:800, 600:1400]

    image = cv2.cvtColor(img_cut, cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(image, (5,5), 2)
    image = cv2.Canny(image, randint(40,60), 100)
    image = cv2.dilate(image, None, iterations = 4)
    image = cv2.erode(image, None, iterations = 3)

    img_res = copy.copy(img_cut)
    limit_low_length = 90
    limit_high_length = 110
    limit_low_width = 25
    limit_high_width = 30
    iter_out = 0

    gripping_points = np.empty([0,5])
    i = 0

    while(iter_out < 15 and i < 5):
        contours , hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #find contours in the image
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[1:100] #get the 60 contours with the biggest area
        np.random.shuffle(contours)
        for contour in contours:
            #print(contour)
            approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
            #print(len(approx))

            if 6 < len(approx) < 30 :
                
                rect = cv2.minAreaRect(contour) #(center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(points)
                if ((limit_low_length <= rect[1][0] <= limit_high_length) and (limit_low_width <= rect[1][1] <= limit_high_width)) or ((limit_low_length <= rect[1][1] <= limit_high_length) and (limit_low_width <= rect[1][0] <= limit_high_width)) :
                    #find center of the rectangle
                    center = rect[0]
                    
                    img = cv2.circle(img_res, (int(center[0]),int(center[1])), radius=0, color=(0, 0, 255), thickness=10)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(img_res,[box],0,(0,0,255),2)
                    
                    #find rotation 
                    angle = rect[2]
                    if rect[1][1] > rect[1][0]:
                        angle = 90 + angle
                    angle = angle * math.pi / 180.0
                    p2_x = ((center[0]) + 5 * math.cos(angle))
                    p2_y = ((center[1]) + 5 * math.sin(angle))

                    cv2.line(img_res, (int(center[0]),int(center[1])), (int(p2_x), int(p2_y)), (0,0,255))

                    x1 = center[0] + 600
                    y1 = center[1] + 130
                    x2 = p2_x + 600
                    y2 = p2_y + 130

                    #continue_iter = False
                    if not np.array([x1, y1, x2, y2, angle]) in gripping_points:
                        gripping_points = np.vstack([gripping_points,np.array([x1, y1, x2, y2, angle])])
                        i += 1
        

        limit_low_length -= 3
        limit_high_length += 3
        limit_low_width -= 2
        limit_high_width += 2

        iter_out += 1

    date = datetime.now().strftime("%d_%m_%Y-%I:%M:%S_%p")
    image_file = "Organised/CapturedNew/Result_Alg1_" + date + ".png"
    #print(f"Saving output image: {image_file}")
    cv2.imwrite(image_file, img_res)

    return gripping_points

def find_gripping_points2(img):

    img_cut = img[130:800, 600:1400]

    dilation_shape = 2
    dilatation_size = 2
    imgGry = cv2.cvtColor(img_cut, cv2.COLOR_BGR2GRAY)
    element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                        (dilatation_size, dilatation_size))

    imgGry = cv2.dilate(imgGry, element)
    ret , thresh = cv2.threshold(imgGry, randint(120,140) , 255, cv2.CHAIN_APPROX_NONE)

    img_res = copy.copy(img_cut)
    limit_low_length = 90
    limit_high_length = 120
    limit_low_width = 25
    limit_high_width = 40
    iter_out = 0

    gripping_points = np.empty([0,5])
    i = 0

    while(iter_out < 15 and i < 5):
        contours , hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #find contours in the image
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[1:200] #get the 60 contours with the biggest area
        np.random.shuffle(contours)
        for contour in contours:
            #print(contour)
            approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
            #print(len(approx))

            if len(approx) < 25 :
                
                rect = cv2.minAreaRect(contour) #(center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(points)
                if ((limit_low_length <= rect[1][0] <= limit_high_length) and (limit_low_width <= rect[1][1] <= limit_high_width)) or ((limit_low_length <= rect[1][1] <= limit_high_length) and (limit_low_width <= rect[1][0] <= limit_high_width)) :
                    #find center of the rectangle
                    center = rect[0]
                    
                    img = cv2.circle(img_res, (int(center[0]),int(center[1])), radius=0, color=(0, 0, 255), thickness=10)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(img_res,[box],0,(0,0,255),2)
                    
                    #find rotation 
                    angle = rect[2]
                    if rect[1][1] > rect[1][0]:
                        angle = 90 + angle
                    angle = angle * math.pi / 180.0
                    p2_x = ((center[0]) + 5 * math.cos(angle))
                    p2_y = ((center[1]) + 5 * math.sin(angle))

                    cv2.line(img_res, (int(center[0]),int(center[1])), (int(p2_x), int(p2_y)), (0,0,255))

                    x1 = center[0] + 600
                    y1 = center[1] + 130
                    x2 = p2_x + 600
                    y2 = p2_y + 130

                    #continue_iter = False
                    if not np.array([x1, y1, x2, y2, angle]) in gripping_points:
                        gripping_points = np.vstack([gripping_points,np.array([x1, y1, x2, y2, angle])])
                        i += 1
        

        limit_low_length -= 3
        limit_high_length += 3
        limit_low_width -= 2
        limit_high_width += 2

        iter_out += 1

    date = datetime.now().strftime("%d_%m_%Y-%I:%M:%S_%p")
    image_file = "Organised/CapturedNew/Result_Alg2_" + date + ".png"
    #print(f"Saving output image: {image_file}")
    cv2.imwrite(image_file, img_res)

    return gripping_points

def randomize(rtde_c, rtde_r, gripper):
    gripper.move(0, speed = 100, force = 0)
    rtde_c.moveL(middle_pos)
    speed = [0, 0, -0.1, 0, 0, 0]
    rtde_c.moveUntilContact(speed)

    goal = rtde_r.getActualTCPPose() 
    goal[2] += 0.01
    rtde_c.moveL(goal)

    goal = rtde_r.getActualTCPPose() 
    goal[0] -= 0.1
    goal[1] += 0.1
    rtde_c.moveL(goal)

    speed = [0, 0, -0.1, 0, 0, 0]
    rtde_c.moveUntilContact(speed)
    goal = rtde_r.getActualTCPPose() 
    goal[2] += 0.01
    rtde_c.moveL(goal)

    goal = rtde_r.getActualTCPPose() 
    #goal[0] = 0.1
    goal[1] -= 0.1 + 0.1*random.randint(0, 1)
    rtde_c.moveL(goal)

    speed = [0, 0, -0.1, 0, 0, 0]
    rtde_c.moveUntilContact(speed)
    goal = rtde_r.getActualTCPPose() 
    goal[2] += 0.01
    rtde_c.moveL(goal)

    goal = rtde_r.getActualTCPPose() 
    goal[0] += 0.1 + 0.1*random.randint(0, 1)
    goal[1] += 0.1 + 0.1*random.randint(0, 1)
    rtde_c.moveL(goal)

    goal = rtde_r.getActualTCPPose()
    goal[2] += 0.01
    rtde_c.moveL(goal)

    rtde_c.moveL(home_pos)


def find_goal_positions(app, settings, settings_2d, camera, M, canny_low):
    frame_3D, image = capture_frame(app, settings, settings_2d, camera)
    date = datetime.now().strftime("%d_%m_%Y-%I:%M:%S_%p")
    image_file = "Organised/CapturedNew/Image_" + date + ".png"
    #print(f"Saving 2D color image to file: {image_file}")
    image.save(image_file)
    image = cv2.imread(image_file)

    # Analyse 2D image to find gripping point
    functions_analyse = [find_gripping_points1(image), find_gripping_points2(image)]
    gripping_points = random.choice(functions_analyse)
    #gripping_points = find_gripping_points1(image)
    #print("Gripping_points:", gripping_points)
    
    goal_positions = np.empty([0,6])
    angles = np.array([])
    for i in range(len(gripping_points)):
        x1 = gripping_points[i][0]
        y1 = gripping_points[i][1]
        x2 = gripping_points[i][2]
        y2 = gripping_points[i][3]
        angles = np.append(angles, gripping_points[i][4])

        # get 3D coordinates from point cloud based on found gripping point
        xyz1 = get_xyz_from_pixels(frame_3D, round(x1), round(y1))

        # save x, y, z of gripping point in array and convert them from the camera to the robot coordinate system
        P_robot_1 = np.array(np.matmul(M, [xyz1[0], xyz1[1], xyz1[2], 1]))


        # define goal position with fixed rotation
        goal_pos = np.array([P_robot_1[0]*0.001,
                    P_robot_1[1]*0.001, P_robot_1[2]*0.001,
                    -2.2120065495659795,
                    -2.205496603905911,
                    -0.00])
        
        goal_positions = np.vstack([goal_positions,goal_pos])
    return goal_positions, angles
