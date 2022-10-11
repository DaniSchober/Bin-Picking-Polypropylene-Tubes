# User Interface for the bin picking application
#
# =============================================================================
#                       IMPORTS
# =============================================================================

from random import randint
import helpers
import os
import numpy as np
import cv2
from datetime import datetime
import zivid
from datetime import timedelta
import robotiq_gripper
from threading import Thread
import time

# Read Transformation Matrix
M = helpers.read_transform("./robodk_hand_eye_calibration/datasets/handeye/2022-09-13_09-47-19/hand_eye_transform.yaml")


# =============================================================================
#                       FUNCTIONS
# =============================================================================

def main_menu():
    # MAIN_MENU Display the MAIN MENU.

    choice = 0

    """ 
    Setting up connections 
    """

    # Connect to UR5
    print("Connecting to robot...")
    rtde_c, rtde_r, rtde_io = helpers.connect_robot()
    rtde_c.moveL(helpers.home_pos, 0.2, 0.2)
    print("Connected!")

    # Activate the gripper and initialize force and speed
    print("Activating Gripper")
    ip = "192.168.2.1"
    gripper = robotiq_gripper.RobotiqGripper()
    gripper.connect(ip, 63352)
    #gripper = RobotiqGripper(rtde_c)
    gripper.activate()  # returns to previous position after activation
    #gripper.set_force(0)  # from 0 to 100 %
    #gripper.set_speed(100)  # from 0 to 100 %
    print("Gripper activated")
    

    # Activating and setting up camera
    print("Connecting to camera")
    app = zivid.Application()
    camera = app.connect_camera()

    print("Configuring 3D settings")
    settings = zivid.Settings()
    settings.experimental.engine = "phase"
    settings.acquisitions.append(zivid.Settings.Acquisition())
    settings.acquisitions[0].aperture = 2.03756615119344
    settings.acquisitions[0].exposure_time = timedelta(microseconds=20000)
    settings.processing.filters.outlier.removal.enabled = True
    settings.processing.filters.outlier.removal.threshold = 5.0

    print("Configuring 2D settings")
    # The Zivid SDK supports 2D captures with a single acquisition only
    settings_2d = zivid.Settings2D()
    settings_2d.acquisitions.append(zivid.Settings2D.Acquisition())
    settings_2d.acquisitions[0].exposure_time = timedelta(microseconds=20000)
    settings_2d.acquisitions[0].aperture = 3.31
    settings_2d.acquisitions[0].brightness = 1.80
    settings_2d.acquisitions[0].gain = 1.0
    settings_2d.processing.color.balance.red = 1.0
    settings_2d.processing.color.balance.green = 1.0
    settings_2d.processing.color.balance.blue = 1.0
    settings_2d.processing.color.gamma = 1.0
    print("Configuration done.")

    # define class for threading

    class CustomThread(Thread):
        # constructor
        def __init__(self):
            Thread.__init__(self)
            self.value1 = None
            self.value2 = None
        
        # function executed in a new thread
        def run(self):
            time.sleep(2.5)
            self.value1, self.value2 = helpers.find_goal_positions(app, settings, settings_2d, camera, M, canny_low)

            
    while True : 
        inp=input("\nInsert here: ")
        inp = inp.strip()
        choice = inp.upper()
        
        # CHOICE 1: "Run program"
        if choice == "RUN":
            print("Run bin picking program")
            canny_low = 50
            # Capture and save 3D and 2D frame
            goal_positions, angles = helpers.find_goal_positions(app, settings, settings_2d, camera, M, canny_low)
            helpers.move_above_goal(helpers.home_pos, helpers.middle_pos, rtde_c)
            i = 0
            grasps = []
            for goal_pos in goal_positions:
                if not (helpers.pick_object(goal_pos, rtde_c, rtde_r, gripper, angles[i])):
                    #helpers.bring_object_to_conveyor(helpers.middle_pos_high, helpers.home_pos, helpers.above_conveyor_pos, helpers.place_tube_pos, rtde_c, rtde_io, gripper)
                    #des rausnehmen fuer simultan
                    bring_object = True
                    grasps.append(i+1)
                    print(grasps)
                    break
                else:
                    rtde_c.moveL(helpers.middle_pos, speed = 2, acceleration = 2)
                i += 1
            if (i == len(goal_positions)):
                #perform randomize and take another picture
                helpers.randomize(rtde_c, rtde_r, gripper)
                canny_low = randint(40, 60)

            while(True):
                
                thread1 = CustomThread()
                thread2 = Thread(target=helpers.bring_object_to_conveyor, args = (helpers.middle_pos_high, helpers.home_pos, helpers.above_conveyor_pos, helpers.place_tube_pos, rtde_c, rtde_io, gripper))
                
                thread1.start()
                if bring_object == True:
                    thread2.start()

                thread1.join()
                if bring_object == True:
                    thread2.join()

                goal_positions = thread1.value1
                #print(goal_positions)
                angles = thread1.value2
                #print(angles)
                

                #helpers.move_above_goal(helpers.home_pos, helpers.middle_pos, rtde_c)
                i = 0
                for goal_pos in goal_positions:
                    if not (helpers.pick_object(goal_pos, rtde_c, rtde_r, gripper, angles[i])):
                        #helpers.bring_object_to_conveyor(helpers.middle_pos_high, helpers.home_pos, helpers.above_conveyor_pos, helpers.place_tube_pos, rtde_c, rtde_io, gripper)
                        #des rausnehmen fuer simultan
                        bring_object = True
                        grasps.append(i+1)
                        print(grasps)
                        break
                    else:
                        rtde_c.moveL(helpers.middle_pos, speed = 2, acceleration = 2)
                    i += 1
                if (i == len(goal_positions)):
                    #perform randomize and take another picture
                    helpers.randomize(rtde_c, rtde_r, gripper)
                    rtde_c.moveL(helpers.home_pos, speed = 2, acceleration = 2)
                    canny_low = randint(40, 60)
                    bring_object = False

        # CHOICE 2: "Warm up camera"     
        elif choice == "WARM UP":
            helpers.warm_up_camera(camera, mints = 5)

        # CHOICE 3: "Run Test Program on Robot"     
        elif choice == "TEST ROBOT":
            x = float(input("\nInsert x coordinate here: ")) #107.9
            y = float(input("\nInsert y coordinate here: ")) #-262.5
            z = float(input("\nInsert z coordinate here: ")) #1278.6
            P_input = np.array([x, y, z, 1])
            print(P_input)
            P_robot = np.matmul(M, P_input)
            print("P_robot ", P_robot)
            P_robot = np.array(P_robot)
            print("P_robot ", P_robot)
            z = P_robot[2]*0.001
            if z < -272:
                z = -272.0
            #P_robot = np.concatenate(P_robot)
            test_pos = [P_robot[0]*0.001,
                        P_robot[1]*0.001, z,
                        -2.2120065495659795,
                        -2.205496603905911,
                        -0.00]
            
            print(test_pos)
            helpers.move_above_goal(helpers.home_pos, helpers.middle_pos, test_pos, rtde_c)
            helpers.pick_object(test_pos, rtde_c, rtde_r, gripper)
            helpers.bring_object_to_conveyor(helpers.middle_pos, helpers.home_pos, helpers.above_conveyor_pos, helpers.place_tube_pos, rtde_c, rtde_io)
        

        # CHOICE 4: "Capture Frame."
        elif choice == "CAPTURE FRAME":
            frame_3D, image = helpers.capture_frame()
            date = datetime.now().strftime("%d_%m_%Y-%I:%M:%S_%p")
            image_file = "Organised/Captured/Image_" + date + ".png"
            zdf_file = "Organised/Captured/Frame_" + date + ".zdf"          
            print(f"Saving 2D color image to file: {image_file}")
            image.save(image_file)
            print(f"Saving frame to file: {zdf_file}")
            frame_3D.save(zdf_file)
            image = cv2.imread(image_file)

        # CHOICE 5: "Exit."
        elif choice== "EXIT":
            os.system('cls' if os.name=='nt' else 'clear')
            print("\n Exit program")
            
            break

        # If it is not a valid choice
        else:
            print("\nPlease try again")

# =============================================================================
#                       MAIN
# =============================================================================

if __name__ == '__main__':
    os.system('cls' if os.name=='nt' else 'clear')
    # Displays the WELCOME only the first time the program runs
    print("\n~~~~ Bin picking of lab tubes ~~~~")
    print("\n~~~~ OPTIONS: ~~~~")
    print("\n   • RUN")
    print("\n   • WARM UP")
    print("\n   • TEST ROBOT")
    print("\n   • CAPTURE FRAME")
    print("\n   • EXIT")
    # Display MAIN MENU
    main_menu()


