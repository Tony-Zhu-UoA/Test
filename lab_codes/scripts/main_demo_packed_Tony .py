import sys
import time
import rospy
import numpy as np
import threading
import re
import queue
import time

from function_library.basis_function.robot_move import UR5eRobot
from function_library.basis_function.open_gripper_ft300_sensor import FT300SensorHandler
from function_library.basis_function.gripper_control import Robotiq85Gripper
from function_library.basis_function.tool_detect_pick_gripper_camera import AzureKinectGripperCamera
from geometry_msgs.msg import WrenchStamped
from function_library.basis_function.conveyor_control import ConveyorBelt
from function_library.voice import VoiceAssistant

flag_open_gripper = False
flag_tool = None
transcript_queue = queue.Queue()
flag_return_tool = False
return_tool_lock = threading.Lock()
last_tool_id = None
name = "Tony"
state = "start"

def get_word_before_phrase(transcript, phrase="who are you"):
    """
    Extract the word before the specified phrase in the transcript.
    If the phrase is not found, return "Tony".
    """
    match = re.search(r'(\w+)\s+' + re.escape(phrase), transcript.lower())
    if match:
        return match.group(1)
    else:
        return "Tony"

def voice_assistant():
    """
    Main function to handle voice commands and tool requests.
    """
    global state, flag_tool, last_tool_id, voice, name
    # Get the recognized text from the queue
    recognized_text = voice.get_transcript(timeout=5)
    if recognized_text is not None:  
        # Process the recognized text
        if state == "start":
            if "who are you" in recognized_text.lower():
                name = get_word_before_phrase(recognized_text.lower())
                voice.text_to_speech("I am Tina, your personal assembly assistant. What are you assembling today?")
            elif "help me" in recognized_text.lower():
                state = "in_progress"
                voice.text_to_speech("I can assist you in getting the tools you need. Just let me know when you need a tool.")
        elif state == "in_progress":
            if "next tool" in recognized_text.lower() or "next to" in recognized_text.lower() or "next 2" in recognized_text.lower():
                if last_tool_id is None:
                    flag_tool = 2
                    last_tool_id = 2
                    voice.text_to_speech(f"Sure, {name}! Here is the insert tool. Let me know if you need help using it.")
                elif last_tool_id == 2:
                    flag_tool = 1
                    last_tool_id = 1
                    voice.text_to_speech(f"Sure, {name}! Here is the large wrench. Let me know if you need help using it.")
                elif last_tool_id == 1:
                    flag_tool = 4
                    last_tool_id = 4
                    voice.text_to_speech(f"Sure, {name}! Here is the small wrench. Let me know if you need help using it.")
                elif last_tool_id == 4:
                    voice.text_to_speech(f"{name}, there are no more tools left.")
            elif "help please" in recognized_text.lower() or "help me" in recognized_text.lower() or "show me" in recognized_text.lower() or "what now" in recognized_text.lower():
                if last_tool_id == 2:
                    voice.text_to_speech(f"{name}, first ensure the stem is correctly aligned then use the insert tool to push in the ball into the valve.")
                elif last_tool_id == 1:
                    voice.text_to_speech(f"{name}, use thee wrench to screw the end cap in.")
                elif last_tool_id == 4:
                    voice.text_to_speech(f"{name}, use the wrench to screw the handle nut in.")
            elif "reboot" in recognized_text.lower() or "reset" in recognized_text.lower() or "restart" in recognized_text.lower():
                last_tool_id = None
                state = "start"
                # synthesize_and_play_thread("I'm going to take a quick nap.")
                voice.text_to_speech("Oh! Hello again! Could you remind me of your name?")
            elif "for the day" in recognized_text.lower():
                state = "shutdown"
                voice.text_to_speech(f"Alright, no problem! See you tomorrow, {name}!")
        elif state == "shutdown":
            if "reboot" in recognized_text.lower() or "reset" in recognized_text.lower() or "restart" in recognized_text.lower() or "start" in recognized_text.lower():
                last_tool_id = None
                state = "start"
                voice.text_to_speech("Oh! Hello again! Could you remind me of your name?")   
    else:
        print("No recognized text received.")




if __name__ == "__main__":
    rospy.init_node("voice_pick")
    # Initialize the robot, gripper, camera, and conveyor
    robot = UR5eRobot()
    # Initialize the gripper
    gripper = Robotiq85Gripper()
    # Initialize the camera
    camera = AzureKinectGripperCamera()
    # Initialize the conveyor belt
    conveyor = ConveyorBelt()
    # Initialize the voice assistant
    credentials_path = "/home/zzq/psyched-thunder-454702-f7-a5bf7cfae33f.json" # Path to your Google Cloud API credentials
    voice = VoiceAssistant(credentials_path)
    # Initialize the FT300 sensor handler
    # ft300_handler = FT300SensorHandler()
    # sensor_control_thread = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, ft300_handler.ft300s_callback)
    #Create a thread to run speech_to_text_recognize
    recognition_thread = threading.Thread(target = voice.speech_to_text_recognize)
    recognition_thread.daemon = True 
    recognition_thread.start()
    robot.go_home()
    gripper.open()
    while True:
        pose_list = []
        pose_list.append([0.297, -0.132, 0.272, 2.226, -2.217, 0.0])
        pose_list.append([-0.132, -0.297, 0.272, 0.0, -3.141, 0.0])
        robot.execute_cartesian_trajectory(pose_list)

        # send converyor to home position
        # conveyor.go_home()

        over_list =[]
        over_list.append([-0.191, -0.668, 0.250, 0.0, -3.141, 0.0])
        robot.execute_cartesian_trajectory(over_list)

        # block further movement until conveyor finished moving

        camera.robot_pick_gripper_camera(0.10, "unpacked")
        
        # close gripper
        gripper.close_with_force(150.0)
        time.sleep(3)
        retract_list = []
        retract_list.append([-0.132, -0.78, 0.35, 0.0, -3.141, 0.0])
        retract_list.append([-0.132, -0.297, 0.272, 0.0, -3.141, 0.0])
        retract_list.append([0.297, -0.132, 0.272, 2.226, -2.217, 0.0])

        robot.execute_cartesian_trajectory(retract_list)

        handover_list = []
        handover_list.append([0.512, 0.241, 0.272, 2.226, -2.217, 0.0])
        handover_list.append([0.512, 0.241, 0.142, 2.226, -2.217, 0.0])
        robot.execute_cartesian_trajectory(handover_list)

        gripper.open()
        time.sleep(3)

        # conveyor.set_position(270)
        observe_list = []
        observe_list.append([0.586, -0.132, 0.663, 2.227, -2.217, 0.0])
        robot.execute_cartesian_trajectory(observe_list)
        

        camera.robot_pick_gripper_camera( 0.14, "packed")

        # close gripper
        gripper.close_with_force(150.0)
        time.sleep(3)

        place_list= []
        place_list.append([0.512, -0.034, 0.272, 2.226, -2.217, 0.0])
        place_list.append([0.148, 0.699, 0.350, 2.226, -2.217, 0.0])
        place_list.append([0.148, 0.699, 0.217, 2.227, -2.217, 0.0])
        robot.execute_cartesian_trajectory(place_list)

        # open gripper
        gripper.open()

        robot.go_home()
        # conveyor.go_home()
        # wait_movement(robot, conveyor, 0)
        time.sleep(2)