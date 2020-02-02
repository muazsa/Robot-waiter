#!/usr/bin/env python

"""
This is a test of all the pipeline that we want to implement
* Detect that object has been places.
* Trigger the calibrate function
* Go to designated table, based on the pickle coordenates
* When arrived to the location, removed the object
* Detect that object has been removed, calibrate and return to loading station
* Begin the loop again.
"""
import os
import sys
import time
import rospy
import numpy
import random
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import String, UInt8
from pickle_commons import read_pickle_file
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from save_waypoint import SaveWaypointClass
from actionlib_msgs.msg import GoalStatus
from droidspeak.srv import DroidEmotionRequest
from actionlib.msg import TestFeedback, TestResult, TestAction

class SpawnDeleteManager(object):

    def __init__(self):

        rospy.logwarn("Waiting for /spawndelete_models_server...")
        rospy.wait_for_service('/spawndelete_models_server')
        rospy.logwarn("Waiting for /spawndelete_models_server...READY")
        # To acll the server that manages spawn and delete
        self._spawndelete_client = rospy.ServiceProxy('/spawndelete_models_server', SpawnModel)

        # To know the exact position of the barista model to spawn the object just on top
        # This is needed because navigation is not exact, the robot wont stop everytime in the same place
        self._model_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Object that we want to spawn and delete for this test
        self.objects_list = ["standard_apple",
                             "standard_cube"]

        self._z_height_spawn = 0.7
        self._x_delta_spawn = 0.1

        pose1 = Pose()
        pose1.position.x = 0.0
        pose1.position.y = 0.0
        pose1.position.z = self._z_height_spawn

        pose2 = Pose()

        pose2.position.x = pose1.position.x + self._x_delta_spawn
        pose1.position.y = pose1.position.y
        pose2.position.z = pose1.position.z

        self.object_pose_dict = {self.objects_list[0]: pose1,
                                 self.objects_list[1]: pose2}

        self.object_package_dict = {self.objects_list[0]: "spawn_robot_tools_pkg",
                                 self.objects_list[1]: "spawn_robot_tools_pkg"}

        self.object_type_dict = {self.objects_list[0]: "models",
                                 self.objects_list[1]: "models"}


    def get_current_robot_pose(self, robot_name="barista", relative_entity_name="world"):
        """
        It returns the pose of the given robot_name model in the simulated scene.
        By default we use barista and map.
        :param robot_name:
        :param relative_entity_name:
        :return:
        """

        request = GetModelStateRequest()
        request.model_name = robot_name
        request.relative_entity_name = relative_entity_name
        response = self._model_client(request)

        return response.success, response.status_message, response.pose


    def update_object_pose_dict(self):
        """
        It updates the objects poses with the current robots position
        :return:
        """

        result_ok, msg, robot_pose = self.get_current_robot_pose()
        if result_ok:
            i = 0
            for test_object in self.objects_list:
                object_pose = Pose()
                object_pose.position.x = robot_pose.position.x + (i*self._x_delta_spawn)
                object_pose.position.y = robot_pose.position.y
                object_pose.position.z = self._z_height_spawn
                self.object_pose_dict[test_object] = object_pose
                i += 1


    def spawn_test_objects(self):

        request = SpawnModelRequest()
        response = SpawnModelResponse()
        response.success = False
        response.status_message = "No Object in list"
        request.robot_namespace = "SPAWN"

        rospy.logdebug("Spawning LIST object Manager ==>" + str(self.objects_list))
        for model_name in self.objects_list:
            rospy.logdebug("Spawning object Manager ==>"+str(model_name))
            request.model_name = model_name
            self.update_object_pose_dict()
            request.initial_pose = self.object_pose_dict.get(model_name, "none")
            request.reference_frame = self.object_type_dict.get(model_name, "none")
            request.model_xml = self.object_package_dict.get(model_name, "none")
            response = self._spawndelete_client(request)

        return response.success, response.status_message


    def delete_test_objects(self):

        request = SpawnModelRequest()
        request.robot_namespace = "DELETE"
        delete_ok = True
        delete_msg = ""
        rospy.logdebug("DELETING LIST object Manager ==>" + str(self.objects_list))
        for model_name in self.objects_list:
            rospy.logdebug("Deleting object MODEL==>" + str(model_name))
            request.model_name = model_name
            request.initial_pose = self.object_pose_dict.get(model_name, "none")
            response = self._spawndelete_client(request)

            delete_ok = delete_ok and response.success
            delete_msg += "Model="+str(model_name)+", MSG="+str(response.status_message)

        return delete_ok, delete_msg


class LoadSensorCalibrateManager(object):

    def __init__(self):
        service_name = '/load_sensor_calibrate_server'
        rospy.logwarn("Waiting for "+service_name+"...")
        rospy.wait_for_service(service_name)
        rospy.logwarn("Waiting for "+service_name+"...READY")
        self._spawndelete_client = rospy.ServiceProxy(service_name, SetBool)

    def _update_newest_load_sensor_info(self):
        """

        :return: loadsensor_info_id
        Extra Object was Added,0
        Object was removed,1
        Object loaded,2
        Load Sensor needs a calibration through ARDUINO calibration program,3
        Loading tray empty,4
        """
        rospy.logdebug("Getting Newest Load Sensor Info")
        self.load_sensor_info = None
        self.loadsensor_info_topic_name = "/load_sensor_info"

        while self.load_sensor_info is None and not rospy.is_shutdown():
            try:
                self.load_sensor_info = rospy.wait_for_message(self.loadsensor_info_topic_name,
                                                        String,
                                                        timeout=1.0)
            except:
                rospy.logerr("Current "+self.loadsensor_info_topic_name+" not ready yet, retrying...")

        rospy.logdebug("Getting Newest Load Sensor Info..DONE")
        loadsensor_info_data = self.load_sensor_info.data
        loadsensor_info_list = loadsensor_info_data.split(",")

        rospy.logdebug("loadsensor_info_list==>" + str(loadsensor_info_list))

        loadsensor_info_msg = loadsensor_info_list[0]
        loadsensor_info_id = int(loadsensor_info_list[1])
        return loadsensor_info_id, loadsensor_info_msg

    def calibrate(self):
        """
        Calibrate for object on top
        :return:
        """
        request = SetBoolRequest()
        request.data = True
        response = self._spawndelete_client(request)
        return response.success, response.message

    def reset(self):
        """
        Reset to Original no objects value
        :return:
        """
        request = SetBoolRequest()
        request.data = False
        response = self._spawndelete_client(request)
        return response.success, response.message

    def check_load_ok(self):
        """
        Special fucntion to check that the object loaded is ok
        This means that from these messages, only the index 2 is
        accepted.
        "Extra Object was Added,0"
        "Object was removed,1"
        "Object loaded,2"
        :return:
        """
        id, msg = self._update_newest_load_sensor_info()
        return id == 2, msg



class MoveBaristaManager(object):

    def __init__(self, barista_feedback_obj,loadsensor_calibration_obj, move_max_time=60.0):

        """
        :param barista_feedback_obj:
        :param loadsensor_calibration_obj:
        """

        self._move_max_time = move_max_time
        # We load classes for talking and checking loadsensor status
        self._barista_feedback_obj = barista_feedback_obj
        self._loadsensor_calibration_obj = loadsensor_calibration_obj

        # We do this to allow to have the most updated TableWaypoints
        self._table_waypoints_object = SaveWaypointClass(init_check_for_waypoints=False)
        rospy.logdebug(str(self._table_waypoints_object.get_waypoint_dict()))

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        self.init_move_to_table_action_server()

        rospy.loginfo("Connected to move base server")

    def get_waypoints_list(self):
        waypoints_dict = self._table_waypoints_object.get_waypoint_dict()
        return list(waypoints_dict.keys())


    def init_move_to_table_action_server(self):
        """
        Initialises the action server and all its variables
        :return:
        """
        self.reset_as_vars()
        self._as_move_ok = True
        self._as_move_msg = ""

        self._feedback = TestFeedback()
        self._result = TestResult()
        self._as = actionlib.SimpleActionServer("/move_to_table_as", TestAction, self.move_to_table_goal_callback, False)
        self._as.start()

    def reset_as_vars(self):
        self._as_flag_check_load_ok = True
        self._as_flag_talk = False
        self._NOTABLE = "NO-TABLE"
        self.table_to_go = self._NOTABLE


    def move_to_table_goal_callback(self, move_to_table_goal):

        """
        This method will wait for commands from the action server
        /move_to_table_as
        :return:
        """
        flag_check_load_ok = self._as_flag_check_load_ok
        flag_talk = self._as_flag_talk

        waypoints_list = self.get_waypoints_list()
        table_name = "T" + str(move_to_table_goal.goal)

        if table_name in waypoints_list:
            self.table_to_go = table_name
            rospy.loginfo("Recieved Table Command to=" + str(self.table_to_go))

            # We retrieve waypoint of the table_name given
            table_waypoint_dict = self._table_waypoints_object.get_waypoint_dict()
            rospy.loginfo("Found " + table_name + "table Waypoint")
            table_waypoint = table_waypoint_dict.get(table_name, "none")
            rospy.logdebug("Table " + str(table_name) + ",P=" + str(table_waypoint))

            # Intialize the waypoint goal
            goal = MoveBaseGoal()

            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = table_waypoint.pose

            percentage_path_done = 0
            # build and publish the feedback message
            self._feedback.feedback = percentage_path_done
            self._as.publish_feedback(self._feedback)

            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)

            # Allow 1 minute to get there
            # finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
            state_result = self.move_base.get_state()
            move_rate = rospy.Rate(20)
            rospy.loginfo("state_result: " + str(state_result))
            init_time = rospy.get_time()
            time_moving = 0


            while state_result <= GoalStatus.ACTIVE and time_moving <= self._move_max_time:
                rospy.logdebug("Moving...Checking Load and Talking....")

                if self._as.is_preempt_requested() or rospy.is_shutdown():
                    rospy.loginfo('The goal has been cancelled/preempted')
                    # the following line, sets the client in preempted state (goal cancelled)
                    self._as.set_preempted()
                    break

                if flag_check_load_ok:
                    load_ok, load_msg = self._loadsensor_calibration_obj.check_load_ok()
                    if not load_ok:
                        msg = "CHECK LOAD WHILE MOVING ERROR..." + load_msg
                        rospy.logerr(msg)
                        self.move_base.cancel_goal()
                        self._as.set_preempted()
                        break
                if flag_talk:
                    self._barista_feedback_obj.say("a")

                percentage_path_done = int((time_moving / self._move_max_time) * 100)
                self._feedback.feedback = percentage_path_done
                self._as.publish_feedback(self._feedback)

                move_rate.sleep()
                state_result = self.move_base.get_state()
                rospy.logdebug("state_result: " + str(state_result))
                now_time = rospy.get_time()
                time_moving = now_time - init_time

            # If we don't get there in time, abort the goal
            finished_within_time = time_moving < self._move_max_time


            if not finished_within_time:
                self.move_base.cancel_goal()
                msg = "Timed out achieving goal"
                rospy.loginfo(msg)
                self._as.set_preempted()
                self._as_move_ok = False
                self._as_move_msg = msg
            else:
                # We made it! But Ok, or something happened?
                if state_result == GoalStatus.SUCCEEDED:
                    self._result.result = 100
                    msg = 'Goal succeeded!, Percentage Path Done' + str(self._result.result)
                    rospy.loginfo(msg)
                    self._as.set_succeeded(self._result)
                    self._as_move_ok = True
                    self._as_move_msg = msg
                else:
                    msg = "Move DIDN'T succeed..."
                    rospy.logerr(msg)
                    self.move_base.cancel_goal()
                    self._as.set_preempted()
                    self._as_move_ok = False
                    self._as_move_msg = msg
        else:
            msg = "TABLE=" + str(table_name) + ", NOT FOUND in ==>>" + str(waypoints_list)
            rospy.logerr(msg)
            self.table_to_go = self._NOTABLE
            self._as.set_preempted()
            self._as_move_ok = False
            self._as_move_msg = msg

        self.reset_as_vars()


    def wait_for_waypoint_from_as(self, flag_check_load_ok=True, flag_talk=False):
        """
        This methos will wait for commands from the action server
        /move_to_table_as
        :return:
        """
        self._as_flag_check_load_ok = flag_check_load_ok
        self._as_flag_talk = flag_talk

        self._as_move_ok = True
        self._as_move_msg = ""

        rospy.logwarn("Start Wait for Waypoint Action Server command...")

        wait_rate = rospy.Rate(5)
        rospy.logwarn("Table="+str(self.table_to_go))
        while not rospy.is_shutdown() and self.table_to_go == self._NOTABLE :
            rospy.loginfo("Waiting for Table Order...")
            rospy.logwarn("Table=" + str(self.table_to_go))
            wait_rate.sleep()

        # Now we wait until it returns to its NO-Table state
        rospy.logwarn("Table=" + str(self.table_to_go))
        while not rospy.is_shutdown() and self.table_to_go != self._NOTABLE :
            rospy.loginfo("Waiting for robot to get to table...")
            rospy.logwarn("Table=" + str(self.table_to_go))
            wait_rate.sleep()

        rospy.logwarn("Wait for Waypoint Action Server command Finished...")

        return self._as_move_ok, self._as_move_msg


    def move_to_random_waypoint(self, flag_check_load_ok=True, flag_talk=False):
        """
        Moves to random waypoint form the list
        :return:
        """
        waypoints_list = self.get_waypoints_list()
        random_index = random.randint(0,len(waypoints_list)-1)
        random_name = waypoints_list[random_index]
        rospy.logwarn("Moving to Random Waypoint=====>>>>>>"+str(random_name))
        move_final_result, msg = self.move_to_waypoint(table_name=random_name,
                              flag_check_load_ok=flag_check_load_ok,
                              flag_talk=flag_talk)

        return move_final_result, msg


    def move_to_waypoint(self, table_name, flag_check_load_ok=True, flag_talk=False, flag_as=False):

        # We retrieve waypoint of the table_name given
        table_waypoint_dict = self._table_waypoints_object.get_waypoint_dict()
        if table_name in table_waypoint_dict.keys():
            rospy.loginfo("Found "+table_name+"table Waypoint")
            table_waypoint = table_waypoint_dict.get(table_name, "none")
            rospy.logdebug("Table "+str(table_name)+",P="+str(table_waypoint))
        else:
            error_msgs = "Table "+str(table_name)+" is NOT in Database =>"+str(self._table_waypoints_object)
            rospy.logerr(error_msgs)
            return False, error_msgs

        # Intialize the waypoint goal
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = table_waypoint.pose
        move_final_result, msg = self.move(goal, flag_check_load_ok, flag_talk)

        return move_final_result, msg



    def move(self, goal, flag_check_load_ok, flag_talk):

        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)

        # Allow 1 minute to get there
        #finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
        state_result = self.move_base.get_state()
        move_rate = rospy.Rate(20)
        rospy.loginfo("state_result: " + str(state_result))
        init_time = rospy.get_time()
        time_moving = 0
        while state_result <= GoalStatus.ACTIVE and time_moving <= self._move_max_time:
            rospy.logdebug("Moving...Checking Load and Talking....")
            if flag_check_load_ok:
                load_ok, load_msg = self._loadsensor_calibration_obj.check_load_ok()
                if not load_ok:
                    msg = "CHECK LOAD WHILE MOVING ERROR..." + load_msg
                    rospy.logerr(msg)
                    self.move_base.cancel_goal()
                    break
            if flag_talk:
                if self._barista_feedback_obj is not None:
                    self._barista_feedback_obj.say("a")

            move_rate.sleep()
            state_result = self.move_base.get_state()
            rospy.logdebug("state_result: " + str(state_result))
            now_time = rospy.get_time()
            time_moving = now_time - init_time

        # If we don't get there in time, abort the goal
        finished_within_time = time_moving < self._move_max_time

        move_final_result = True
        if not finished_within_time:
            self.move_base.cancel_goal()
            move_final_result = False
            msg = "Timed out achieving goal"
            rospy.loginfo(msg)
        else:
            # We made it! But Ok, or something happened?
            if state_result == GoalStatus.SUCCEEDED:
                msg = "Goal succeeded!"
                rospy.loginfo(msg)
            else:
                msg = "Move DIDN'T succeed..."
                rospy.logerr(msg)
                self.move_base.cancel_goal()
                move_final_result = False

        return move_final_result, msg

class BaristaFeedback(object):

    def __init__(self):
        self._droid_talker_msg_pub = rospy.Publisher('droid_talker_msg', String, queue_size=1)
        self._droid_talker_emotion_pub = rospy.Publisher('droid_talker_emotion', UInt8, queue_size=1)

    def change_emotion(self, emotion):
        """
        NORMAL, this will triguer normal speed speaking.
        SORRY, this will trigger slower speaking.
        ANGRY, this will trigger faster speaking.
        :param emotion:
        :return:
        """
        emotion = UInt8()
        if emotion == "NORMAL":
            emotion.data = DroidEmotionRequest.NORMAL
        elif emotion == "SORRY":
            emotion.data = DroidEmotionRequest.SORRY
        elif emotion == "ANGRY":
            emotion.data = DroidEmotionRequest.ANGRY
        else:
            emotion.data = DroidEmotionRequest.NORMAL

        self._droid_talker_emotion_pub.publish(emotion)

    def say(self, text):
        """
        It will make Barista say something in droid language
        :param text:
        :return:
        """
        say_message = String()
        say_message.data = text
        self._droid_talker_msg_pub.publish(say_message)


def countdown(t, step=1, msg='sleeping'):  # in seconds
    pad_str = ' ' * len('%d' % step)
    for i in numpy.arange(t,0,-step):
        rospy.loginfo('%s for the next %f seconds %s\r' % (msg, i, pad_str))
        time.sleep(step)
    rospy.loginfo('Done %s for %f seconds!  %s' % (msg, t, pad_str))

def main():
    if len(sys.argv) < 2:
        print("usage: pick_calibrate_go_return_pipeline.py speak_active")
    else:
        speak_active = (sys.argv[1] == "true")

        rospy.init_node("pick_go_return_demo_node", log_level=rospy.INFO)
        if speak_active:
            barista_feedback_obj = BaristaFeedback()
        else:
            barista_feedback_obj = None

        loadsensor_calibration_obj = LoadSensorCalibrateManager()
        move_barista_manage_obj = MoveBaristaManager(barista_feedback_obj,loadsensor_calibration_obj)
        refresh_rate = 10.0
        rate = rospy.Rate(refresh_rate)

        if speak_active:
            barista_feedback_obj.say("home")

        move_barista_manage_obj.move_to_waypoint(table_name="X2",flag_check_load_ok=False)

        while not rospy.is_shutdown():

            # Spawn of Objects
            spawn_delete_models_manager = SpawnDeleteManager()
            ok, message = spawn_delete_models_manager.spawn_test_objects()
            if not ok:
                rospy.logerr("PickGoReturnDemo:::Error in SpawnObject=" + str(message))
                break
            else:
                rospy.logwarn("PickGoReturnDemo:::NO Error in SpawnObject=" + str(message))



            # We check that the Object was placed on top
            id = -1
            msg = "..."
            # We look now for "'Load Sensor needs a calibration through ARDUINO/SimDriver calibration program', '3'"
            while id != 3 and not rospy.is_shutdown():
                id, msg = loadsensor_calibration_obj._update_newest_load_sensor_info()
                rospy.loginfo("Msg=" + str(msg) + ",Id=" + str(id))
                rate.sleep()

            # Just in case we wait for the objects to settle
            time.sleep(3)

            # Calibrate Load Sensor
            ok, message = loadsensor_calibration_obj.calibrate()
            if not ok:
                rospy.logerr("Error in Calibration="+str(message))
                assert True, "Error In Calibration"
            rospy.logerr("Result Calibration=" + str(ok)+", msg="+str(message))
            if speak_active:
                barista_feedback_obj.say("calibrated")


            #We check that the Object was loaded and calibrated
            id = -1
            msg = "..."
            # We look now for "Object loaded,2"
            while id != 2 and not rospy.is_shutdown():
                id, msg = loadsensor_calibration_obj._update_newest_load_sensor_info()
                rospy.loginfo("Msg="+str(msg)+",Id="+str(id))
                rate.sleep()
            rospy.loginfo("Object loaded Detected...")

            # Move to Given Table ( Ex: T1)
            if speak_active:
                barista_feedback_obj.say("work")
            #move_barista_manage_obj.move_to_waypoint(table_name="T1")
            # We test different tables to go to
            move_ok, msg = move_barista_manage_obj.move_to_random_waypoint(flag_check_load_ok=True)

            if move_ok:

                # When arrived, ask to pick object. Wait and repeat until object picked
                # We look now for "Object was removed,1"
                id = -1
                # We have to put random times to delete
                max_time_until_delete = 5.0
                seconds_until_delete = (random.random())*max_time_until_delete
                talking_period = 2.0

                init_seconds = rospy.get_time()
                init_seconds_speak  = rospy.get_time()

                if speak_active:
                    barista_feedback_obj.say("getorder")
                while id != 1 and not rospy.is_shutdown():

                    id, msg = loadsensor_calibration_obj._update_newest_load_sensor_info()
                    rospy.loginfo("Msg="+str(msg)+",Id="+str(id))
                    rate.sleep()

                    # We check if it has passed this time
                    now_seconds = rospy.get_time()
                    delta = now_seconds - init_seconds
                    delta_speak = now_seconds - init_seconds_speak
                    rospy.logwarn("Random Time until Object remove==>" + str(seconds_until_delete))
                    rospy.loginfo("Delta=" + str(delta)+", DeltaSpeak="+str(delta_speak))
                    if delta >= seconds_until_delete:
                        spawn_delete_models_manager.delete_test_objects()
                    if delta_speak > talking_period:
                        if speak_active:
                            barista_feedback_obj.say("getorder")
                        init_seconds_speak = rospy.get_time()

                if speak_active:
                    barista_feedback_obj.say("thankyou")

            else:
                # The move base had some issue so lets remove the objects
                spawn_delete_models_manager.delete_test_objects()

            # Reset LoadSensor and Move to Table X1 ( Loading Table )
            ok, message = loadsensor_calibration_obj.reset()
            if not ok:
                rospy.logerr("Error in Calibration="+str(message))
                break

            if speak_active:
                barista_feedback_obj.say("home")
            move_barista_manage_obj.move_to_waypoint(table_name="X2",flag_check_load_ok=False)

            if speak_active:
                barista_feedback_obj.say("finished")

            rospy.logwarn("######################### END OF LOOP....")
            time_before_restart = 2.0
            countdown(time_before_restart, step=1, msg='RESTARTING....')
            rospy.logwarn("######################### RESTARTING....")




if __name__ == "__main__":
    main()