#! /usr/bin/env python
import rospy
import time
import actionlib
from actionlib.msg import TestFeedback, TestResult, TestAction
from actionlib_msgs.msg import GoalStatus
from pick_calibrate_go_return_pipeline import MoveBaristaManager, LoadSensorCalibrateManager

class MoveToTableActionServer(object):


    def __init__(self, time_movement_max = 120.0):

        self._time_movement_max = time_movement_max

        # creates the action server
        # create messages that are used to publish feedback/result

        self._feedback = TestFeedback()
        self._result = TestResult()
        self._as = actionlib.SimpleActionServer("/move_to_table_as", TestAction, self.goal_callback, False)
        self._as.start()

        # We init the classes for moving and checking load while moving
        # TODO: Solve issue for when we want to NOT monitor load through web while moving.
        #loadsensor_calibration_obj = LoadSensorCalibrateManager()
        loadsensor_calibration_obj = None
        # We wont give feedback here
        barista_feedback_obj = None
        self.move_barista_manage_obj = MoveBaristaManager(barista_feedback_obj=barista_feedback_obj,
                                                     loadsensor_calibration_obj=loadsensor_calibration_obj,
                                                    move_max_time = self._time_movement_max)


    def goal_callback(self, goal):
        # this callback is called when the action server is called.

        # helper variables
        r = rospy.Rate(1)
        move_final_result_ok = True
        msg = ""

        # TODO: We simulate the perfentage of the path done
        percentage_path_done = 10
        # build and publish the feedback message
        self._feedback.feedback = percentage_path_done
        self._as.publish_feedback(self._feedback)

        waypoints_list = self.move_barista_manage_obj.get_waypoints_list()
        table_name = "T"+str(goal.goal)
        if table_name in waypoints_list:
            move_final_result_ok, msg = self.move_barista_manage_obj.move_to_waypoint(table_name=table_name,
                                                          flag_check_load_ok=False)
            percentage_path_done = 100
        else:
            msg = "TABLE="+str(table_name)+", NOT FOUND in ==>>"+str(waypoints_list)
            rospy.logerr(msg)
            move_final_result_ok = False

        # build and publish the feedback message
        self._feedback.feedback = percentage_path_done
        self._as.publish_feedback(self._feedback)

        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If success, then we publish the final result
        # If not success, we do not publish anything in the result
        self._result.result = percentage_path_done
        rospy.loginfo('Percentage Path Done %i' % self._result.result)
        self._as.set_succeeded(self._result)



    def goal_callback_complete(self, goal):
        # this callback is called when the action server is called.

        # helper variables
        r = rospy.Rate(1)
        success = True

        # TODO: We simulate the perfentage of the path done
        percentage_path_done = 0
        for i in range(101):
            rospy.loginfo("Moving To Table=>>>"+str(goal.goal))
            # check that preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the calculation of the Fibonacci sequence
                break

            # TODO: Dummy Wait to simulate doing something
            time.sleep(0.1)

            # build and publish the feedback message
            self._feedback.feedback = i
            self._as.publish_feedback(self._feedback)

            percentage_path_done = i

        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If success, then we publish the final result
        # If not success, we do not publish anything in the result
        if success:
            self._result.result = percentage_path_done
            rospy.loginfo('Percentage Path Done %i' % self._result.result)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('move_to_table_action_server_node')
    MoveToTableActionServer(time_movement_max=180.0)
    rospy.spin()