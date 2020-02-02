#!/usr/bin/env python

"""
This is the final script that emulates the closes to real script but in simulation
* We press key to spawn objects.
* Waits for calibration
* Waits for table order
* Goes to place and waits for delete objects
* Autocalibrates and returns to base
* Begin the loop again.
"""
import sys
import time
import rospy
from pick_calibrate_go_return_pipeline import BaristaFeedback, LoadSensorCalibrateManager
from pick_calibrate_go_return_pipeline import MoveBaristaManager, SpawnDeleteManager

def main():
    if len(sys.argv) < 2:
        print("usage: pick_calibrate_go_return_pipeline.py speak_active")
    else:
        speak_active = (sys.argv[1] == "true")

        rospy.init_node("sim_webserver_demo_node", log_level=rospy.INFO)

        barista_feedback_obj = BaristaFeedback()
        loadsensor_calibration_obj = LoadSensorCalibrateManager()
        move_barista_manage_obj = MoveBaristaManager(barista_feedback_obj=barista_feedback_obj,
                                                    loadsensor_calibration_obj=loadsensor_calibration_obj,
                                                    move_max_time=60.0)

        refresh_rate = 10.0
        rate = rospy.Rate(refresh_rate)

        if speak_active:
            barista_feedback_obj.say("home")
        move_barista_manage_obj.move_to_waypoint(table_name="X2",
                                                flag_check_load_ok=False)

        while not rospy.is_shutdown():

            # Spawn of Objects
            #raw_input("Press Key to Spawn objects onto Barista...")
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

            #We check that the Object was loaded and calibrated
            id = -1
            msg = "..."
            # We look now for "Object loaded,2"
            while id != 2 and not rospy.is_shutdown():
                id, msg = loadsensor_calibration_obj._update_newest_load_sensor_info()
                rospy.loginfo("Msg="+str(msg)+",Id="+str(id))
                if id == 3:
                    rospy.logwarn("Please Hit Calibrate before issueing any table command...")
                rate.sleep()
            rospy.loginfo("Object loaded Detected...")

            # Move to Given Table through webserver
            if speak_active:
                barista_feedback_obj.say("work")
            move_ok, msg = move_barista_manage_obj.wait_for_waypoint_from_as(flag_check_load_ok=True,
                                                                            flag_talk=False)
            if move_ok:

                # When arrived, ask to pick object. Wait and repeat until object picked
                # We look now for "Object was removed,1"
                id = -1
                # We have to put random times to delete

                talking_period = 2.0
                init_seconds_speak = rospy.get_time()

                rate_ask_delete = rospy.Rate(1)

                if speak_active:
                    barista_feedback_obj.say("getorder")
                deleted_objects = False
                while id != 1 and not rospy.is_shutdown():

                    id, msg = loadsensor_calibration_obj._update_newest_load_sensor_info()
                    rospy.loginfo("Msg="+str(msg)+",Id="+str(id))
                    rate_ask_delete.sleep()

                    # We check if it has passed this time
                    now_seconds = rospy.get_time()
                    delta_speak = now_seconds - init_seconds_speak
                    if not deleted_objects:
                        answer = raw_input("Want to delete objects y-[n]")
                        if answer == "y":
                            spawn_delete_models_manager.delete_test_objects()
                            deleted_objects = True
                    else:
                        rospy.logwarn("Object not deleted Yet...")
                    if delta_speak > talking_period:
                        if speak_active:
                            barista_feedback_obj.say("getorder")
                        init_seconds_speak = rospy.get_time()

                if speak_active:
                    barista_feedback_obj.say("thankyou")

            else:
                # The move base had some issue so lets remove the objects
                raw_input("Something went wrong in move, press any key to delete objects")
                spawn_delete_models_manager.delete_test_objects()

            # Reset LoadSensor and Move to Table X1 ( Loading Table )
            ok, message = loadsensor_calibration_obj.reset()
            if not ok:
                rospy.logerr("Error in Calibration="+str(message))
                break

            if speak_active:
                barista_feedback_obj.say("home")
            move_barista_manage_obj.move_to_waypoint(table_name="X2",
                                                    flag_check_load_ok=False)
            if speak_active:
                barista_feedback_obj.say("finished")

            rospy.logwarn("######################### END OF LOOP....")
            rospy.logwarn("######################### RESTARTING....")

if __name__ == "__main__":
    main()