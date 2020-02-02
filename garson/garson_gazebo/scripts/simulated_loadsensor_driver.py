#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from gazebo_msgs.msg import ContactsState
import math

"""
rosmsg show gazebo_msgs/ContactState
string info
string collision1_name
string collision2_name
geometry_msgs/Wrench[] wrenches
  geometry_msgs/Vector3 force
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 torque
    float64 x
    float64 y
    float64 z
geometry_msgs/Wrench total_wrench
  geometry_msgs/Vector3 force
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 torque
    float64 x
    float64 y
    float64 z
geometry_msgs/Vector3[] contact_positions
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3[] contact_normals
  float64 x
  float64 y
  float64 z
float64[] depths
"""

class LoadSensorInfo(object):

    def __init__(self, gravity=9.81):

        self.min_pressure = 0.0
        self.max_pressure = 2.0

        self._gravity = gravity

        self.load_msg = Float32()
        self.load_msg.data = 0.0

        rospy.Subscriber("/loadsensor_link_contactsensor_state", ContactsState, self.loadsensor_callback)
        self._pub = rospy.Publisher('/load_sensor', Float32, queue_size=10)
        rospy.loginfo("Simulated LOAD Sensor Ready")

    def loadsensor_callback(self, data):
        """
        Converts Contact Sensor Data into Weight (mass in grams) of all the object detected
        :param data:
        :return:
        """
        if len(data.states) >= 1:
            total_force = 0
            objects_contact_list = []
            rospy.logdebug("##### 1 #######")
            for contact_state in data.states:
                collision1 = contact_state.collision1_name
                # standard_apple::link::collision
                object_name = collision1.split("::")[0]

                if object_name not in objects_contact_list:
                    objects_contact_list.append(object_name)
                    rospy.logdebug("New Model model_name=" + str(object_name))
                    Fx = contact_state.total_wrench.force.x
                    Fy = contact_state.total_wrench.force.y
                    Fz = contact_state.total_wrench.force.z
                    force_magnitude = math.sqrt(pow(Fx, 2) + pow(Fy, 2) + pow(Fz, 2))

                    total_force += force_magnitude
                    mass = force_magnitude / self._gravity
                    rospy.logdebug("ForceMagnitude =" + str(force_magnitude))
                    rospy.logdebug("mass =" + str(mass))
                    rospy.logdebug("[F=" + str(force_magnitude) + ",M=" + str(mass) + "]")

                else:
                    # We have already added that weight
                    pass

            total_mass = total_force / self._gravity
            rospy.logdebug("TotalForce =" + str(total_force))
            rospy.logdebug("TotalMass =" + str(total_mass))

            # We publish the mass in grams
            self.load_msg.data = total_mass * 1000
            rospy.logdebug("##### 0 #######")


        else:
            rospy.logdebug("NO object on top detected...")
            self.load_msg.data = 0.0

        self._pub.publish(self.load_msg)


if __name__ == '__main__':
    rospy.init_node('loadsensor_node', anonymous=True, log_level=rospy.INFO)
    loadsensor_object = LoadSensorInfo()
    rospy.spin()

