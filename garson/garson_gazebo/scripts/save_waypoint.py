#!/usr/bin/env python

import os
import rospy
from barista_gazebo.srv import SaveWaypoint, SaveWaypointRequest, SaveWaypointResponse
from ros_waypoint_generator.msg import WaypointArray
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from pickle_commons import update_pickle_file, read_pickle_file
import random
import copy

"""
rosmsg show ros_waypoint_generator/WaypointArray
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
ros_waypoint_generator/Waypoint[] waypoints
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  int32 number
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  int32 is_search_area
  float64 reach_tolerance
"""


"""
visualization_msgs/Marker[] markers
  uint8 ARROW=0
  uint8 CUBE=1
  uint8 SPHERE=2
  uint8 CYLINDER=3
  uint8 LINE_STRIP=4
  uint8 LINE_LIST=5
  uint8 CUBE_LIST=6
  uint8 SPHERE_LIST=7
  uint8 POINTS=8
  uint8 TEXT_VIEW_FACING=9
  uint8 MESH_RESOURCE=10
  uint8 TRIANGLE_LIST=11
  uint8 ADD=0
  uint8 MODIFY=0
  uint8 DELETE=2
  uint8 DELETEALL=3
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string ns
  int32 id
  int32 type
  int32 action
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  geometry_msgs/Vector3 scale
    float64 x
    float64 y
    float64 z
  std_msgs/ColorRGBA color
    float32 r
    float32 g
    float32 b
    float32 a
  duration lifetime
  bool frame_locked
  geometry_msgs/Point[] points
    float64 x
    float64 y
    float64 z
  std_msgs/ColorRGBA[] colors
    float32 r
    float32 g
    float32 b
    float32 a
  string text
  string mesh_resource
  bool mesh_use_embedded_materials

"""
class SaveWaypointClass():
    def __init__(self, init_check_for_waypoints=True):

        rospy.on_shutdown(self.shutdown)

        # Get the file path used to save csv
        self.way_points_file_path = rospy.get_param("~way_points_file_path", "saved_waypoints.pickle")  # meters
        self._way_points_dir_path = os.path.dirname(os.path.realpath(self.way_points_file_path))

        # Start the Service to manage the saving of the waypoints by name and save in a file
        s = rospy.Service('/waypoints_save_server', SaveWaypoint, self.save_waypoint_callback)

        # Initialize the visualization markers for RViz
        self.init_markers()
        self.init_saved_waypoints_dict()
        if init_check_for_waypoints:
            self._update_newest_waypoint()
        else:
            # We are not interested in waiting until we move .
            # Usefull when we want to use this for loading only waypoints.
            pass



    def init_saved_waypoints_dict(self):
        """
        We initialise the waypoonts dict where we will store all the waypoints saved
        {"NameOfWaypoint": waypoint}
        :return:
        """
        # We check if a previously waypoints files exists
        rospy.logdebug("Checking if file exists="+str(self.way_points_file_path))

        if not os.path.isdir(self._way_points_dir_path):
            # We create all the dirs until the designated path if it doesnt exist
            rospy.logerr("Folder doesnt exist, creating="+str(self._way_points_dir_path))
            os.makedirs(self._way_points_dir_path)
            rospy.logerr("Folder doesnt exist, creating..DONE")

        if os.path.isfile(self.way_points_file_path) :
            self.load_waypoints_file()
        else:
            self._saved_waypoints_dict = {}

    def add_waypoint_to_dict(self, name, waypoint):
        """
        We add the waypoint to the  dict self._saved_waypoints_dict
        {"NameOfWaypoint": waypoint}
        :return:
        """
        self._saved_waypoints_dict[name] = waypoint

    def get_waypoint_dict(self):
        return self._saved_waypoints_dict

    def save_waypoint_callback(self, request):
        """
        This service gets as request the name of a Table or waypoint.
        If its different from "SAVE", then we will store the last waypoint in a dictionary
        with the given name.
        If save is recieved , then we will get the waypoint dictionary and save it
        into a file.
        :param request:
        :return:
        """

        result = True
        waypoint_name = request.name
        rospy.logdebug("Server Save WayPoint , waypoint name=" + str(waypoint_name))

        if waypoint_name == "SAVE":
            rospy.logwarn("Executing order to save file...")
            self.save_waypoints_file()
            rospy.logwarn("Executing order to save file...DONE")
        else:
            rospy.logwarn("Executing order to save waypoint...")
            self._update_newest_waypoint()
            self.add_waypoint_to_dict(waypoint_name, self.last_waypoint)
            self.add_new_marker(waypoint_name, self.last_waypoint.pose.position)
            rospy.logwarn("Executing order to save waypoint...DONE")

        response = SaveWaypointResponse()

        response.success = result
        return response

    def load_waypoints_file(self):
        """
        It loads all the waypoints from an existing file
        :return:
        """
        rospy.logerr("Loading WayPoints File...")
        self._saved_waypoints_dict = read_pickle_file(pickle_file_path=self.way_points_file_path)
        self._load_file_waypoints_into_markers()
        rospy.logerr("Loading WayPoints File...DONE")


    def save_waypoints_file(self):
        """
        It dumps all the waypoints saved into a file
        :return:
        """
        rospy.logdebug("Saving WayPoints File...")
        update_pickle_file(data=self._saved_waypoints_dict,
                           pickle_file_path=self.way_points_file_path)
        rospy.logdebug("Saving WayPoints File...DONE")

    def _update_newest_waypoint(self):
        rospy.logdebug("Getting Newest Waypoint")
        self.last_waypoint = None
        while self.last_waypoint is None and not rospy.is_shutdown():
            try:
                waypoint_array = rospy.wait_for_message("/waypoints", WaypointArray, timeout=1.0)
                self.last_waypoint = waypoint_array.waypoints[-1]
            except Exception as e:
                rospy.logerr("Current /waypoints not ready yet, retrying..."+str(e))
        rospy.logdebug("Getting Newest Waypoint..DONE")

    def _load_file_waypoints_into_markers(self):
        """
        This will get the dict and load all the waypoints got from the picle file
        :return:
        """
        for name, waypoint in self._saved_waypoints_dict.items():
            self.add_new_marker(name, waypoint.pose.position)

    def add_new_marker(self, name, waypoint_position):
        """
        It appends the given waypoint position to the marker list to publish it
        :return:
        """
        rospy.logerr(">>>>>>>>>>>>>>Publishing New Marker...")
        # We check first that the marker doesnt exist already
        marker_exists = False
        i = 0
        for marker_obj in self.marker_array.markers:
            if marker_obj.text == name:
                marker_exists = True
                break
            i += 1

        new_marker = copy.deepcopy(self.marker)
        new_marker.pose.position = waypoint_position
        new_marker.text = name

        if marker_exists:
            waypoint_id = i
        else:
            waypoint_id = len(self.marker_array.markers)

        new_marker.id = waypoint_id

        if marker_exists:
            rospy.logerr("Marker EXISTS")
            self.marker_array.markers[i] = new_marker
        else:
            rospy.logerr("Marker DOESNT exist")
            self.marker_array.markers.append(new_marker)

        self._check_marker_publishers_connection()
        self.marker_array_pub.publish(self.marker_array)
        rospy.logerr("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Publishing New Marker...DONE")

    def _check_marker_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self.marker_array_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No subscribers to /saved_waypoint_markers yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("marker_array_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 2.0
        marker_lifetime = 0  # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0}

        # Define a marker publisher.
        self.marker_array_pub = rospy.Publisher('/saved_waypoint_markers', MarkerArray, queue_size=100)

        # Initialize the marker points list.
        self.marker_array = MarkerArray()
        
        self.marker = Marker()
        self.marker.ns = marker_ns
        self.marker.id = marker_id
        self.marker.type = Marker.TEXT_VIEW_FACING
        self.marker.action = Marker.ADD
        self.marker.lifetime = rospy.Duration(marker_lifetime)
        self.marker.scale.x = marker_scale
        self.marker.scale.y = marker_scale
        self.marker.scale.z = marker_scale
        self.marker.color.r = marker_color['r']
        self.marker.color.g = marker_color['g']
        self.marker.color.b = marker_color['b']
        self.marker.color.a = marker_color['a']

        self.marker.header.frame_id = 'map'
        self.marker.header.stamp = rospy.Time.now()
        #self.marker.points = list()

        self._check_marker_publishers_connection()



    def shutdown(self):
        rospy.logerr("Closing saving waypoints system...")
        # Cancel any active goals
        self.save_waypoints_file()
        rospy.logerr("Closing saving waypoints system...DONE")



if __name__ == '__main__':
    rospy.init_node('save_waypoint_node', anonymous=False, log_level=rospy.DEBUG)
    try:
        SaveWaypointClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Save waypoints finished.")