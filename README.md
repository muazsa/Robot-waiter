# Robot-waiter
Web controlled Robot-waiter with Turtlebot3 using ROS, Rviz and Gazebo simulator 	


This project is about simulating a robot that serves drinks to the tables in a coffee shop
or a bar. For that purpose, we are going to use a Turtlebot3 (Burger) robot.
We are going to achieve this by doing several things:<br/><br/>
Properly configure the ROS navigation stack in the robot in order to make the robot
create a map of the environment.<br/>
Configure the ROS navigation stack to make the robot localize and navigate in the
environment, making use of the previously generated map. <br/>
Create a ROS program that allows to receive the number of a table through a topic,
that would be the table that has requested a drink.<br/>
Then, once the bartender published the number in the topic using the webpage, the robot must go to that table and wait for
other commands, normally it’s gonna be to the bar stand ot it can be to any other table.


![](Images/default_gzclient_camera(1)-2019-12-09T00_16_44.551922.jpg)

![](Images/default_gzclient_camera(1)-2019-12-09T00_15_52.017305.jpg)



![](Images/rviz_screenshot_2019_12_09-00_21_35.png)
