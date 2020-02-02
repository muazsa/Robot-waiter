cd ~/arduino-1.8.10
sudo .install.sh

sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial

# Install ros_lib in arduino
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
ls
# You should see now >>> readme.txt  ros_lib

# Enable Loading Sketches in Arduino
ls -l /dev/ttyACM*
# An error occurred while uploading the sketch avrdude: ser_open(): can't open device "/dev/ttyACM0": Permission denied
# Error permission denied >> crw-rw---- 1 root dialout 166, 0 oct 20 08:49 /dev/ttyACM0
# To fix this: https://forum.arduino.cc/index.php?topic=495039.0

# OPTION1: Fast buts have to do it every time
sudo chmod a+rw /dev/ttyACM0
ls -l /dev/ttyACM*
# OPTION2: Add your user to the group, it should work always


# >> crw-rw-rw- 1 root dialout 166, 0 oct 20 08:49 /dev/ttyACM0
# Select this port nw in the IDE >Tools

# Upload the example HelloWorld or ros_lib examples in ARDUINO

# Start example
roscore

# OP1: Normal option
rosrun rosserial_python serial_node.py /dev/ttyACM0

# Listen
rostopic echo chatter


### INSTALL THE load library
cp -r barista_hardware/load_sensor/HX711  ~/Arduino/libraries/

