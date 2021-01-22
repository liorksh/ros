# ROS (Robotic Operation System) Demo Application

This test application exemplifies a simple ROS Publisher-Subscriber application. 

Follow the script below to install this ROS application in your ROS workspace:
```
#navigate to the workspase (/devel)<br>
$ roscd
#go up to the workspace root<br>
$ cd ..
#get the source code from GitHub<br>
$ git clone https://github.com/liorksh/ros
#move the package to the src directory<br>
$ mv ./ros/chatwithme ./src
#remove the leftovers from Github (to keep a clean environment)<br>
$ rm -f -r ./ros
#build the new package<br>
$ catkin_make
#make the python file as executable<br>
$ sudo chmod +x ./src/chatwithme/src/talker.py
$ sudo chmod +x ./src/chatwithme/src/listener.py
```
Read the full article about [the foundations of ROS programming](https://liorshalom.com/2020/12/23/robot-operation-system-ros/)


