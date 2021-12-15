# Path_coverage_ros


#  Area coverage simulation of multiple cleaning robot #

clone this to src of your workspace
go to your workspace
```
cd ..
```
make with catkin_make
```
catkin_make
```
source 
```
source devel/setup.bash
```
To launch 1st map 
```
roslaunch multi_robot main.launch
```
To launch 2nd map 
```
roslaunch multi_robot main_world_2.launch
```
To launch 3rd map 
```
roslaunch multi_robot main_world_3.launch
```
To launch 4th map 
```
roslaunch multi_robot main_world_4.launch
```
To navigate run navs.py
```
rosrun multi_robot navs.py

## You are good to go ##
