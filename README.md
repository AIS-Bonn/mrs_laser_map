
mrs_laser_map - Local Multiresolution Grids for Efficient 3D Laser Mapping and Localization
=====================================================================

`mrs_laser_map` is a mapping system for mobile robots using laser range sensors. 
It provides efficient local mapping for obstacle perception, and allocentric mapping 
for autonomous navigation. It has been successfully used on different 
robotic platforms, such as micro aerial vehicles and ground robots, in 
different research projects and robotic competitions.

Features
--------

       
Overview 
--------

The repository contains the following ROS packages:

* `mrs_laser_maps`: map data structure and surfel-based registration
* `mrs_laser_mapping`: ROS nodes for local and allocentric mapping
* `config_server`: parameter server 
* `parameter_tuner`: rqt plugin for config server
* `mod_laser_filters`: laser filters for preprocessing
* `cloud_compression`: modified pcl cloud_compression 

        
Dependencies
---------------


* Ubuntu 14.04 

* [ROS Indigo](http://www.ros.org/wiki/indigo/Installation/Ubuntu):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install ros-indigo-desktop-full
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* [g2o](https://openslam.org/g2o.html):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
sudo apt-get install ros-indigo-libg2o
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Getting started
---------------

Clone the repository in your catkin workspace (here ~/catkin_ws)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
cd ~/catkin_ws/src # Or another path without spaces can be taken
git clone https://github.com/AIS-Bonn/mrs_laser_map mrs_laser_map
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

and build it:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
cd ~/catkin_ws/
catkin_make
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Download the sample [bagfile](http://www.ais.uni-bonn.de/~droeschel/bags/mav_2.bag) and run mav_mapping.launch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
roslaunch mrs_laser_mapping mav_mapping.launch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

License
-------

`mrs_laser_map` is licensed under the BSD 3-clause license.

Authors & Contact
-----------------

```
David Droeschel <droeschel@ais.uni-bonn.de>
Institute of Computer Science VI
Rheinische Friedrich-Wilhelms-Universität Bonn
Friedrich Ebert-Allee 144
53113 Bonn
```
