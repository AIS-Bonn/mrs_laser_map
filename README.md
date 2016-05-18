
mrs_laser_map - Local Multiresolution Grids for Efficient 3D Laser Mapping and Localization
=====================================================================

`mrs_laser_map` is a mapping system for mobile robots using laser range sensors. 
It provides efficient local mapping for obstacle perception, and allocentric mapping 
for autonomous navigation. It has been successfully used on different 
robotic platforms, such as micro aerial vehicles and ground robots, in 
different research projects and robotic competitions.


Papers Describing the Approach 
--------

David Droeschel, Jörg Stückler, and Sven Behnke: [Local Multi-Resolution Representation for 6D Motion Estimation and Mapping with a Continuously Rotating 3D Laser Scanner](http://ais.uni-bonn.de/papers/ICRA_2014_Droeschel.pdf) IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, May 2014 

David Droeschel, Jörg Stückler, and Sven Behnke: [Local Multi-Resolution Surfel Grids for MAV Motion Estimation and 3D Mapping](http://ais.uni-bonn.de/papers/IAS_2014_Droeschel_MAV-Mapping.pdf) In Proceedings of 13th International Conference on Intelligent Autonomous Systems (IAS), Padova, Italy, July 2014. 

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

Make sure you have the latest libg2o ROS package, previous versions were compiled without optimization.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
droeschel@flanders:~/catkin_ws$ apt-cache policy ros-indigo-libg2o
ros-indigo-libg2o:
  Installed: 2014.2.18-0trusty-20160321-175501-0700
  Candidate: 2014.2.18-0trusty-20160321-175501-0700
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Getting started
---------------

Clone the repository in your catkin workspace (here ~/catkin_ws)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
cd ~/catkin_ws/src 
git clone https://github.com/AIS-Bonn/mrs_laser_map mrs_laser_map
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

and build it with catkin_make (or catkin tools):
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

Contact
-----------------

```
David Droeschel <droeschel@ais.uni-bonn.de>
Institute of Computer Science VI
Rheinische Friedrich-Wilhelms-Universität Bonn
Friedrich Ebert-Allee 144
53113 Bonn
```
