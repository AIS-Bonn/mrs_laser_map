
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

        
Getting started
---------------



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
