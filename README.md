# Table of Contents
1. [About](#about)
2. [Parameter Specification](#parameter-specification)
3. [Usage](#usage)

***

## About
This ROS package is intended to support searches for particular predefined shape primitives in pointclouds. Currently it supports locating planes and cylidners within a pointcloud scene with expected locations and orientations (or without these specified). This package depends on the [pointcloud_processing_server](https://github.com/UTNuclearRoboticsPublic/pointcloud_processing_server)

<img src=images/tunnel_inspection.png width="400">
The above image illustrates an example, with the server finding three planes (two walls and the floor) and two cylindrical ducts within an example scene, recorded from real data. 

## Parameter Specification
The behavior of the server is specified in yaml files - an example is provided in param/tunnel_search.yaml. The yaml file structure is based on that provided in the [pointcloud_processing_server](https://github.com/UTNuclearRoboticsPublic/pointcloud_processing_server), so reference that package's readme for more information on those parameters. 

The base of the yaml file includes the following general, cross-primitive parameters:

- **min_cloud_size**: if the cloud size being processed goes below this threshold, the service will fail and exit
- **task_list**: the list of tasks to be performed. These names MUST match the names given in the dictionary of task definitions below. The order here defines their order in the actual processing pipeline, not the order in the dictionary. 
- **plane_options**: these expose default options for plane segmentation searches, for when these options are not specified for a particular task
- **cylinder_options**: these expose default options for cylinder segmentation searches, for when these options are not specified for a particular task
- **map_offset**: this gives the baseline transform in the input cloud from the actual map in which primitive pose defitions are given below. This is intended to provide the initial offset, with an updating routine being available to make changes later on (although this isn't implemented yet).

As well, there is a dictionary of primitive defitions under the **tasks** heading. Each task is a kind of RANSAC segmentation with parameters based on the [pointcloud_processing_server](https://github.com/UTNuclearRoboticsPublic/pointcloud_processing_server). The tasks also have a set of extra parameters specific to the primitive_search which are not included in the pointcloud_processing_server. Explanations for those follow: 
- **angle_threshold**: allowable offset in found primitive *orientation* from the expected orientation.
- **offset_threshold**: allowable offset in found primitive *position* from the expected position 
- **check_orientation**: if this is false, the orientation of the found primitive will not be used as a selection criterion
- **check_distance**: if this is false, the position of the found primitive will not be used as a selection criterion
- **expected_coefficients**: this provides the list of primitive-specific coefficients to look for. 
  - Planes: A, B, C, D for the plane Ax+By+Cz+D = 0
  - Cylinders: xo, yo, zo, xd, yd, zd, r for a cylinder along the line <xd,yd,zd> passing through the point (xo,yo,zo) with radius r
- **clip_boundaries**: prior to a search, the cloud is clipped to the vicinity of the expected primitive location. This gives the dimensions of the clipping bounds in which the search is made. Make these values large to effectively skip this step
  - Planes: +X,-X,+Y,-Y,+Z,-Z, where X is the direction normal to the plane and Y is horizontal.
  - Cylinders: +X,-X,+Y,-Y,+Z,-Z, where X is the direction along the cylinder axis and Y is horizontal

## Usage
An example launch file usage can be run by launching the launch/tunnel_search.launch file. 
```
roslaunch pointcloud_primitive_search tunnel_search.launch
```

The user can also construct their own client file to interface with the pointcloud_primitive_search server. If doing so, all that should be necessary is to run the associated client file as well as the pointcloud_painter.srv node itself and the necessary yaml file:

```
rosload param/tunnel_search.yaml
rosrun pointcloud_primitive_search pointcloud_painter
```

It is currently necessary to load the yaml file data before loading the server. Failing to do so will keep the server from initializing correctly. There should be more options soon to refresh the server based on changes to the ros parameters (eg loading the yaml file later, or updating ros parameters manually to make changes). 
