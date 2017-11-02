This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Team Info

* Team Name:autoDrive

* Team Leader:
  * @ckyrkou ckyrkou@gmail.com   Traffic Light detection

* Team Member: 
  * @wowdd1  developergf@gmail.com  twist controller
  * @hello2all point follower
  * @jzhang  wszjzhang@gmail.com  Traffic Light Classifier
  * @lacfo  liyuanlacfo1990@gmail.com  planning

## Project Overview
### System Architecture Diagram
For this project, we'll be writing ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following! You will test your code using a simulator, and when you are ready, your group can submit the project to be run on Carla.

The following is a system architecture diagram showing the ROS nodes and topics used in the project. You can refer to the diagram throughout the project as needed. The ROS nodes and topics shown in the diagram are described briefly in the Code Structure section below, and more detail is provided for each node in later classroom concepts of this lesson.
![alt text](./final-project-ros-graph-v2.png)

### Code Structure
Below is a brief overview of the repo structure, along with descriptions of the ROS nodes. The code that you will need to modify for the project will be contained entirely within the (path\_to\_project\_repo)/ros/src/ directory. Within this directory, you will find the following ROS packages:

#### (path\_to\_project\_repo)/ros/src/tl\_detector/
This package contains the traffic light detection node: tl\_detector.py. This node takes in data from the /image\_color, /current\_pose, and /base\_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic\_waypoint topic.

The /current_pose topic provides the vehicle's current position, and /base\_waypoints provides a complete list of waypoints the car will be following.

You will build both a traffic light detection node and a traffic light classification node. Traffic light detection should take place within tl_detector.py, whereas traffic light classification should take place within ../tl\_detector/light\_classification\_model/tl\_classfier.py.
![alt text](./tl-detector-ros-graph.png)

#### (path\_to\_project\_repo)/ros/src/waypoint\_updater/
This package contains the waypoint updater node: waypoint_updater.py. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the /base\_waypoints, /current\_pose, /obstacle\_waypoint, and /traffic\_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final\_waypoints topic.
![alt text](waypoint-updater-ros-graph.png)

#### (path\_to\_project\_repo)/ros/src/twist\_controller/
Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw\_node.py and the file twist\_controller.py, along with a pid and lowpass filter that you can use in your implementation. The dbw\_node subscribes to the /current\_velocity topic along with the /twist\_cmd topic to receive target linear and angular velocities. Additionally, this node will subscribe to /vehicle/dbw\_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle\_cmd, /vehicle/brake\_cmd, and /vehicle/steering\_cmd topics.
![alt text](dbw-node-ros-graph.png)

In addition to these packages you will find the following, which are not necessary to change for the project. The styx and styx_msgs packages are used to provide a link between the simulator and ROS, and to provide custom ROS message types:

* **(path\_to\_project\_repo)/ros/src/styx/**
A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
* **(path\_to\_project\_repo)/ros/src/styx\_msgs/**
A package which includes definitions of the custom ROS message types used in the project.
* **(path\_to\_project\_repo)/ros/src/waypoint\_loader/**
A package which loads the static waypoint data and publishes to /base_waypoints.
* **(path\_to\_project\_repo)/ros/src/waypoint\_follower/**
A package containing code from Autoware which subscribes to /final\_waypoints and publishes target vehicle linear and angular velocities in the form of twist commands to the /twist\_cmd topic.

## Suggested Order of Project Development
Because you will be writing code across several packages with some nodes depending on messages published by other nodes, we suggest completing the project in the following order:

1. Waypoint Updater Node (Partial): Complete a partial waypoint updater which subscribes to /base\_waypoints and /current\_pose and publishes to /final\_waypoints.
2. DBW Node: Once your waypoint updater is publishing /final_waypoints, the waypoint\_follower node will start publishing messages to the/twist\_cmd topic. At this point, you have everything needed to build the dbw\_node. After completing this step, the car should drive in the simulator, ignoring the traffic lights.
3. Traffic Light Detection: This can be split into 2 parts:
	1. Detection: Detect the traffic light and its color from the /image\_color. The topic /vehicle/traffic\_lights contains the exact location and status of all traffic lights in simulator, so you can test your output.
	2. Waypoint publishing: Once you have correctly identified the traffic light and determined its position, you can convert it to a waypoint index and publish it.
4. Waypoint Updater (Full): Use /traffic\_waypoint to change the waypoint target velocities before publishing to /final\_waypoints. Your car should now stop at red traffic lights and move when they are green.

## Project preparation

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
