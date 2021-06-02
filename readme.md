# ROS laser scan to point cloud conversion 

This repository is converting laser scan messages to point cloud messages in ROS2 using the laser_geometry package from: [https://github.com/ros-perception/laser_geometry].
The code does not publish a tf-transform. 

## Installation

Install the laser_geometry package with:

```bash
sudo apt-get install ........
```

And clone this repository in a ROS workspace.

## Usage
Change the subscription topic "scan" to your topic: 
```
subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10 , std::bind(&MinimalPublisher::scanCallback, this, _1));
```

Run the node:
```
ros2 run laser_geometry laser2pc_node
```
