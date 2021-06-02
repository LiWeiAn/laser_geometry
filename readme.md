# ROS Laser scan to ROS point cloud conversion

This repository is converting laser scan messages to point cloud messages in ROS2 using the laser_geometry package from: [https://github.com/ros-perception/laser_geometry].
The code does not publish a tf-transform. 

## Installation

Install the laser_geometry package with:

```bash
sudo apt-get install ........
```

And clone this repository.

## Usage

```python
import foobar

foobar.pluralize('word') # returns 'words'
foobar.pluralize('goose') # returns 'geese'
foobar.singularize('phenomena') # returns 'phenomenon'
```
