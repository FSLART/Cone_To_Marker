# Cones_To_Markers

A simple ros2 message converter.

## Description

A simple ros2 package that converts the Lart ConeArray Topic message to a MarkerArray message to visualize in rviz or foxglove studio.

## Getting Started

### Dependencies

#### Bare Metal

* ros2 (humble if possible)
* [lart_msgs](https://github.com/FSLART/lart_msgs)

#### Dockerized

* docker engine / docker desktop

### Installing

#### Bare Metal

Go to your ros2 workspace.

Clone this repo to your src directory.

`Build the package from workspace root`

```shell
colcon build
source ./install/setup.sh
```

#### Dockerized

Clone this repo and go to the root directory

Give execution priviledge to `fetch_git.sh`.

And then run this:
```shell
./fetch_git.sh
docker build . -t cone_marker
```

### Executing program

#### Bare Metal
 
```shell
ros2 run cone_markers mark_cones


## debug commands
# Publisher
ros2 run cone_markers debug_publisher

# Subscriber
ros2 run cone_markers debug_subscriber
```

#### Dockerized
```shell
docker run cone_marker
```