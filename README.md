[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)


# ROS Beginner Tutorials - Introduction to Publisher and Subscriber, w/ Additional ROS service #

### Overview

Here you will see an introduction to what a publisher and subscriber is in ROS. These two (publisher and subscriber) nodes demonstrate how these interact with one another. To get a better understanding of what each one of these are, please follow the tutorials listed below. These will start from the most basic, such as navigating the ROS filesystem, creating a package, building a package, and understanding nodes. Please follow links below.

- [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
- [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [Building a ROS Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
- [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
- [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
- [Writing a Simple Publisher and Subcriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
- [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)


Tutorials for services and clients on ROS. Follow tutorials below to have a better understanding of how these work.

- [Getting started with roswtf](http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf)
- [Understanding ROS Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
- [Using rqt_console and roslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
- [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

Tutorials for running ROSTEST and GTESTS

- [Writing rostest Files](http://wiki.ros.org/rostest/Writing)
- [gtest](http://wiki.ros.org/gtest)

Tutorials for ROSBAG

- [Recording and playing back data](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)

Basic important Tutorials for tf

- [Introduction to tf](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf)
- [Writing a tf broadcaster (C++)](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29)
- [Writing a tf listener (C++)](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29)


**The last 3 links of the publisher and subscriber tutorials are the most important to get started.**

**Last link of service and clientes is the most important for writing a simple service and client in ROS. This same tutorial was followed for the Simple Service and Client section of this repo.**

**The tutorials for ROSTEST, GTEST, and introduction to tf are important for this repo.**


## License

BSD License
Copyright (c) 2019 - Pablo Sanhueza

```
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```


### Dependencies

Program is meand to run on Ubuntu 16.04LTS and ROS Kinetic.

- Install ROS Kinetic. Make sure you follow all steps. Please follow link [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- Create a catkin Package. Catkin needs to be installed. Follow link [here](http://wiki.ros.org/catkin)



### Running the code

First, you need to be in a catkin workspace. Please see the 2nd and 3rd link in first part of the Overview.

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ cd src/
$ git clone -b https://github.com/sanhuezapablo/beginner_tutorials.git
$ cd ..
$ catkin_make
```


If you did not see the tutorials linked, this is how you create a catkin workspace. This includes running catkin_make.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone -b https://github.com/sanhuezapablo/beginner_tutorials.git
$ cd ..
$ catkin_make
```


Finally, we can run the code. Here we will run the 2 nodes. The publisher is the talker.cpp, and the subscriber is the listener.cpp. 

First, run **roscore** in a terminal

```
$ roscore
```


Open a new terminal. In your catkin workspace run the commands below. Here we will run the pusblisher (**talker**).

```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker
```


Now, open a new terminal (again), and follow comands below. Here we will run the subscriber (**listener**)

```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener
```

**Now you will see these 2 nodes communicating**


## Simple Service (C++) in ROS

Before running a simple service in ROS, we will create a launch file. This is included in the repo. Since this has already been created, we will run the launch file. This launch file will run both nodes (the talker and listener). This allows us to do the same as we did before, but this time instead of running 2 nodes independently, not we will be able to just open one terminal and launch both nodes. Remember that you need to be inside the cakin workspace.

```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch beginner_tutorials launch_talker_listener.launch
```

This launch file is set to a default frequency of 1 Hz. To change this please add the desired frequency in replacement of <new_frequency> in the commands below.

```
$ source ./devel/setup.bash
$ roslaunch beginner_tutorials launch_talker_listener.launch freq:=<new_freq>
```

### Running and Calling Service to change default String

Here the service will change the default string being published by our talker.cpp source code. While having both nodes running at the default or your desired frequency, now open a **new terminal** and run the commands below.

```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosservice call /set_string "input: '<new_string>'"
```

Now you should see a change in string being published by the talker.


### Viewing & Inspecting tf frames

Here we are able to visualize and inspect what a tf broadcaster does. A tf broadcaster broadcasts the transformation between the /world and /talker frame. Additionaly, for this specific repo we have a time varying broadcaster (check source codes for more information). To run the commands below, please keep in mind that you need to have either the talker node running, or both the talker and listener node. The steps shown to run these nodes are displayed in earlier sections by using the launch file launch_talker_listener.launch.

```
$ rosrun tf tf_echo /world /talk
```

To visualize this we can run the command below in a new terminal.

```
$ rosrun rqt_tf_tree rqt_tf_tree
```

Graphical form of what we saw in the previous command. Run command below. This will generate a pdf.

```
$ rosrun tf view_frames
```

To open the pdf we run:

```
$ evince frames.pdf
```

### Running rostest (gtests)

For this particular repo I have decided to test that the service call has been initialized properly. Additionally, we will test if the service input and output match. All of this is done in one TestSuite.

To build the code and also run rostest, please follow commands below.

```
$ cd ~/catkin_ws
$ catkin_make run_tests_beginner_tutorials
```

Another form of running rostest is by using the launch (test.launch) file if the code has already been compiled.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rostest beginner_tutorials test.launch
```

Also, we can run the tests if the talker node is running. Follow steps displayed in previous sections to run talker node.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials test_talker
```

### Using rosbag with launch file

Here we will use a bag file that records topic messages of the talker node. Later, we will run the listener node and play the information stored in the bag file (results_talker.bag). The rosbag is saved as results_talker.bag in the results directory.

**Keep in mind that by not passing the "record:=enable" argument you will not record anything. The launch file is by default disabled to not record.**

Running the launch file and passing the argument to enable recording.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials launch_talker_listener.launch record:=enable
```

## Inspect bag file

To inspect the information in the bag file we need to be in the results directory of this repo. Make sure you are in this directory if not you will get an error in the terminal. Commands below to go into the directory specified in addition to viewing bag information.

```
$ cd ~/catkin_ws/src/beginner_tutorials/results
$ rosbag info results_talker.bag
```

## Play recorded bag

Once the bag has been recorded, you can run the listener node and play the content of the results_talker.bag

First, open a new terminal and run the listener node.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials listener
```

Now open a different terminal and play recorded bag.

```
$ cd ~/catkin_ws/src/beginner_tutorials/results
$ rosbag play results_talker.bag
```

**Now go back to the terminal where the listener node is running, and you will be able to see the information in the recorded bag file.**


