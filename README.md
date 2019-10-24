# ROS Beginner Tutorials - Introduction to Publisher and Subscriber #

### Overview

Here you will see an introduction to what a publisher and subscriber is in ROS. These two (publisher and subscriber) nodes demonstrate how these interact with one another. To get a better understanding of what each one of these are, please follow the tutorials listed below. These will start from the most basic, such as navigatinr the ROS filesystem, creating a package, building a package, and understanding nodes. Please follow links below.

- [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
- [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [Building a ROS Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
- [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
- [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
- [Writing a Simple Publisher and Subcriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
- [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

**The last 3 links are the most important. These will explain how this repository was made and also how to run.**

### Dependencies

Program is meand to run on Ubuntu 16.04LTS and ROS Kinetic.

- Install ROS Kinetic. Make sure you follow all steps. Please follow link [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- Create a catkin Package. Catkin needs to be installed. Follow link [here](http://wiki.ros.org/catkin)



### Running the code

First, you need to be in a catkin workspace. Please see the 2nd and 3rd link in Overview.

'''
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ cd src/
$ git clone -b https://github.com/sanhuezapablo/beginner_tutorials.git
$ cd ..
$ catkin_make
'''


If you did not see the tutorials linked, this is how you create a catkin workspace. This includes running catkin_make.

'''

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone -b https://github.com/sanhuezapablo/beginner_tutorials.git
$ cd ..
$ catkin_make
'''


Finally, we can run the code. Here we will run the 2 nodes. The publisher is the talker.cpp, and the subscriber is the listener.cpp. 

First, run **roscore** in a terminal

'''

$ roscore
'''


Open a new terminal. In your catkin workspace run the commands below. Here we will run the pusblisher (**talker**).

'''

$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker
'''


Now, open a new terminal (again), and follow comands below. Here we will run the subscriber (**listener**)

'''

$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener
'''

**Now you will see these 2 nodes communicating**


