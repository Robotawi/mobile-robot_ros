# ROS - Mobile Robot
This is a ROS melodic workspace created on Ubuntu 18.04. The robot package in this class has two nodes. The first one builds a mobile robot from scratch, includes sensors and actuators plugins, and incorporate the robot in a world file. The world file is built after my house. It is available [here](https://github.com/Robotawi/gazebo_world).

## Installation

- #### Prerequisite
    - You should have ROS melodic on your ubuntu 18.04.
    - All ROS dependency is satisfied.
    - Gazebo simulator is installed.

- #### Clone

    ```
    git clone https://github.com/Robotawi/mobile-robot_ros.git
    ```

- #### Setup
    ```
    cd mobile-robot_ros
    catkin_make
    source ./devel/setup.bash
    ```
## Package description
This package incorporates two nodes. The first is responsible for spawning the robot model in Gazebo simulator. The second is responsible to visual perception and actuation of the robot.The robot is set to follow a white ball that appears in the view of its camera, and stop otherwise. 

**To launch the simulation environment:**
```
roslaunch my_robot world.launch
```

- #### The above command will do the following
    - Set the robot description param and include the robot_description.launch file.
    - Set the world file and the robot initial pose in it.
    - Start Gazebo, load the world, and spawn the robot model.
    - Start rviz for visualization of the robot sensors. (uncomment if needed)
  
The mobile robot model includes a camera and a LIDAR sensor.

![](./pkg_images/rbt_model.png)



**To launch the visual perception and actuation nodes:**
```
roslaunch ball_chaser ball_chaser.launch
```
- #### The above command will do the following
    - Start the vision processing node and detect where the white object is, on the left, middle, or the right of the camera image.
    - Start the actuation node and drive the robot based on the visual feedback returned from the above node.

The following shows the robot motion and the returned camera images

![](./pkg_images/mobile_robot15.gif)

 The camera image is visualized through `rqt_image_view` node
 ```
 rosrun rqt_image_view rqt_image_view
 ```
 Then choose the topic `/camera/rgb/image_raw` from the list.


## Contact
In this project, I built everything from scratch because I love to understand how the internals of ROS work. This is a step towards my aim to actively contribute to robotics open-source software.

If you are interested in the presented work/ideas or if you have any questions, please feel free to connect with me on [LinkedIn](https://www.linkedin.com/in/mohraess). We can discuss about this project and other interesting projects.