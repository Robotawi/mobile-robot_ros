# ROS - Mobile Robot

This is a ROS workspace created on Ubuntu 18.04. The workspace is for the simulation of a differential mobile robot built from scratch. The robots includes sensors and actuators plugins. The robot is simulated in a a world file that I created based on the design of my house. The world environment is available [here](https://github.com/Robotawi/gazebo_world), if you like to download and use it.

## Installation

- #### Prerequisite
    - ROS melodic on ubuntu 18.04.
    - All ROS dependencies are satisfied.
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
There are two packages in this ROS workspace. The first is responsible for spawning the robot model in Gazebo simulator. The second incorporates two nodes which implement visual perception and actuation of the robot. The robot is set to follow a white ball that appears in the view of its camera, and stop otherwise. 

![](./pkg_images/mobile_robot18.gif)

**To launch the simulation world:**
```
roslaunch my_robot world.launch
```

**The above roslaunch command does the following:**
- Set the robot description param and include the robot_description.launch file.
- Set the world file and the robot initial pose in it.
- Start Gazebo, load the world, and spawn the robot model.
- Start rviz for visualization of the robot sensors.
  
**To launch the visual perception and actuation nodes:**
```
roslaunch ball_chaser ball_chaser.launch
```



**The above command will do the following:**
- Start the vision processing node and detect the white ball.
- Start the actuation node and drive the robot based on the visual feedback returned from the above node.

 The camera image is visualized through `rqt_image_view` node. The camera topic is `/camera/rgb/image_raw`.
```
 rosrun rqt_image_view rqt_image_view 
```




## The robot design

The robot is a differential drive mobile robot that I bulit from scratch. The model includest two sensors; a camera and a LIDAR as shown below.

![](./pkg_images/rbt_model_new.png)

## Solving the localization problem

Localization is the problem of estimating the robot's pose (position and orientation) within a known map of the environment. To solve this problem, I used Adaptive Monte Carlo Localization (AMCL), and its ROS package for mobile robot localization. AMCL is part of the ROS Navigation Stack, which provides a collection of packages for autonomous navigation of mobile robots.

AMCL is a probabilistic algorithm that uses a particle filter to estimate the robot's pose (position and orientation) in a given environment. It works by sampling a set of particles, where each particle represents a possible pose of the robot. These particles are then propagated through the environment based on the robot's motion and sensor readings.

As the robot moves, the particle filter is updated using sensor data to weight each particle according to how well it matches the sensor data. The particles with higher weights are then resampled, and the process is repeated. Over time, the particles converge towards the robot's true pose, and the estimated pose becomes more accurate.

### The localization result

The following screenshot shows the robot pose in the simulated environment and the corresponding map. The poses are the same which indicates successful localization. 

![](./pkg_images/success_robot_localized.png)

## Solving the SLAM problem

SLAM (Simultaneous Localization and Mapping) is the problem of constructing a map of an unknown environment while simultaneously estimating the robot's pose (position and orientation) within that environment. I utilized RTAB-Map (Real-Time Appearance-Based Mapping) and its corresponding ROS package, which employs visual data  to solve the SLAM problem in real-time, in order to address this issue.

RTAB-Map uses a feature-based approach to extract keypoint features from visual data and then matches these features across different camera frames to construct a map. It also uses loop closure detection to detect revisited locations and to optimize the map over time. This combination of feature-based mapping and loop closure detection makes RTAB-Map a powerful tool for solving the SLAM problem.

### The resulting map results

The resulting map using Real-Time Appearance-Based Mapping 

<!-- ![](./pkg_images/success_map_no_graph.png) -->
<img src="./pkg_images/success_map_no_graph.png" alt="Image description" width="600">


One aspect of getting successful results is having loop closures, which are added in the following map. 

<!-- ![](./pkg_images/success_map_graph.png) -->
<img src="./pkg_images/success_map_graph.png" alt="Image description" width="600">

The resulting 3D map shows a correctly reconstructed environment from the sensors data. 

![](./pkg_images/success_3d_map_opt.gif)


## Autonomous navigation 

After solving the SLAM problem, we can use Rviz as well as programmed input to test the performance. 

Using Rviz GUI to move the robot: 

![](./pkg_images/success_nav_rviz.gif)

Using programmed input to move the robot: 

![](./pkg_images/success_nav_prog.gif)

## Contact
In this project, I built everything from scratch because I love to understand how the internals of ROS work. This is a step towards my aim to actively contribute to robotics open-source software.

If you are interested in the presented work/ideas or if you have any questions, please feel free to connect with me on [LinkedIn](https://www.linkedin.com/in/mohraess). We can discuss about this project and other interesting projects.
