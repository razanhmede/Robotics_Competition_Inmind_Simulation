ros2 pkg create --build-type ament_cmake --node-name initNode control --dependencies rclcpp rclpy

# To Build The Base Image (build once)
sudo docker build -t ros_galactic_with_deps -f Dockerfile.base .

# To Build the Pip Image 
sudo docker build -t ros_galactic_with_pip -f Dockerfile.pip .

# To Build the custom image (takes less time)
sudo docker build -t my_ros2_application .

# To Run the container
sudo docker run --network host -it my_ros2_application 
or
sudo docker run -it my_ros2_application /bin/bash (to interact with the container)

# To run the gazebo world
copy the turtlebot3_waffle folder and the turtlebot3_common folder from /opt/ros/humble/share/turtlebot3_gazebo/models
and paste it in this path: ~/.gazebo/models 
then run the command to open gazebo: 
 gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so world.world 