Sharing insights from my Master Thesis work, especially valuable for those looking to implement Openpose application in a ROS environment!

Encountering several challenges building Openpose software on Ubuntu 20.04 LTS led to a important decision: leveraging Docker for a seamless implementation. This involved configuring a Docker image based on Ubuntu 18.04, incorporating the ROS Melodic distribution, the Openpose application, Openpose ROS wrapper(specifically with Microsoft Kinect RGB-D Camera v1), CUDA, CuDNN, and other essential dependencies.
- Docker Image File: [GitHub - Custom Docker Image](https://github.com/vin3697/docker_image_ros_openpose/blob/main/docker_ros_openpose/Dockerfile)
- Docker Hub Link: [Openpose Docker Image](https://hub.docker.com/r/vin8/openpose_vin)

Emphasizing the importance of the NVIDIA Container Toolkit and Docker configuration for GPU utilization within the Docker container, enhancing computational efficiency.
- Configuring Docker: [NVIDIA Container Toolkit Setup](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)
- Toolkit Installation: [NVIDIA Container Toolkit Installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)

The journey involved overcoming hurdles, culminating in a robust solution for the seamless integration of Openpose within the ROS framework, enabling the implementation of Gesture Control for Robot Movements. 

- Openpose Application: [Openpose GitHub Repository](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
- Openpose ROS wrapper: [ROS Openpose GitHub Repository](https://github.com/ravijo/ros_openpose)

Hardware Setup : Microsoft Kinect RGB-D Camera v1, YD LiDAR X4 and Ohmni Telepresence Robot.
#Openpose #ROS #Docker #ComputerVision
