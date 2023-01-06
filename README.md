# ergonomic_assessment
This repo provides vision-based real-time ergonomic feedback based on the [Rapid Upper Limb Assessment (RULA)](https://doi.org/10.1016/0003-6870(93)90080-S).
Based on body tracking data, we calculate all relevant joint angles and use them to compute RULA scores.
This adds the detection of statis and repetive Motions.

# Demo Video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/GOXe7FzxO5g/0.jpg)](https://www.youtube.com/watch?v=GOXe7FzxO5g)

# Simplest Installation
- build the Docker Image: ``docker build -t musclework_docker_image:1.0 .``
- You need to have installed this repo: ``https://github.com/ignc-research/ergonomic_assessment``

# Launch ROS
- Start the ergonomic_assessment
- Then start the Docker file via ``docker run -it --net=host musclework_docker_image:1.0``



# Use Rosbags
To use recorded rosbags, run ``rosbag play './rosbags/body_tracking_data.bag' --loop``


