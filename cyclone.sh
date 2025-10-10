#!/bin/bash

# Add the RMW_IMPLEMENTATION environment variable to bashrc for ROS 2 CycloneDDS middleware
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# Add a line to bashrc to echo the path to the cyclone_config.xml file on each new shell
echo 'echo $HOME/cyclone_config.xml' >> ~/.bashrc
