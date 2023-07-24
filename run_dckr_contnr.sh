#!/bin/bash

# Grant permission to root user to access the X server
sudo xhost +si:localuser:root

# Start the Docker container named "vibrant_buck"
docker start vibrant_buck

# Attach the Docker container named "vibrant_buck"
docker attach vibrant_buck