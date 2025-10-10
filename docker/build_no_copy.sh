#!/bin/bash

# Optionally clone ABB driver repositories (commented out)
# git clone -b marco-sg-control git@github.com:MerlinLaboratory/abb_librws_2.0.git
# git clone -b cari git@github.com:MerlinLaboratory/abb_omnicore_ros2.git

# Example: Build and push multi-architecture Docker image using buildx (commented out)
# docker buildx build --no-cache --platform linux/amd64,linux/arm64 --network=host --ssh default -t smentasti/drims2:2025 --push . 

# Example: Build and push Docker image to different tags (commented out)
# docker build  --network=host -t gabrinovas/drims2:2025 --push . 
# docker build  --network=host -t drims2:2025 --push . 

# Build and push the Docker image locally with tag 'drims2:local'
docker build  --network=host -t drims2:local --push . 

# Alternative: Build without pushing to a registry (commented out)
# docker build  --network=host -t smentasti/drims2:2025 .
