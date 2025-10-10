#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Clean up all unused Docker data, images, containers, and networks
echo "🧹 Cleaning everything..."
docker system prune -a -f

# Build the Docker image without using cache, tagging it as 'my_drims2:local'
echo "🚀 Building with no cache..."
docker build \
    --network=host \
    --no-cache \
    -t gabrinovas/drims2:v1 \
    .

# # Alternative: Build using cache (commented out)
# echo "🚀 Building..."
# docker build \
#     --network=host \
#     -t my_drims2:local \
#     .

docker push gabrinovas/drims2:v1

echo "✅ Build successful!"