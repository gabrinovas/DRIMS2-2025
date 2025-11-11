#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Parse arguments
CLEAN_APP=false
REBUILD_BASE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --clean-app)
            CLEAN_APP=true
            shift
            ;;
        --rebuild-base)
            REBUILD_BASE=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Prune Docker images and containers if rebuild-base is requested
if [ "$REBUILD_BASE" = true ]; then
    echo "🧹 Pruning Docker system for base rebuild..."
    
    # Stop and remove drims2 container if it exists
    if docker ps -a | grep -q drims2; then
        echo "Stopping and removing drims2 container..."
        docker rm -f drims2 2>/dev/null || true
    fi
    
    # Remove all application images (including base)
    echo "Removing all drims2 images..."
    docker images | grep "gabrinovas/drims2" | awk '{print $3}' | xargs -r docker rmi -f 2>/dev/null || true
    
    # More aggressive cleanup
    echo "Removing all dangling images..."
    docker image prune -f
    
    echo "Removing all unused containers..."
    docker container prune -f
    
    echo "Removing all unused images..."
    docker image prune -a -f
    
    echo "Removing build cache..."
    docker builder prune -f
fi

# Clean application images if requested (only if not already done by rebuild-base)
if [ "$CLEAN_APP" = true ] && [ "$REBUILD_BASE" = false ]; then
    echo "🧹 Cleaning application images..."
    
    # Stop and remove container if it exists
    if docker ps -a | grep -q drims2; then
        echo "Stopping and removing drims2 container..."
        docker rm -f drims2 2>/dev/null || true
    fi
    
    # Remove application images (excluding base)
    echo "Removing application images..."
    docker images | grep "gabrinovas/drims2" | grep -v "gabrinovas/drims2-base" | awk '{print $3}' | xargs -r docker rmi -f 2>/dev/null || true
    
    # Clean up dangling images
    echo "Cleaning up dangling images..."
    docker image prune -f
fi

# Build base image only if it doesn't exist or rebuild requested
if [ "$REBUILD_BASE" = true ] || ! docker image inspect gabrinovas/drims2-base:latest >/dev/null 2>&1; then
    echo "🚀 Building base image with no cache..."
    docker build \
        --network=host \
        --no-cache \
        -t gabrinovas/drims2-base:latest \
        -f Dockerfile.base .
else
    echo "✅ Base image exists, skipping build..."
fi

# Build main application image
echo "🚀 Building main application image..."
docker build \
    --network=host \
    --no-cache \
    -t gabrinovas/drims2:driver_trial \
    -f Dockerfile .

echo "✅ Build successful!"

echo "🚚 Pushing images to Docker Hub..."
# docker push gabrinovas/drims2-base:latest
# docker push gabrinovas/drims2:driver_trial
echo "✅ Push successful!"

# Usage examples:
# ./build.sh                    # Normal build, reuse base if exists
# ./build.sh --clean-app        # Clean app only, reuse base
# ./build.sh --rebuild-base     # Rebuild both base and app (with full cleanup)