#!/bin/bash

# Script to push MuJoCo Docker image to Docker Hub

set -e

# Configuration
IMAGE_NAME="yguel/mujoco-desktop"
VERSION="v1.0"

echo "📤 Pushing MuJoCo Docker Image to Docker Hub"
echo "============================================"
echo "Image: $IMAGE_NAME"
echo "Version: $VERSION"
echo ""

# Check if images exist locally
if ! docker images | grep -q "$IMAGE_NAME"; then
    echo "❌ Error: Image $IMAGE_NAME not found locally!"
    echo "   Please run ./build-for-dockerhub.sh first"
    exit 1
fi

# Check if user is logged in to Docker Hub
echo "🔐 Checking Docker Hub authentication..."
if ! docker info | grep -q "Username:"; then
    echo "⚠️  Not logged in to Docker Hub"
    echo "   Please login first: docker login"
    read -p "   Do you want to login now? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker login
    else
        echo "❌ Cannot push without Docker Hub authentication"
        exit 1
    fi
else
    echo "✅ Authenticated to Docker Hub"
fi

# Show image details before pushing
echo ""
echo "📦 Image to push:"
docker images | grep "$IMAGE_NAME" | head -2

echo ""
echo "📤 Pushing images to Docker Hub..."


# Push version tag only
echo "  🏷️  Pushing $IMAGE_NAME:$VERSION..."
docker push $IMAGE_NAME:$VERSION

echo ""
echo "✅ Successfully pushed to Docker Hub!"
echo ""
echo "🌐 Your image is now available at:"
echo "   https://hub.docker.com/r/${IMAGE_NAME#*/}"
echo ""
echo "👥 Others can now use your image with:"
echo "   docker pull $IMAGE_NAME:$VERSION"
echo ""
echo "🚀 Quick start for users:"
echo "   docker run -d -p 6080:6080 -p 8888:8888 --name mujoco-env $IMAGE_NAME:$VERSION"
echo ""
echo "📝 Consider creating a README.md with usage instructions!"

# Optional: Show image info from Docker Hub
echo ""
read -p "🔍 Do you want to verify the push by checking Docker Hub? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🌐 Opening Docker Hub page..."
    xdg-open "https://hub.docker.com/r/${IMAGE_NAME#*/}" 2>/dev/null || echo "Please visit: https://hub.docker.com/r/${IMAGE_NAME#*/}"
fi