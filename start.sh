#!/bin/bash

# Student-friendly MuJoCo environment starter with smart OpenGL and Jupyter
# Runs in foreground with Ctrl-C support for easy terminal control

set -e

# Configuration
IMAGE_NAME="yguel/mujoco-desktop:v1.0"
CONTAINER_NAME="mujoco-student"

echo "🎓 Starting MuJoCo Student Environment"
echo "====================================="
echo "Image: $IMAGE_NAME"
echo "Container: $CONTAINER_NAME"
echo ""

# Function to detect GPU support
detect_gpu_support() {
    # Check for NVIDIA GPU support
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
        echo "✅ NVIDIA GPU detected"
        return 0
    fi
    
    # Check for Intel/AMD GPU in /dev/dri
    if [ -d "/dev/dri" ] && [ "$(ls -A /dev/dri 2>/dev/null)" ]; then
        echo "✅ GPU devices found in /dev/dri"
        return 0
    fi
    
    echo "⚠️  No GPU detected - will use software rendering"
    return 1
}

# Function to build Docker run command with smart OpenGL
build_docker_command() {
    local cmd="docker run --rm --name $CONTAINER_NAME"
    
    # Basic ports and volumes
    cmd="$cmd -p 6080:6080"        # noVNC desktop
    cmd="$cmd -p 8888:8888"        # Jupyter notebook
    cmd="$cmd -v $(pwd)/workspace:/home/student/workspace"
    
    # Pass host user UID and GID for proper file permissions
    cmd="$cmd -e HOST_UID=$(id -u)"
    cmd="$cmd -e HOST_GID=$(id -g)"
    
    # GPU support if available (check silently)
    if [ -d "/dev/dri" ] && [ "$(ls -A /dev/dri 2>/dev/null)" ] || (command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1); then
        # GPU detected - configure hardware acceleration
        
        # Add GPU devices
        if [ -d "/dev/dri" ]; then
            cmd="$cmd --device=/dev/dri"
        fi
        
        # NVIDIA runtime if available
        if command -v nvidia-smi >/dev/null 2>&1; then
            cmd="$cmd --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all"
        fi
        
        # Environment for hardware acceleration
        cmd="$cmd -e DISPLAY=:1"
        cmd="$cmd -e LIBGL_ALWAYS_SOFTWARE=0"
    else
        # No GPU - configure software rendering
        cmd="$cmd -e DISPLAY=:1"
        cmd="$cmd -e LIBGL_ALWAYS_SOFTWARE=1"
        cmd="$cmd -e MUJOCO_GL=osmesa"
    fi
    
    # Add the image
    cmd="$cmd $IMAGE_NAME"
    
    echo "$cmd"
}

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down MuJoCo environment..."
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
    echo "✅ Container stopped"
    exit 0
}

# Set trap for Ctrl-C
trap cleanup SIGINT SIGTERM

# Create workspace structure
echo "📁 Setting up workspace..."
mkdir -p workspace/{notebooks,examples,models}

# Stop any existing container
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "🛑 Stopping existing container..."
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
fi

# Check for image availability (prioritize local images)
echo "📥 Checking image availability..."
if docker images --format "table {{.Repository}}:{{.Tag}}" | grep -q "^$IMAGE_NAME$"; then
    echo "✅ Using local image: $IMAGE_NAME"
    echo "   (Built locally - will NOT pull from Docker Hub)"
else
    echo "📦 Local image not found, pulling from Docker Hub..."
    if docker pull "$IMAGE_NAME"; then
        echo "✅ Successfully pulled from Docker Hub"
    else
        echo "❌ Failed to pull image from Docker Hub"
        echo "   Please check:"
        echo "   1. Internet connection"
        echo "   2. Image name: $IMAGE_NAME"
        echo "   3. Docker Hub accessibility"
        exit 1
    fi
fi

# Show GPU detection results
echo ""
echo "🎮 Detecting graphics capabilities..."
if detect_gpu_support; then
    echo "   → Will use hardware-accelerated OpenGL"
else
    echo "   → Will use software rendering (OSMesa)"
fi

# Build and display the command
DOCKER_CMD=$(build_docker_command)
echo ""
echo "🚀 Starting container in foreground mode..."
echo "💡 Control tips:"
echo "   🖱️  Press Ctrl-C to stop the environment"
echo "   📊 Container logs will appear below"
echo "   🌐 Access will be available at:"
echo "      Desktop: http://localhost:6080"
echo "      Jupyter: http://localhost:8888"
echo ""
echo "⏳ Starting services (this may take 30 seconds)..."
echo "=============================================="

# Run the container in foreground
eval $DOCKER_CMD