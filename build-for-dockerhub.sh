#!/bin/bash

# Optimized script to build the MuJoCo Docker image for Docker Hub
# Includes advanced caching, build optimizations, and retry logic

set -e

# Configuration
IMAGE_NAME="yguel/mujoco-desktop"
VERSION="v1.0"
DOCKERFILE="Dockerfile.novnc"
CACHE_DIR="/tmp/.buildx-cache-mujoco"
MAX_RETRIES=3
RETRY_DELAY=30

echo "🚀 Building MuJoCo Docker Image for Docker Hub (Optimized)"
echo "=========================================================="
echo "Image: $IMAGE_NAME:$VERSION"
echo "Dockerfile: $DOCKERFILE"
echo "Cache dir: $CACHE_DIR"
echo ""

# Function to check Docker Hub connectivity
check_docker_hub() {
    echo "🔗 Checking Docker Hub connectivity..."
    # Test with anonymous access to a minimal public image
    if timeout 15 docker pull --quiet alpine:latest >/dev/null 2>&1; then
        echo "✅ Docker Hub is accessible (anonymous)"
        docker rmi alpine:latest >/dev/null 2>&1 || true
        return 0
    else
        echo "❌ Docker Hub is not accessible"
        return 1
    fi
}

# Function to try pulling base image anonymously
try_pull_base_image() {
    local base_image="ubuntu:24.04"
    echo "🔍 Checking base image availability..."
    
    # Check if already exists locally
    if docker images --format "table {{.Repository}}:{{.Tag}}" | grep -q "^$base_image$"; then
        echo "✅ Base image $base_image already available locally"
        return 0
    fi
    
    # Try anonymous pull
    echo "📦 Attempting anonymous pull of $base_image..."
    if timeout 30 docker pull "$base_image" >/dev/null 2>&1; then
        echo "✅ Successfully pulled $base_image anonymously"
        return 0
    else
        echo "❌ Failed to pull $base_image"
        return 1
    fi
}

# Function to retry Docker Hub operations
retry_with_docker_hub() {
    local attempt=1
    while [ $attempt -le $MAX_RETRIES ]; do
        if check_docker_hub; then
            return 0
        else
            echo "⏰ Attempt $attempt/$MAX_RETRIES failed. Waiting ${RETRY_DELAY}s before retry..."
            if [ $attempt -lt $MAX_RETRIES ]; then
                sleep $RETRY_DELAY
            fi
            attempt=$((attempt + 1))
        fi
    done
    echo "❌ Docker Hub unavailable after $MAX_RETRIES attempts"
    return 1
}

echo "🚀 Building MuJoCo Docker Image for Docker Hub (Optimized)"
echo "=========================================================="
echo "Image: $IMAGE_NAME:$VERSION"
echo "Dockerfile: $DOCKERFILE"
echo "Cache dir: $CACHE_DIR"
echo ""

# Check if Dockerfile exists
if [ ! -f "$DOCKERFILE" ]; then
    echo "❌ Error: $DOCKERFILE not found!"
    exit 1
fi

# Check Docker Hub connectivity before starting
echo "🔍 Pre-build connectivity check..."

# First, try to ensure we have the base image
if try_pull_base_image; then
    echo "✅ Base image ready for build"
    OFFLINE_BUILD=false
else
    echo "⚠️  Cannot access base image from Docker Hub"
    
    # Check if base image exists locally
    BASE_IMAGE="ubuntu:24.04"
    if docker images --format "table {{.Repository}}:{{.Tag}}" | grep -q "^$BASE_IMAGE$"; then
        echo "✅ Base image $BASE_IMAGE found locally - can build offline"
        OFFLINE_BUILD=true
    else
        echo ""
        echo "❌ No base image available for building"
        echo ""
        echo "� This issue is likely due to:"
        echo "  1. Docker Hub authentication service outage (503 errors)"
        echo "  2. Network connectivity issues"
        echo "  3. Docker Hub rate limiting"
        echo ""
        echo "🔧 Solutions:"
        echo "  1. Wait 10-30 minutes and try again (most common fix)"
        echo "  2. Check Docker Hub status: https://status.docker.com/"
        echo "  3. When Docker Hub works, pre-pull: docker pull ubuntu:24.04"
        echo "  4. Use a different registry temporarily"
        echo ""
        echo "Note: Authentication is NOT required for building with public images"
        echo "The issue is with Docker Hub's service availability, not credentials"
        exit 1
    fi
fi

echo ""

# Build the image with optimizations
if [ "$OFFLINE_BUILD" = true ]; then
    echo "🔨 Building Docker image in OFFLINE mode (using local base image)..."
else
    echo "🔨 Building Docker image with maximum optimization..."
fi

# Check Docker buildx availability
if docker buildx version >/dev/null 2>&1 && [ "$OFFLINE_BUILD" != true ]; then
    echo "✅ Docker Buildx detected - using advanced caching and optimizations"
    
    # Create or use existing builder instance
    BUILDER_NAME="mujoco-builder-optimized"
    if ! docker buildx ls | grep -q "$BUILDER_NAME"; then
        echo "🔧 Creating optimized buildx builder: $BUILDER_NAME"
        docker buildx create --name "$BUILDER_NAME" --driver docker-container --use
    else
        echo "🔄 Using existing buildx builder: $BUILDER_NAME"
        docker buildx use "$BUILDER_NAME"
    fi
    
    # Advanced buildx build with maximum optimizations
    echo "🚀 Building with advanced caching and multi-layer optimization..."
    if docker buildx build \
        --file "$DOCKERFILE" \
        --tag "$IMAGE_NAME:$VERSION" \
        --platform linux/amd64 \
        --cache-from type=local,src="$CACHE_DIR" \
        --cache-to type=local,dest="$CACHE_DIR-new",mode=max \
        --build-arg BUILDKIT_INLINE_CACHE=1 \
        --progress=auto \
        --load \
        . 2>&1; then
        BUILD_SUCCESS=true
        BUILD_METHOD="Docker Buildx (optimized)"
    else
        echo "❌ Buildx build failed, falling back to standard build..."
        BUILD_SUCCESS=false
    fi
    
    # Rotate cache for next build (atomic operation)
    if [ "$BUILD_SUCCESS" = true ] && [ -d "$CACHE_DIR-new" ]; then
        echo "🔄 Updating build cache..."
        rm -rf "$CACHE_DIR-old" 2>/dev/null || true
        [ -d "$CACHE_DIR" ] && mv "$CACHE_DIR" "$CACHE_DIR-old"
        mv "$CACHE_DIR-new" "$CACHE_DIR"
        rm -rf "$CACHE_DIR-old" 2>/dev/null || true
    fi
else
    BUILD_SUCCESS=false
fi

# Fallback to standard build if buildx failed or offline mode
if [ "$BUILD_SUCCESS" != true ]; then
    if [ "$OFFLINE_BUILD" = true ]; then
        echo "🔧 Using standard build (offline mode - no registry access)"
    else
        echo "⚠️  Docker Buildx not available or failed - using standard build with basic caching"
    fi
    
    # Standard docker build with basic optimizations
    if docker build \
        --file "$DOCKERFILE" \
        --tag "$IMAGE_NAME:$VERSION" \
        --build-arg BUILDKIT_INLINE_CACHE=1 \
        . 2>&1; then
        BUILD_SUCCESS=true
        BUILD_METHOD="Standard Docker build$([ "$OFFLINE_BUILD" = true ] && echo " (offline)")"
    else
        echo "❌ Standard build also failed!"
        exit 1
    fi
fi

echo ""
echo "✅ Build completed successfully!"
echo ""
echo "� Build Summary:"
echo "  Method: $BUILD_METHOD"
echo "  Image: $IMAGE_NAME:$VERSION"
echo "  Platform: linux/amd64"
echo "  Cache: $([ -d "$CACHE_DIR" ] && echo "Available" || echo "None")"
echo ""
echo "�📦 Images created:"
docker images | grep "$IMAGE_NAME" | head -3

echo ""
echo "🔍 Image details:"
echo "  Name: $IMAGE_NAME"
echo "  Tag: $VERSION"
echo "  Size: $(docker images $IMAGE_NAME:$VERSION --format "table {{.Size}}" | tail -n 1)"
echo "  Created: $(docker images $IMAGE_NAME:$VERSION --format "table {{.CreatedSince}}" | tail -n 1)"

echo ""
echo "🧪 Quick test commands:"
echo "  Test locally: docker run -d -p 6080:6080 -p 8888:8888 $IMAGE_NAME:$VERSION"
echo "  Test desktop: http://localhost:6080"
echo "  Test jupyter: http://localhost:8888"
echo "  OpenGL test: docker run --rm $IMAGE_NAME:$VERSION /setup-opengl.sh"

echo ""
echo "⚡ Performance optimizations applied:"
echo "  ✅ Advanced layer caching"
echo "  ✅ Buildkit inline cache"
echo "  ✅ Optimized builder instance"
echo "  ✅ Atomic cache rotation"
echo ""
echo "� Cache management:"
echo "  View cache: ls -la $CACHE_DIR"
echo "  Clear cache: rm -rf $CACHE_DIR"
echo "  Cache size: $([ -d "$CACHE_DIR" ] && du -sh "$CACHE_DIR" 2>/dev/null | cut -f1 || echo "0")"

echo ""
echo "📤 Next steps:"
if [ "$OFFLINE_BUILD" = true ]; then
    echo "  ⚠️  Built in offline mode due to Docker Hub unavailability"
    echo "  1. Test the image locally first"
    echo "  2. Wait for Docker Hub to recover before pushing"
    echo "  3. Check Docker Hub status: https://status.docker.com/"
    echo "  4. Retry ./push-to-dockerhub.sh later"
    
    echo ""
    echo "🛠️  Troubleshooting Docker Hub issues:"
    echo "  • 503 errors are usually temporary (try again in 10-30 minutes)"
    echo "  • Check status: https://status.docker.com/"
    echo "  • Cache base images locally: docker pull ubuntu:24.04"
    echo "  • Use --no-cache if builds seem stuck"
else
    echo "  1. Test the image locally"
    echo "  2. Run ./push-to-dockerhub.sh to push to Docker Hub"
    echo "  3. Subsequent builds will be much faster due to caching"
fi