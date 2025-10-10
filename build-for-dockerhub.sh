#!/bin/bash

# Optimized script to build the MuJoCo Docker image for Docker Hub
# Includes advanced caching and build optimizations for maximum speed

set -e

# Configuration
IMAGE_NAME="yguel/mujoco-desktop"
VERSION="v1.0"
DOCKERFILE="Dockerfile.novnc"
CACHE_DIR="/tmp/.buildx-cache-mujoco"

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

# Build the image with optimizations
echo "🔨 Building Docker image with maximum optimization..."

# Check Docker buildx availability
if docker buildx version >/dev/null 2>&1; then
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
    docker buildx build \
        --file "$DOCKERFILE" \
        --tag "$IMAGE_NAME:$VERSION" \
        --platform linux/amd64 \
        --cache-from type=local,src="$CACHE_DIR" \
        --cache-to type=local,dest="$CACHE_DIR-new",mode=max \
        --build-arg BUILDKIT_INLINE_CACHE=1 \
        --progress=auto \
        --load \
        .
    
    # Rotate cache for next build (atomic operation)
    if [ -d "$CACHE_DIR-new" ]; then
        echo "🔄 Updating build cache..."
        rm -rf "$CACHE_DIR-old" 2>/dev/null || true
        [ -d "$CACHE_DIR" ] && mv "$CACHE_DIR" "$CACHE_DIR-old"
        mv "$CACHE_DIR-new" "$CACHE_DIR"
        rm -rf "$CACHE_DIR-old" 2>/dev/null || true
    fi
    
    BUILD_METHOD="Docker Buildx (optimized)"
else
    echo "⚠️  Docker Buildx not available - using standard build with basic caching"
    
    # Standard docker build with basic optimizations
    docker build \
        --file "$DOCKERFILE" \
        --tag "$IMAGE_NAME:$VERSION" \
        --build-arg BUILDKIT_INLINE_CACHE=1 \
        .
    
    BUILD_METHOD="Standard Docker build"
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
echo "�📤 Next steps:"
echo "  1. Test the image locally"
echo "  2. Run ./push-to-dockerhub.sh to push to Docker Hub"
echo "  3. Subsequent builds will be much faster due to caching"