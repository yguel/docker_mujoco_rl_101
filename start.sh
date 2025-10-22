#!/bin/bash

# Student-friendly MuJoCo environment starter with smart OpenGL and Jupyter
# Runs in foreground with Ctrl-C support for easy terminal control

set -e

# Parse command line arguments
USE_LOCAL_ONLY=false
SMALL_RAM_MODE=false
CUSTOM_RAM_VALUE=""
VNC_RESOLUTION="1920x1080"
VNC_QUALITY="high"

while [[ $# -gt 0 ]]; do
    case $1 in
        --local)
            USE_LOCAL_ONLY=true
            shift
            ;;
        --small_ram)
            SMALL_RAM_MODE=true
            shift
            ;;
        --ram)
            if [[ $# -gt 1 && ! "$2" =~ ^-- ]]; then
                CUSTOM_RAM_VALUE="$2"
                shift 2
            else
                echo "Error: --ram requires a value (e.g., --ram 1g)"
                exit 1
            fi
            ;;
        --resolution)
            if [[ $# -gt 1 && ! "$2" =~ ^-- ]]; then
                VNC_RESOLUTION="$2"
                shift 2
            else
                echo "Error: --resolution requires a value (e.g., --resolution 1920x1080)"
                exit 1
            fi
            ;;
        --quality)
            if [[ $# -gt 1 && ! "$2" =~ ^-- ]]; then
                VNC_QUALITY="$2"
                shift 2
            else
                echo "Error: --quality requires a value (high, medium, low)"
                exit 1
            fi
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --local            Use local Docker image only, skip remote version check"
            echo "  --small_ram        Use conservative memory settings: min(2GB, 50% RAM)"
            echo "  --ram SIZE         Use specific memory amount (e.g., --ram 1g, --ram 512m)"
            echo "  --resolution WxH   Set VNC resolution (default: 1920x1080)"
            echo "  --quality LEVEL    Set VNC quality: high, medium, low (default: high)"
            echo "  -h, --help         Show this help message"
            echo ""
            echo "Memory allocation:"
            echo "  Default: min(4GB, 50% RAM)"
            echo "  --small_ram: min(2GB, 50% RAM)"
            echo "  --ram SIZE: exact value specified"
            echo ""
            echo "VNC Display:"
            echo "  Default: 1920x1080 @ 24-bit color depth, 96 DPI"
            echo "  High quality: 24-bit color, standard DPI"
            echo "  Medium quality: 16-bit color, standard DPI"
            echo "  Low quality: 8-bit color, low DPI"
            echo ""
            echo "Workspace management:"
            echo "  Persistent: \$HOME/rl/mujoco (if writable)"
            echo "  Temporary: /tmp/rl/mujoco (auto-backup on exit)"
            echo ""
            echo "Examples:"
            echo "  $0                              # Normal mode: Full HD, high quality"
            echo "  $0 --resolution 1440x900       # Custom resolution"
            echo "  $0 --quality medium --small_ram # Medium quality, low memory"
            echo "  $0 --resolution 2560x1440      # 1440p resolution"
            echo "  $0 --ram 512m --quality low    # Low resource usage"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Configuration
IMAGE_NAME="yguel/mujoco-desktop:v1.0"
CONTAINER_NAME="mujoco-student"

echo "🎓 Starting MuJoCo Student Environment"
echo "====================================="
echo "Image: $IMAGE_NAME"
echo "Container: $CONTAINER_NAME"
if [ "$USE_LOCAL_ONLY" = true ]; then
    echo "Mode: LOCAL ONLY (skipping remote checks)"
else
    echo "Mode: SMART UPDATE (checking remote)"
fi
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

# Function to install backup script from GitHub
install_backup_script() {
    local script_dir="$HOME/.local/bin/rl"
    local script_path="$script_dir/save_rl_env.sh"
    local github_url="https://raw.githubusercontent.com/yguel/docker_mujoco_rl_101/main/save_rl_env.sh"
    # Human-readable logs go to stderr; function prints ONLY the installed path on stdout
    >&2 echo "🔧 Installing backup script from GitHub..."

    # Create directory
    mkdir -p "$script_dir"

    # Download latest version from GitHub
    if command -v curl >/dev/null 2>&1; then
        if curl -fsSL "$github_url" -o "$script_path" >/dev/null 2>&1; then
            chmod +x "$script_path"
            >&2 echo "✅ Backup script installed: $script_path"
            printf '%s\n' "$script_path"
            return 0
        else
            >&2 echo "❌ Failed to download backup script with curl"
        fi
    elif command -v wget >/dev/null 2>&1; then
        if wget -q "$github_url" -O "$script_path" >/dev/null 2>&1; then
            chmod +x "$script_path"
            >&2 echo "✅ Backup script installed: $script_path"
            printf '%s\n' "$script_path"
            return 0
        else
            >&2 echo "❌ Failed to download backup script with wget"
        fi
    else
        >&2 echo "❌ Neither curl nor wget available for downloading backup script"
    fi

    return 1
}

# Function to detect volume binding capability and determine workspace path
detect_workspace_path() {
    local home_path="$HOME/rl/mujoco"
    local tmp_path="/tmp/rl/mujoco"
    
    # Create both directories first
    mkdir -p "$home_path/workspace"/{examples,models,notebooks} 2>/dev/null || true
    mkdir -p "$tmp_path/workspace"/{examples,models,notebooks}
    
    # Test actual Docker volume binding capability
    >&2 echo "🧪 Testing Docker volume binding capability..."

    # Create a test file in home path
    local test_file="$home_path/.docker_bind_test"
    if ! echo "test" > "$test_file" 2>/dev/null; then
        >&2 echo "⚠️  Cannot write to $home_path, using temporary workspace: $tmp_path"
        printf '%s|TEMP\n' "$tmp_path"
        return 0
    fi

    # Try to bind mount and check if Docker daemon can access it
    if timeout 10 docker run --rm -v "$home_path:/.test_mount" alpine:latest cat /.test_mount/.docker_bind_test >/dev/null 2>&1; then
        rm -f "$test_file" 2>/dev/null || true
        >&2 echo "✅ Docker can bind to home directory: $home_path"
        printf '%s|PERSIST\n' "$home_path"
        return 0
    else
        rm -f "$test_file" 2>/dev/null || true
        >&2 echo "⚠️  Docker daemon cannot bind to $home_path, using temporary workspace: $tmp_path"
        printf '%s|TEMP\n' "$tmp_path"
        return 0
    fi
}

# Detect workspace path and binding capability
echo ""
echo "📁 Detecting workspace configuration..."
WORKSPACE_DETECT_RESULT=$(detect_workspace_path)
WORKSPACE_PATH="${WORKSPACE_DETECT_RESULT%%|*}"
WORKSPACE_MODE="${WORKSPACE_DETECT_RESULT##*|}"

# Define backup target directory
BACKUP_TARGET_DIR="$HOME/rl/docker_snapshots/mujoco_snapshots"

if [ "$WORKSPACE_MODE" = "TEMP" ]; then
    USE_TEMP_WORKSPACE="true"
    HOST_WORKSPACE_INFO="Temporary: /tmp/rl/mujoco (will be backed up on exit)"
    BACKUP_SCRIPT_PATH=$(install_backup_script)
    if [ $? -eq 0 ]; then
        echo "🔧 Backup script ready at: $BACKUP_SCRIPT_PATH"
        # Ensure backup target directory exists and is writable
        if [ ! -d "$BACKUP_TARGET_DIR" ]; then
            echo "ℹ️  Backup directory $BACKUP_TARGET_DIR does not exist. Creating it..."
            if ! mkdir -p "$BACKUP_TARGET_DIR"; then
                echo "❌ Failed to create backup directory $BACKUP_TARGET_DIR."
                exit 1
            fi
            chmod 700 "$BACKUP_TARGET_DIR" 2>/dev/null || true
        fi
        if ! touch "$BACKUP_TARGET_DIR/.backup_test" 2>/dev/null; then
            echo "❌ Backup directory $BACKUP_TARGET_DIR is not writable. Please fix permissions."
            exit 1
        fi
        rm -f "$BACKUP_TARGET_DIR/.backup_test"
    else
        echo "⚠️  Warning: Could not install backup script from GitHub"
        echo "   Backup on exit will be skipped"
        BACKUP_SCRIPT_PATH=""
    fi
else
    USE_TEMP_WORKSPACE="false"
    HOST_WORKSPACE_INFO="Persistent: $WORKSPACE_PATH"
    BACKUP_SCRIPT_PATH=""
fi

# Create workspace structure
echo "📁 Setting up workspace structure..."
mkdir -p "$WORKSPACE_PATH/workspace"/{notebooks,examples,models}

# Function to calculate memory settings
calculate_memory_settings() {
    # If --ram has a value, use it directly
    if [ -n "$CUSTOM_RAM_VALUE" ]; then
        >&2 echo "🧠 Custom RAM: ${CUSTOM_RAM_VALUE} (user specified)"
        printf '%s\n' "$CUSTOM_RAM_VALUE"
        return
    fi
    
    # Get total RAM in GB (from /proc/meminfo)
    local total_ram_kb=$(grep MemTotal /proc/meminfo | awk '{print $2}')
    local total_ram_gb=$((total_ram_kb / 1024 / 1024))
    
    local shm_size_gb
    
    if [ "$SMALL_RAM_MODE" = true ]; then
        # Small RAM mode: max(2GB, 50% RAM)
        local fifty_percent=$((total_ram_gb / 2))
        if [ $fifty_percent -gt 2 ]; then
            shm_size_gb=2
        else
            shm_size_gb=$fifty_percent
        fi
        >&2 echo "🧠 Small RAM mode: ${shm_size_gb}GB (min of 2GB or 50% of ${total_ram_gb}GB)"
    else
        # Normal mode: min(4GB, 50% RAM)
        local fifty_percent=$((total_ram_gb / 2))
        if [ $fifty_percent -gt 4 ]; then
            shm_size_gb=4
        else
            shm_size_gb=$fifty_percent
        fi
        >&2 echo "🧠 Memory: ${shm_size_gb}GB (min of 4GB or 50% of ${total_ram_gb}GB RAM)"
    fi
    
    printf '%sg\n' "$shm_size_gb"
}

# Function to build Docker run command with smart OpenGL
build_docker_command() {
    local gpu_support=$1
    local workspace_path=$2
    
    # Calculate memory settings
    local shm_size=$(calculate_memory_settings)
    
    # Base command with memory settings
    cmd="docker run -it --rm --name $CONTAINER_NAME --shm-size=${shm_size}"
    
    # Basic ports and volumes
    cmd="$cmd -p 6080:6080"        # noVNC desktop
    cmd="$cmd -p 8888:8888"        # Jupyter notebook
    cmd="$cmd -v ${workspace_path}/workspace:/home/student/workspace"
    
    # Pass host user UID and GID for proper file permissions
    cmd="$cmd -e HOST_UID=$(id -u)"
    cmd="$cmd -e HOST_GID=$(id -g)"
    
    # Pass workspace information to entrypoint
    cmd="$cmd -e HOST_WORKSPACE_INFO=\"$HOST_WORKSPACE_INFO\""
    cmd="$cmd -e USE_TEMP_WORKSPACE=\"$USE_TEMP_WORKSPACE\""
    cmd="$cmd -e BACKUP_TARGET_DIR=\"$BACKUP_TARGET_DIR\""
    
    # VNC settings for better resolution and quality
    case $VNC_QUALITY in
        high)
            VNC_DEPTH=24
            VNC_DPI=96
            ;;
        medium)
            VNC_DEPTH=16
            VNC_DPI=96
            ;;
        low)
            VNC_DEPTH=8
            VNC_DPI=72
            ;;
        *)
            echo "⚠️  Unknown quality '$VNC_QUALITY', using high quality"
            VNC_DEPTH=24
            VNC_DPI=96
            ;;
    esac
    
    cmd="$cmd -e VNC_RESOLUTION=$VNC_RESOLUTION"   # User-specified or default resolution
    cmd="$cmd -e VNC_DEPTH=$VNC_DEPTH"             # Quality-based color depth
    cmd="$cmd -e VNC_DPI=$VNC_DPI"                 # Quality-based DPI
    cmd="$cmd -e NOVNC_PORT=6080"                  # noVNC port
    cmd="$cmd -e VNC_PORT=5901"                    # VNC direct port
    
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

# Create workspace structure (WORKSPACE_PATH already detected earlier)
echo "📁 Setting up workspace structure..."
mkdir -p "$WORKSPACE_PATH/workspace"/{notebooks,examples,models}

# Stop any existing container
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "🛑 Stopping existing container..."
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
fi

# Check for image availability and version comparison
echo "📥 Checking image availability..."
if docker images --format "table {{.Repository}}:{{.Tag}}" | grep -q "^$IMAGE_NAME$"; then
    echo "✅ Local image found: $IMAGE_NAME"
    
    if [ "$USE_LOCAL_ONLY" = true ]; then
        echo "🏠 LOCAL MODE: Using local image without remote checks"
        echo "   (Remote version checking skipped)"
    else
        # Get local image digest/ID
        LOCAL_DIGEST=$(docker images --no-trunc --quiet "$IMAGE_NAME" 2>/dev/null)
        
        # Try to get remote image digest (without pulling)
        echo "🔍 Comparing with remote image..."
        if REMOTE_DIGEST=$(docker manifest inspect "$IMAGE_NAME" 2>/dev/null | grep -o '"digest":"[^"]*"' | head -1 | cut -d'"' -f4); then
            # Get the actual local manifest digest
            LOCAL_MANIFEST_DIGEST=$(docker image inspect "$IMAGE_NAME" --format='{{index .RepoDigests 0}}' 2>/dev/null | cut -d'@' -f2)
            
            if [ "$LOCAL_MANIFEST_DIGEST" = "$REMOTE_DIGEST" ]; then
                echo "✅ Local image is up-to-date with remote"
                echo "   (Will NOT pull from Docker Hub)"
            else
                echo "📦 Local image differs from remote, pulling latest version..."
                if docker pull "$IMAGE_NAME"; then
                    echo "✅ Successfully updated image from Docker Hub"
                else
                    echo "⚠️  Failed to pull updated image, using local version"
                fi
            fi
        else
            echo "⚠️  Could not check remote image (offline or registry unavailable)"
            echo "   Using local image: $IMAGE_NAME"
        fi
    fi
else
    if [ "$USE_LOCAL_ONLY" = true ]; then
        echo "❌ LOCAL MODE: No local image found and --local flag prevents pulling"
        echo "   Please build the image locally first or run without --local flag"
        exit 1
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
fi

# Show GPU detection results
echo ""
echo "🎮 Detecting graphics capabilities..."
if detect_gpu_support; then
    echo "   → Will use hardware-accelerated OpenGL"
else
    echo "   → Will use software rendering (OSMesa)"
fi

# Show VNC configuration
echo ""
echo "🖥️  VNC Display Configuration:"
echo "   Resolution: $VNC_RESOLUTION"
echo "   Quality: $VNC_QUALITY"
case $VNC_QUALITY in
    high)   echo "   → 24-bit color, 96 DPI (best quality)" ;;
    medium) echo "   → 16-bit color, 96 DPI (balanced)" ;;
    low)    echo "   → 8-bit color, 72 DPI (performance)" ;;
esac

# Build and display the command
DOCKER_CMD=$(build_docker_command true "$WORKSPACE_PATH")
echo $DOCKER_CMD

echo ""
echo "🚀 Starting container in foreground mode..."
echo "💡 Control tips:"
echo "   🖱️  Press Ctrl-C to stop the environment"
echo "   📊 Container logs will appear below"
echo "   🌐 Access will be available at:"
echo "      Desktop: http://localhost:6080"
echo "      Jupyter: http://localhost:8888"
echo "   📁 Workspace folder:"
echo "      Host: $HOST_WORKSPACE_INFO"
echo "      Container: /home/student/workspace"
if [ "$USE_TEMP_WORKSPACE" = "true" ]; then
    echo "      ⚠️  Using temporary workspace - will be backed up on exit to: $BACKUP_TARGET_DIR"
fi
echo ""
echo "⏳ Starting services (this may take 30 seconds)..."
echo "=============================================="

# Function for cleanup on exit
cleanup_on_exit() {
    echo ""
    echo "🛑 Stopping container..."
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
    
    # If using temp workspace, create backup
    if [ "$USE_TEMP_WORKSPACE" = "true" ] && [ -n "$BACKUP_SCRIPT_PATH" ] && [ -f "$BACKUP_SCRIPT_PATH" ]; then
        echo "💾 Creating backup of temporary workspace..."
        "$BACKUP_SCRIPT_PATH" "$BACKUP_TARGET_DIR"
    fi
    
    echo "✅ Environment stopped"
}

# Set trap for cleanup
trap cleanup_on_exit SIGINT SIGTERM EXIT

# Run the container in foreground
eval $DOCKER_CMD