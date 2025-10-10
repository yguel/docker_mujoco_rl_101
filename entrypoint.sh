#!/bin/bash
set -e

# Adjust student user UID/GID to match host user for proper file permissions
if [ -n "$HOST_UID" ] && [ -n "$HOST_GID" ]; then
    echo "🔧 Adjusting student user permissions..."
    echo "   Host UID: $HOST_UID, Host GID: $HOST_GID"
    
    # Get current student UID/GID
    CURRENT_UID=$(id -u student)
    CURRENT_GID=$(id -g student)
    
    if [ "$CURRENT_UID" != "$HOST_UID" ] || [ "$CURRENT_GID" != "$HOST_GID" ]; then
        echo "   Updating student user UID:GID from $CURRENT_UID:$CURRENT_GID to $HOST_UID:$HOST_GID"
        
        # Stop any processes running as student
        pkill -u student 2>/dev/null || true
        
        # Change the UID and GID
        groupmod -g $HOST_GID student 2>/dev/null || true
        usermod -u $HOST_UID -g $HOST_GID student 2>/dev/null || true
        
        echo "   ✅ Student user permissions updated"
        echo "   📁 Workspace files will now have correct permissions"
    else
        echo "   ✅ Student user already has correct UID:GID"
    fi
else
    echo "⚠️  No HOST_UID/HOST_GID provided, using default student user permissions"
fi

# Set proper HOME environment  
export HOME=/home/student
export USER=student

echo "🎓 Starting MuJoCo Desktop Environment as student user (UID: $(id -u student))"

# Configure OpenGL for MuJoCo (hardware acceleration with software fallback)
echo "🎮 Configuring OpenGL for optimal MuJoCo performance..."
source /setup-opengl.sh
configure_opengl

# Initialize dbus (fix dbus issues like in your reference)
export $(dbus-launch)

# Remove any existing X11 lock files
rm -f /tmp/.X*-lock /tmp/.X11-unix/X*

# Set up VNC (no password for easier container access)
mkdir -p "$HOME/.vnc"

# Create VNC config
cat > "$HOME/.vnc/xstartup" << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

# Start dbus session
eval $(dbus-launch --sh-syntax)

# Configure OpenGL for desktop environment
export DISPLAY=:1
export MUJOCO_GL=glfw
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export __GLX_VENDOR_LIBRARY_NAME=mesa

# Configure MATE to avoid panel issues
export MATE_PANEL_DISABLE_BRISK=1
export MATE_DESKTOP_SESSION_ID=this-is-deprecated

# Start MATE desktop environment (like tiryoh/ros2-desktop-vnc)
mate-session &

# Wait for desktop to load
sleep 8

# Disable screensaver and power management to prevent lock screen
gsettings set org.mate.screensaver lock-enabled false 2>/dev/null || true
gsettings set org.mate.screensaver idle-activation-enabled false 2>/dev/null || true
gsettings set org.mate.power-manager sleep-display-ac 0 2>/dev/null || true
gsettings set org.mate.power-manager sleep-display-battery 0 2>/dev/null || true
gsettings set org.mate.power-manager sleep-computer-ac 0 2>/dev/null || true
gsettings set org.mate.power-manager sleep-computer-battery 0 2>/dev/null || true
gsettings set org.mate.session idle-delay 0 2>/dev/null || true

# Update desktop database to recognize new applications
update-desktop-database /usr/share/applications/ 2>/dev/null || true
update-desktop-database /home/student/.local/share/applications/ 2>/dev/null || true

# Create symlinks to user applications directory for menu visibility
mkdir -p /home/student/.local/share/applications/
ln -sf /usr/share/applications/vscode.desktop /home/student/.local/share/applications/ 2>/dev/null || true
ln -sf /usr/share/applications/chromium.desktop /home/student/.local/share/applications/ 2>/dev/null || true
ln -sf /usr/share/applications/mujoco-examples.desktop /home/student/.local/share/applications/ 2>/dev/null || true
ln -sf /usr/share/applications/quick-launcher.desktop /home/student/.local/share/applications/ 2>/dev/null || true

# Reset panel configuration to avoid Brisk Menu issues
mate-panel --reset &

# Refresh menu cache
killall mate-menu 2>/dev/null || true

# Start file manager
caja &

# Start terminal
mate-terminal &

# Start Chromium with MuJoCo documentation
chromium --no-sandbox --disable-gpu --disable-software-rasterizer \
  "https://mujoco.readthedocs.io/" \
  "https://github.com/google-deepmind/mujoco" &

# Wait a bit for desktop to be ready
sleep 3

# Install VS Code extensions for Python development
echo "Installing VS Code extensions..."
code --no-sandbox --disable-gpu --user-data-dir=/home/student/.vscode \
  --install-extension ms-python.python \
  --install-extension ms-python.pylint \
  --install-extension ms-toolsai.jupyter \
  --install-extension ms-vscode.cmake-tools 2>/dev/null || true

# Start VS Code with workspace and open example
sleep 2

# Additional screensaver disable commands for extra safety
xset s off 2>/dev/null || true
xset s noblank 2>/dev/null || true
xset -dpms 2>/dev/null || true

# Set blank password for student user as fallback for unlock screen
echo "student:" | sudo chpasswd 2>/dev/null || true

code --no-sandbox --disable-gpu --user-data-dir=/home/student/.vscode \
  /home/student/workspace \
  /home/student/workspace/examples/mujoco_example.py &

# Keep the session alive
while true; do
    sleep 1
done
EOF

chmod +x "$HOME/.vnc/xstartup"

# Function to start VNC server
start_vnc() {
    echo "Starting VNC server on display :1..."
    vncserver :1 -geometry ${VNC_RESOLUTION} -depth ${VNC_DEPTH} -dpi ${VNC_DPI} \
        -localhost no -SecurityTypes None --I-KNOW-THIS-IS-INSECURE
}

# Function to start noVNC
start_novnc() {
    echo "Starting noVNC on port ${NOVNC_PORT}..."
    websockify --web=/usr/share/novnc/ ${NOVNC_PORT} localhost:${VNC_PORT} > /dev/null 2>&1 &
    NOVNC_PID=$!
    echo "noVNC started with PID: $NOVNC_PID"
}

# Function to start Jupyter notebook server
start_jupyter() {
    echo "Starting Jupyter notebook server on port 8888..."
    
    # Use /tmp for Jupyter directories to avoid permission issues
    sudo -u student bash -c "
        mkdir -p /tmp/jupyter/runtime
        mkdir -p /tmp/jupyter/config
        mkdir -p /home/student/workspace
    "
    
    # Start Jupyter as student user with temp directories for Jupyter config
    sudo -u student -H bash -c "
        export HOME=/home/student
        export JUPYTER_CONFIG_DIR=/tmp/jupyter/config
        export JUPYTER_DATA_DIR=/tmp/jupyter
        export JUPYTER_RUNTIME_DIR=/tmp/jupyter/runtime
        cd /home/student/workspace
        jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser \
            --ServerApp.token='' --ServerApp.password='' \
            --ServerApp.allow_origin='*' \
            --ServerApp.base_url='/' \
            --notebook-dir=/home/student/workspace \
            > /home/student/jupyter.log 2>&1 &
        echo \$! > /home/student/jupyter.pid
    "
    
    # Get the PID from the file
    sleep 3
    if [ -f /home/student/jupyter.pid ]; then
        JUPYTER_PID=$(cat /home/student/jupyter.pid)
        echo "Jupyter started with PID: $JUPYTER_PID"
        
        # Check if it's running
        if kill -0 $JUPYTER_PID 2>/dev/null; then
            echo "✅ Jupyter process is alive"
        else
            echo "❌ Jupyter process died, checking logs..."
            echo "--- Jupyter Error Log ---"
            cat /home/student/jupyter.log 2>/dev/null || echo "No log file found"
            echo "--- End Log ---"
        fi
    else
        echo "❌ Could not get Jupyter PID"
    fi
}

# Function to cleanup on exit
cleanup() {
    echo "Stopping services..."
    vncserver -kill :1 2>/dev/null || true
    kill $NOVNC_PID 2>/dev/null || true
    
    # Kill Jupyter using PID file if it exists
    if [ -f /home/student/jupyter.pid ]; then
        JUPYTER_PID=$(cat /home/student/jupyter.pid)
        kill $JUPYTER_PID 2>/dev/null || true
        rm -f /home/student/jupyter.pid
    fi
    
    pkill -f websockify 2>/dev/null || true
    pkill -f jupyter 2>/dev/null || true
    exit 0
}

# Set trap for cleanup
trap cleanup SIGTERM SIGINT

# Start services
start_vnc
sleep 3
start_novnc
sleep 2
start_jupyter
sleep 2

# Verify services are running
echo "Checking services status..."
if pgrep -f "Xtigervnc" > /dev/null; then
    echo "✅ VNC server is running"
else
    echo "❌ VNC server failed to start"
fi

if pgrep -f "websockify" > /dev/null; then
    echo "✅ noVNC is running"
else
    echo "❌ noVNC failed to start"
fi

if pgrep -f "jupyter" > /dev/null; then
    echo "✅ Jupyter notebook is running"
else
    echo "❌ Jupyter notebook failed to start"
fi

echo "================================"
echo "MuJoCo Desktop Environment Ready!"
echo "================================"
echo "Access via:"
echo "  Desktop: http://localhost:${NOVNC_PORT}"
echo "  Jupyter: http://localhost:8888"
echo "  VNC Client: localhost:${VNC_PORT}"
echo "  Security: No password (safe for local development)"
echo "================================"

# If arguments provided, execute them
if [ $# -gt 0 ]; then
    # If it's just 'bash', start an interactive session but keep services running
    if [ "$1" = "bash" ]; then
        echo "Starting interactive mode..."
        echo "Services running in background. Container will stay alive."
        # Keep the main process running to prevent container exit
        while true; do
            sleep 60
            # Check if services are still running
            if ! pgrep -f "Xtigervnc" > /dev/null || ! pgrep -f "websockify" > /dev/null; then
                echo "Services stopped, restarting..."
                start_vnc
                sleep 2
                start_novnc
            fi
        done
    else
        exec "$@"
    fi
else
    # Keep container running
    echo "Container running... Press Ctrl+C to stop."
    while true; do
        sleep 60
        # Health check
        if ! pgrep -f "Xtigervnc" > /dev/null || ! pgrep -f "websockify" > /dev/null; then
            echo "Services stopped, restarting..."
            start_vnc
            sleep 2
            start_novnc
        fi
    done
fi