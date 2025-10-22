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
        
        # Fix ownership of student's home directory after UID change
        chown -R $HOST_UID:$HOST_GID /home/student 2>/dev/null || true
        
        # Set empty password for student user to allow su without password
        passwd -d student 2>/dev/null || true
        
        echo "   ✅ Student user permissions updated"
        echo "   📁 Workspace files will now have correct permissions"
    else
        echo "   ✅ Student user already has correct UID:GID"
    fi
    
    # Set empty password for student user (in case it wasn't set above)
    passwd -d student 2>/dev/null || true
    
    # Always switch to student user for running services (only if we're not already student)
    if [ "$(whoami)" = "root" ]; then
        echo "🔄 Switching to student user for service execution..."
        export HOST_WORKSPACE_INFO
        export USE_TEMP_WORKSPACE
        export BACKUP_TARGET_DIR
        exec runuser -l student -c "cd '$PWD' && HOST_UID=$HOST_UID HOST_GID=$HOST_GID VNC_RESOLUTION=$VNC_RESOLUTION VNC_DEPTH=$VNC_DEPTH VNC_DPI=$VNC_DPI NOVNC_PORT=$NOVNC_PORT VNC_PORT=$VNC_PORT HOST_WORKSPACE_INFO=\"$HOST_WORKSPACE_INFO\" USE_TEMP_WORKSPACE=\"$USE_TEMP_WORKSPACE\" BACKUP_TARGET_DIR=\"$BACKUP_TARGET_DIR\" bash '$0'"
    else
        echo "🔄 Already running as student user, continuing with service startup..."
    fi
else
    echo "⚠️  No HOST_UID/HOST_GID provided, using default student user permissions"
fi

# If we reach here, we're running as student user

# Set proper HOME environment  
export HOME=/home/student
export USER=student

echo "🎓 Starting MuJoCo Desktop Environment as student user (UID: ${HOST_UID:-$(id -u)})"

# Configure OpenGL for MuJoCo (hardware acceleration with software fallback)
echo "🎮 Configuring OpenGL for optimal MuJoCo performance..."
source /setup-opengl.sh
configure_opengl

# Initialize dbus (fix dbus issues like in your reference)
# Clean up any existing dbus session
sudo rm -rf /tmp/dbus-* /var/run/dbus/pid 2>/dev/null || true
sudo service dbus restart 2>/dev/null || true

# Wait a bit for D-Bus to fully restart after user changes
sleep 2

# Skip D-Bus initialization here - it will be handled in VNC startup script
# Initialize D-Bus session as the (potentially updated) student user
# sudo -u student bash -c 'export $(dbus-launch)' 2>/dev/null || export $(dbus-launch)

# Remove any existing X11 lock files
rm -f /tmp/.X*-lock /tmp/.X11-unix/X*

# VNC setup will be handled in start_vnc function after user permissions are set

# Function to start VNC server
start_vnc() {
    echo "Starting VNC server on display :1..."
    
    # Create VNC directory and script
    mkdir -p "$HOME/.vnc"
    
    cat > "$HOME/.vnc/xstartup" << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

# Start dbus session
eval $(dbus-launch --sh-syntax)

# Configure OpenGL for desktop environment
export DISPLAY=:1
export MUJOCO_GL=glfw
export LIBGL_ALWAYS_SOFTWARE=0
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export __GLX_VENDOR_LIBRARY_NAME=mesa

# Configure MATE to avoid panel issues
export MATE_PANEL_DISABLE_BRISK=1
export MATE_DESKTOP_SESSION_ID=this-is-deprecated

# Start MATE desktop environment
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

# Start applications and desktop setup
update-desktop-database /usr/share/applications/ 2>/dev/null || true
update-desktop-database /home/student/.local/share/applications/ 2>/dev/null || true
mkdir -p /home/student/.local/share/applications/
ln -sf /usr/share/applications/vscode.desktop /home/student/.local/share/applications/ 2>/dev/null || true
ln -sf /usr/share/applications/chromium.desktop /home/student/.local/share/applications/ 2>/dev/null || true

# Start desktop apps
mate-panel --reset &
caja &
mate-terminal &

# Start Chromium with Docker-compatible flags and open both required URLs
chromium-browser \
    --no-sandbox \
    --disable-dev-shm-usage \
    --disable-gpu \
    --disable-software-rasterizer \
    --disable-background-timer-throttling \
    --disable-backgrounding-occluded-windows \
    --disable-renderer-backgrounding \
    --disable-features=TranslateUI \
    --disable-ipc-flooding-protection \
    --no-first-run \
    --no-default-browser-check \
    --password-store=basic \
    --use-mock-keychain \
    https://yguel.github.io/apprentissage_par_renforcement_et_simulation/ \
    https://mujoco.readthedocs.io/ &

# Keep the session alive
while true; do
    sleep 1
done
EOF

    chmod +x "$HOME/.vnc/xstartup"
    
    # Start VNC server
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
    mkdir -p /tmp/jupyter/runtime
    mkdir -p /tmp/jupyter/config
    mkdir -p "$HOME/workspace"
    
    # Set Jupyter environment variables
    export JUPYTER_CONFIG_DIR=/tmp/jupyter/config
    export JUPYTER_DATA_DIR=/tmp/jupyter
    export JUPYTER_RUNTIME_DIR=/tmp/jupyter/runtime
    cd "$HOME/workspace"
    
    # Start Jupyter notebook
    jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser \
        --ServerApp.token='' --ServerApp.password='' \
        --ServerApp.allow_origin='*' \
        --ServerApp.base_url='/' \
        --notebook-dir="$HOME/workspace" \
        > "$HOME/jupyter.log" 2>&1 &
    echo $! > "$HOME/jupyter.pid"
    
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
echo ""
echo "Workspace folder:"
if [ -n "$HOST_WORKSPACE_INFO" ] && [ "$HOST_WORKSPACE_INFO" != "\"\"" ]; then
    # Remove any leading/trailing quotes and whitespace
    CLEANED_HOST_WORKSPACE_INFO=$(echo "$HOST_WORKSPACE_INFO" | sed -e 's/^"//' -e 's/"$//' | xargs)
    if [ -z "$CLEANED_HOST_WORKSPACE_INFO" ]; then
        echo "  Host: Local workspace (debug: HOST_WORKSPACE_INFO is empty after cleaning)"
    else
        echo "  Host: $CLEANED_HOST_WORKSPACE_INFO"
    fi
else
    echo "  Host: Local workspace (debug: HOST_WORKSPACE_INFO not set or empty)"
fi
echo "  Container: /home/student/workspace"
if [ "$USE_TEMP_WORKSPACE" = "true" ]; then
    if [ -n "$BACKUP_TARGET_DIR" ]; then
        echo "  ⚠️  Using temporary workspace - will be backed up on exit to: $BACKUP_TARGET_DIR"
    else
        echo "  ⚠️  Using temporary workspace - will be backed up on exit (backup location unknown)"
    fi
fi
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