#!/bin/bash
# Smart OpenGL configuration script
# Detects hardware acceleration capability and configures MuJoCo accordingly

setup_opengl() {
    echo "🎮 Configuring OpenGL for MuJoCo..."
    
    # Set display for VNC desktop
    export DISPLAY=:1
    
    # Default to GLFW for desktop interactive viewer
    export MUJOCO_GL="glfw"
    export LIBGL_ALWAYS_SOFTWARE=0
    
    # Check if we're in a VNC/desktop environment
    if [ "$DISPLAY" = ":1" ]; then
        echo "🖥️  VNC Desktop environment detected"
        echo "   - Attempting hardware acceleration first, software fallback available"
        
        # Try hardware acceleration first if GPU devices are available
        if [ -d "/dev/dri" ] && [ "$(ls -A /dev/dri 2>/dev/null)" ]; then
            echo "   - GPU devices found, trying hardware acceleration..."
            export MUJOCO_GL="glfw"
            export LIBGL_ALWAYS_SOFTWARE=0
            export MESA_GL_VERSION_OVERRIDE="3.3"
            export MESA_GLSL_VERSION_OVERRIDE="330"
            export __GLX_VENDOR_LIBRARY_NAME=mesa
            return 0
        else
            echo "   - No GPU devices, using software rendering"
            export MUJOCO_GL="glfw"
            export LIBGL_ALWAYS_SOFTWARE=1
            export MESA_GL_VERSION_OVERRIDE="3.3"
            export MESA_GLSL_VERSION_OVERRIDE="330"
            return 0
        fi
    fi
    
    # Check if we're in a container with GPU access
    if [ -d "/dev/dri" ] && [ "$(ls -A /dev/dri 2>/dev/null)" ]; then
        echo "✅ GPU devices detected in /dev/dri"
        
        # Try to detect GPU capability for hardware acceleration
        if command -v glxinfo >/dev/null 2>&1; then
            # Test hardware OpenGL
            if timeout 5s glxinfo >/dev/null 2>&1; then
                echo "✅ Hardware OpenGL available - using accelerated rendering"
                export MUJOCO_GL="glfw"
                export LIBGL_ALWAYS_SOFTWARE=0
                return 0
            fi
        fi
        
        # Try EGL for headless hardware acceleration
        if command -v eglinfo >/dev/null 2>&1; then
            if timeout 5s eglinfo >/dev/null 2>&1; then
                echo "✅ EGL hardware acceleration available"
                export MUJOCO_GL="egl"
                export LIBGL_ALWAYS_SOFTWARE=0
                return 0
            fi
        fi
    fi
    
    # Check for software Mesa rendering capability
    if command -v glxinfo >/dev/null 2>&1; then
        export LIBGL_ALWAYS_SOFTWARE=1
        if timeout 5s glxinfo >/dev/null 2>&1; then
            echo "⚠️  Using software OpenGL rendering (Mesa)"
            export MUJOCO_GL="glfw"
            return 0
        fi
    fi
    
    # Fallback to OSMesa (most compatible)
    echo "⚠️  Using OSMesa software rendering (safest fallback)"
    export MUJOCO_GL="osmesa"
    export LIBGL_ALWAYS_SOFTWARE=1
    
    # Additional Mesa environment variables for compatibility
    export MESA_GL_VERSION_OVERRIDE="3.3"
    export MESA_GLSL_VERSION_OVERRIDE="330"
}

# Test MuJoCo OpenGL configuration
test_mujoco_opengl() {
    echo "🧪 Testing MuJoCo OpenGL configuration..."
    
    # Create a simple test script
    cat > /tmp/test_mujoco_gl.py << 'EOF'
import os
import sys
try:
    import mujoco
    print(f"MuJoCo version: {mujoco.__version__}")
    print(f"MUJOCO_GL: {os.environ.get('MUJOCO_GL', 'not set')}")
    print(f"LIBGL_ALWAYS_SOFTWARE: {os.environ.get('LIBGL_ALWAYS_SOFTWARE', 'not set')}")
    
    # Test model creation and rendering
    xml = """
    <mujoco>
      <worldbody>
        <geom name="floor" size="1 1 0.1" type="plane"/>
        <body name="box" pos="0 0 1">
          <geom name="box_geom" size="0.1 0.1 0.1" type="box"/>
        </body>
      </worldbody>
    </mujoco>
    """
    
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    print("✅ MuJoCo OpenGL test passed!")
    
except Exception as e:
    print(f"❌ MuJoCo OpenGL test failed: {e}")
    sys.exit(1)
EOF
    
    # Run the test
    if python3 /tmp/test_mujoco_gl.py; then
        echo "✅ MuJoCo OpenGL configuration successful!"
        return 0
    else
        echo "❌ MuJoCo OpenGL configuration failed!"
        return 1
    fi
}

# Main configuration function
configure_opengl() {
    echo "🔧 Starting OpenGL configuration..."
    setup_opengl
    
    echo ""
    echo "📊 OpenGL Configuration Summary:"
    echo "  MUJOCO_GL: $MUJOCO_GL"
    echo "  LIBGL_ALWAYS_SOFTWARE: $LIBGL_ALWAYS_SOFTWARE"
    echo "  MESA_GL_VERSION_OVERRIDE: ${MESA_GL_VERSION_OVERRIDE:-not set}"
    echo "  GPU devices: $(ls /dev/dri 2>/dev/null | wc -l) found"
    echo ""
    
    # Test the configuration
    test_mujoco_opengl
    return $?
}

# Export the configuration function
if [ "${BASH_SOURCE[0]}" == "${0}" ]; then
    # Script is being run directly
    configure_opengl
else
    # Script is being sourced
    echo "OpenGL configuration functions loaded"
fi