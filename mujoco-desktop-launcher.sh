#!/bin/bash
# Desktop launcher for MuJoCo examples with proper OpenGL setup

echo "🎮 MuJoCo Desktop Launcher"
echo "========================="

# Set OpenGL environment for desktop
export DISPLAY=:1
export MUJOCO_GL=glfw
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export __GLX_VENDOR_LIBRARY_NAME=mesa

echo "✅ OpenGL configured for desktop environment"
echo "Environment:"
echo "  DISPLAY=$DISPLAY"
echo "  MUJOCO_GL=$MUJOCO_GL"
echo "  LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"
echo ""

# Test OpenGL availability
echo "🧪 Testing OpenGL..."
python3 -c "
import os
try:
    import mujoco
    print('✅ MuJoCo imported successfully')
    
    # Simple test model
    xml = '''
    <mujoco>
      <worldbody>
        <geom name=\"floor\" size=\"1 1 0.1\" type=\"plane\"/>
        <body name=\"box\" pos=\"0 0 1\">
          <geom name=\"box_geom\" size=\"0.1 0.1 0.1\" type=\"box\"/>
        </body>
      </worldbody>
    </mujoco>
    '''
    
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    print('✅ MuJoCo simulation test passed')
except Exception as e:
    print(f'❌ MuJoCo test failed: {e}')
" 2>/dev/null

echo ""
echo "🚀 Available MuJoCo examples:"
echo "1. Basic interactive viewer example"
echo "2. Advanced falling boxes simulation"
echo "3. Custom model playground"
echo ""

read -p "Choose example (1-3) or press Enter for example 1: " choice

case "${choice:-1}" in
    1)
        echo "🎯 Running basic interactive example..."
        cd /home/student/workspace/examples
        python3 mujoco_example.py
        ;;
    2)
        echo "🎯 Running advanced falling boxes..."
        cd /home/student/workspace/examples
        python3 -c "
import mujoco
import mujoco.viewer
import numpy as np
import time

# Enhanced falling boxes model
xml = '''
<mujoco>
  <option timestep=\"0.001\" gravity=\"0 0 -9.81\"/>
  <asset>
    <texture name=\"grid\" type=\"2d\" builtin=\"checker\" rgb1=\".1 .2 .3\" rgb2=\".2 .3 .4\" width=\"300\" height=\"300\"/>
    <material name=\"grid\" texture=\"grid\" texrepeat=\"8 8\" reflectance=\".2\"/>
  </asset>
  <worldbody>
    <geom name=\"floor\" size=\"3 3 0.1\" type=\"plane\" material=\"grid\"/>
    <light name=\"spotlight\" mode=\"targetbodycom\" target=\"box1\" diffuse=\".8 .8 .8\" specular=\"0.3 0.3 0.3\" pos=\"0 -1 4\" cutoff=\"30\"/>
    
    <body name=\"box1\" pos=\"0 0 3\">
      <freejoint/>
      <geom name=\"box1_geom\" size=\"0.2 0.2 0.2\" type=\"box\" rgba=\"1 0.2 0.2 1\"/>
      <inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"0.1 0.1 0.1\"/>
    </body>
    
    <body name=\"sphere\" pos=\"0.8 0.5 4\">
      <freejoint/>
      <geom name=\"sphere_geom\" size=\"0.15\" type=\"sphere\" rgba=\"0.2 1 0.2 1\"/>
      <inertial pos=\"0 0 0\" mass=\"0.5\" diaginertia=\"0.05 0.05 0.05\"/>
    </body>
  </worldbody>
</mujoco>
'''

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(model, data)
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
"
        ;;
    3)
        echo "🎯 Opening custom model playground..."
        echo "💡 You can modify the XML model and see changes in real-time!"
        cd /home/student/workspace/examples
        python3 -c "
import mujoco
import mujoco.viewer

# Simple customizable model
xml = '''
<mujoco>
  <worldbody>
    <geom name=\"floor\" size=\"2 2 0.1\" type=\"plane\" rgba=\"0.5 0.5 0.5 1\"/>
    <body name=\"pendulum\" pos=\"0 0 1\">
      <joint name=\"hinge\" type=\"hinge\" axis=\"1 0 0\"/>
      <geom name=\"pole\" fromto=\"0 0 0 0 0 -1\" size=\"0.05\" type=\"capsule\" rgba=\"0.8 0.2 0.2 1\"/>
      <body name=\"mass\" pos=\"0 0 -1\">
        <geom name=\"ball\" size=\"0.1\" type=\"sphere\" rgba=\"0.2 0.8 0.2 1\"/>
      </body>
    </body>
  </worldbody>
</mujoco>
'''

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
data.qpos[0] = 0.5  # Initial angle

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
"
        ;;
    *)
        echo "Invalid choice, running default example..."
        cd /home/student/workspace/examples
        python3 mujoco_example.py
        ;;
esac

echo ""
echo "✅ MuJoCo session completed!"