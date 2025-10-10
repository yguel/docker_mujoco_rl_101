#!/usr/bin/env python3
"""
Simple test script for MuJoCo installation
"""

import mujoco
import numpy as np

def test_mujoco():
    """Test basic MuJoCo functionality"""
    print("Testing MuJoCo installation...")
    print(f"MuJoCo version: {mujoco.__version__}")
    
    # Create a simple model (empty world)
    xml = """
    <mujoco>
        <worldbody>
            <geom name="floor" type="plane" size="10 10 0.1" rgba="0.5 0.5 0.5 1"/>
            <body name="box" pos="0 0 1">
                <geom name="box_geom" type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
                <joint name="free_joint" type="free"/>
            </body>
        </worldbody>
    </mujoco>
    """
    
    try:
        # Load model
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        
        # Run a few simulation steps
        for i in range(10):
            mujoco.mj_step(model, data)
        
        print("✅ MuJoCo test successful!")
        print(f"   Simulation time: {data.time:.3f}s")
        print(f"   Box position: {data.qpos[:3]}")
        
    except Exception as e:
        print(f"❌ MuJoCo test failed: {e}")
        return False
    
    return True

if __name__ == "__main__":
    test_mujoco()