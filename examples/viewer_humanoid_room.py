import mujoco
from mujoco import viewer
from pathlib import Path
import numpy as np

xml = """
<mujoco model="humanoid_room">
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="8 8 0.1" rgba="0.95 0.95 0.95 1"/>
    <geom type="box" pos="0.7 0.0 0.25" size="0.25 0.25 0.25" rgba="0.4 0.7 0.9 1"/>
    <geom type="cylinder" pos="-0.7 -0.4 0.2" size="0.2 0.2" rgba="0.9 0.6 0.4 1"/>
    <body name="torso" pos="0 0 1.1">
      <joint name="root" type="free"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.12" rgba="0.8 0.6 0.4 1"/>
      <body name="thigh_l" pos="0.05 0 0">
        <joint name="thigh_l_joint" type="hinge" axis="1 0 0" range="-45 45"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.07"/>
        <body name="shank_l" pos="0 0 -0.3">
          <joint name="shank_l_joint" type="hinge" axis="1 0 0" range="-90 0"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.06"/>
        </body>
      </body>
      <body name="thigh_r" pos="-0.05 0 0">
        <joint name="thigh_r_joint" type="hinge" axis="1 0 0" range="-45 45"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.07"/>
        <body name="shank_r" pos="0 0 -0.3">
          <joint name="shank_r_joint" type="hinge" axis="1 0 0" range="-90 0"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.06"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="thigh_l_joint" gear="80"/>
    <motor joint="shank_l_joint" gear="60"/>
    <motor joint="thigh_r_joint" gear="80"/>
    <motor joint="shank_r_joint" gear="60"/>
  </actuator>
</mujoco>
"""

script_dir = Path(__file__).parent
xml_path = script_dir / "humanoid_room.xml"
xml_path.write_text(xml)
m = mujoco.MjModel.from_xml_path(str(xml_path))
d = mujoco.MjData(m)
torso_id = m.body("torso").id

# Petit contrôleur "push" via touches (flèches) dans le viewer:
# gauche/droite/avant/arrière = appliquer une force courte sur le torse.
force = 250.0
duration = 30
push_timer = 0
push_dir = np.zeros(3)

def key_callback(keycode):
    global push_timer, push_dir
    # keycodes pour flèches (approx vus par glfw)
    # gauche=263, droite=262, haut=265, bas=264, espace=32
    if keycode == 263:   # gauche
        push_dir = np.array([0, +1, 0])
        push_timer = duration
    elif keycode == 262: # droite
        push_dir = np.array([0, -1, 0])
        push_timer = duration
    elif keycode == 265: # haut (vers l'avant x+)
        push_dir = np.array([+1, 0, 0])
        push_timer = duration
    elif keycode == 264: # bas (vers l'arrière x-)
        push_dir = np.array([-1, 0, 0])
        push_timer = duration
    elif keycode == 32:  # espace -> reset
        d.reset()

with viewer.launch_passive(m, d, key_callback=key_callback) as v:
    while v.is_running():
        if push_timer > 0:
            dirn = push_dir / (np.linalg.norm(push_dir) or 1.0)
            d.xfrc_applied[torso_id,:3] = force * dirn
            push_timer -= 1
        else:
            d.xfrc_applied[torso_id,:3] = 0.0
        mujoco.mj_step(m, d)
        v.sync()
