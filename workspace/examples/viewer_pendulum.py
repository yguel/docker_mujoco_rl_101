import mujoco
from mujoco import viewer
from pathlib import Path

xml = """
<mujoco model="pendulum">
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="link" pos="0 0 1.0">
      <joint name="hinge" type="hinge" axis="0 1 0" range="-180 180"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05" rgba="0.3 0.6 0.9 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" gear="1.0"/>
  </actuator>
</mujoco>
"""
Path("/home/student/workspace/examples/pendulum.xml").write_text(xml)
m = mujoco.MjModel.from_xml_path("/home/student/workspace/examples/pendulum.xml")
d = mujoco.MjData(m)

with viewer.launch_passive(m, d) as v:
    while v.is_running():
        d.ctrl[:] = 0.1   # couple constant
        mujoco.mj_step(m, d)
        v.sync()
