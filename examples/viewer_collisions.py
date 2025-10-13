import mujoco
from mujoco import viewer
from pathlib import Path

xml = """
<mujoco model="collisions_demo">
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="boxA" pos="-0.25 0 1.3">
      <joint type="free"/>
      <geom type="box" size="0.12 0.12 0.12" rgba="0.2 0.6 0.9 1"/>
    </body>
    <body name="boxB" pos="0.25 0 1.6">
      <joint type="free"/>
      <geom type="box" size="0.12 0.12 0.12" rgba="0.9 0.3 0.3 1"/>
    </body>
  </worldbody>
</mujoco>
"""
script_dir = Path(__file__).parent
xml_path = script_dir / "collisions_demo.xml"
xml_path.write_text(xml)
m = mujoco.MjModel.from_xml_path(str(xml_path))
d = mujoco.MjData(m)

with viewer.launch_passive(m, d) as v:
    while v.is_running():
        mujoco.mj_step(m, d)
        v.sync()
