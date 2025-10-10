# MuJoCo Desktop Environment Docker Image

A complete Ubuntu Desktop environment with MuJoCo physics simulation, Jupyter notebooks, and development tools - all accessible through your web browser.

## 🚀 Quick Start

```bash
# Pull and run the image
docker run -d \
  --name mujoco-env \
  -p 6080:6080 \
  -p 8888:8888 \
  yguel/mujoco-desktop:latest

# Access the environment
# Desktop: http://localhost:6080
# Jupyter: http://localhost:8888
```

## 📦 What's Included

- **Ubuntu 24.04** with MATE Desktop
- **MuJoCo 3.3.6** physics simulation
- **Python 3.12** with scientific libraries
- **Jupyter Notebook** for interactive development
- **VS Code** for advanced coding
- **Chromium Browser** for documentation
- **noVNC** for web-based desktop access

## 🛠️ Usage Examples

### Basic Usage
```bash
docker run -d -p 6080:6080 -p 8888:8888 yguel/mujoco-desktop:latest
```

### With Persistent Storage
```bash
# Create workspace directory
mkdir -p ./workspace

# Run with volume mounting
docker run -d \
  --name mujoco-env \
  -p 6080:6080 \
  -p 8888:8888 \
  -v ./workspace:/home/student/workspace \
  yguel/mujoco-desktop:latest
```

### Using Docker Compose
```yaml
services:
  mujoco:
    image: yguel/mujoco-desktop:latest
    ports:
      - "6080:6080"
      - "8888:8888"
    volumes:
      - ./workspace:/home/student/workspace
    environment:
      - MUJOCO_GL=osmesa
```

## 🌐 Access Points

- **Desktop Environment**: http://localhost:6080
  - Full Ubuntu MATE desktop
  - Pre-installed VS Code, terminal, file manager
  - MuJoCo examples ready to run

- **Jupyter Notebooks**: http://localhost:8888
  - No password required
  - MuJoCo pre-configured
  - Scientific Python stack included

## 📁 File Structure

```
/home/student/workspace/
├── notebooks/     # Jupyter notebooks
├── examples/      # MuJoCo example scripts
└── models/        # MuJoCo model files
```

## 🔧 Environment Variables

- `MUJOCO_GL=osmesa` - Headless OpenGL rendering
- `DISPLAY=:1` - VNC display
- `USER=student` - Default user

## 💡 Tips

1. **No passwords needed** - Environment is configured for development ease
2. **Persistent data** - Mount volumes to save your work
3. **Multiple access methods** - Use desktop OR Jupyter, or both
4. **Pre-configured** - MuJoCo examples work out of the box

## 🧪 Test MuJoCo

After starting the container, test MuJoCo with:

```python
import mujoco
import numpy as np

# Create simple model
xml = """
<mujoco>
  <worldbody>
    <geom name="floor" size="0 0 0.1" type="plane"/>
    <body name="box" pos="0 0 1">
      <geom name="box_geom" size="0.1 0.1 0.1" type="box"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
print("MuJoCo is working!")
```

## 🏗️ Building from Source

```bash
git clone <your-repo>
cd docker/
./build-for-dockerhub.sh
./push-to-dockerhub.sh
```

## 📋 System Requirements

- Docker installed
- 4GB+ RAM recommended
- Ports 6080 and 8888 available

## 🆘 Troubleshooting

- **Black screen**: Wait 30 seconds for desktop to load
- **Jupyter not accessible**: Check port 8888 is not in use
- **Performance issues**: Increase Docker memory allocation

## 📄 License

MIT License - Feel free to use and modify!