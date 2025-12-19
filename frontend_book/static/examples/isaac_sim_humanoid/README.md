# Isaac Sim Humanoid Synthetic Data Generation

Generate photorealistic synthetic training data for object detection and semantic segmentation using NVIDIA Isaac Sim 5.1.0.

## 📋 Contents

- `scripts/import_urdf.py` - Import simple_humanoid.urdf into Isaac Sim scene
- `scripts/generate_synthetic_data.py` - Generate 1000 images with domain randomization
- `scripts/export_coco.py` - Convert annotations to COCO format
- `urdf/simple_humanoid.urdf` - Humanoid robot URDF (from Module 1)
- `humanoid_warehouse.usd` - Pre-configured Isaac Sim scene (create manually, see below)

## 🚀 Quick Start

### Prerequisites

- **Ubuntu 22.04 LTS** (native or dual-boot)
- **NVIDIA RTX GPU**: RTX 2060+ (6GB VRAM minimum, 12GB recommended)
- **NVIDIA Driver**: v580.65.06 or later
- **Docker**: Latest version with NVIDIA Container Toolkit
- **Disk Space**: 30GB for Isaac Sim Docker image

### Step 1: Pull Isaac Sim Docker Image

```bash
# Login to NVIDIA NGC (create free account at ngc.nvidia.com)
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>

# Pull Isaac Sim 5.1.0
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

### Step 2: Launch Isaac Sim Container

```bash
# Allow Docker to access X server
xhost +local:docker

# Run Isaac Sim container
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  -v $(pwd):/workspace:rw \
  --network=host \
  -e DISPLAY=$DISPLAY \
  nvcr.io/nvidia/isaac-sim:5.1.0
```

### Step 3: Import Humanoid URDF

```bash
# Inside Docker container
cd /workspace/scripts
../python.sh import_urdf.py
```

**Expected Output**: Humanoid robot appears at origin with all joints and links properly configured.

### Step 4: Create Warehouse Scene (Manual)

**Note**: USD scenes are complex - create manually in Isaac Sim GUI:

1. **File → New** (blank scene)
2. **Menu → Isaac Examples → Environments → Simple Warehouse** (loads warehouse assets)
3. Delete default robot (keep warehouse environment)
4. Import humanoid from Step 3 or re-run `import_urdf.py`
5. **Create → Camera**, attach to `/World/Humanoid/Head` prim
6. Position camera forward-looking (0.1m offset from head center)
7. **Create → Light → Distant Light** (directional sun light, intensity 1000 lux)
8. **Create → Light → Dome Light** (ambient environment, intensity 500)
9. **File → Save As** → `humanoid_warehouse.usd`

### Step 5: Generate Synthetic Data

```bash
# Generate 1000 images (8-12 minutes on RTX 3060)
../python.sh generate_synthetic_data.py --num-images 1000 --output-dir /root/Documents/synthetic_data

# Check output
ls /root/Documents/synthetic_data/
# Should show: rgb/ depth/ segmentation/ annotations/
```

### Step 6: Export to COCO Format

```bash
# Convert to COCO JSON
python export_coco.py --input-dir /root/Documents/synthetic_data --output coco_annotations.json

# Verify COCO format
python -c "import json; data = json.load(open('coco_annotations.json')); print(f\"Images: {len(data['images'])}, Annotations: {len(data['annotations'])}\")"
```

## 📊 Expected Output

### Dataset Structure

```
synthetic_data/
├── rgb/                    # 1000 RGB images (1280x720 PNG)
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
├── depth/                  # 1000 depth maps (16-bit PNG, millimeters)
│   ├── 000000.png
│   └── ...
├── segmentation/           # 1000 semantic segmentation masks
│   ├── 000000.png
│   └── ...
├── annotations/            # 1000 bounding box JSONs (Isaac Sim format)
│   ├── 000000.json
│   └── ...
└── coco_annotations.json   # COCO-format annotations
```

### Performance Benchmarks (RTX 3060 12GB)

- **Image Generation**: ~1.2 images/second (1000 images in 8-12 minutes)
- **VRAM Usage**: 6-8GB during generation
- **Disk Space**: ~2GB for 1000 images (RGB + depth + segmentation)

## 🧪 Validation Tests

### Test 1: Verify Image Count

```bash
ls synthetic_data/rgb/ | wc -l
# Expected: 1000
```

### Test 2: Check Image Resolution

```bash
file synthetic_data/rgb/000000.png
# Expected: PNG image data, 1280 x 720, 8-bit/color RGB
```

### Test 3: Validate COCO JSON

```python
from pycocotools.coco import COCO
coco = COCO('coco_annotations.json')
print(f"Images: {len(coco.imgs)}")          # Expected: 1000
print(f"Annotations: {len(coco.anns)}")     # Expected: 1000+ (1+ bbox per image)
print(f"Categories: {coco.cats}")           # Expected: {1: {'id': 1, 'name': 'humanoid', ...}}
```

## ⚙️ Customization

### Adjust Image Count

```bash
# Generate 10,000 images (for large-scale training)
../python.sh generate_synthetic_data.py --num-images 10000 --output-dir /root/Documents/large_dataset
```

### Change Resolution

```bash
# High-resolution (1920x1080)
../python.sh generate_synthetic_data.py --resolution 1920x1080 --num-images 1000

# Low-resolution (640x480, faster generation)
../python.sh generate_synthetic_data.py --resolution 640x480 --num-images 1000
```

### Add Multiple Categories

```bash
# If scene has multiple object types (humanoid, obstacle, floor)
python export_coco.py --input-dir synthetic_data --categories "humanoid,obstacle,floor"
```

## 🐛 Troubleshooting

### Issue: CUDA Out of Memory

**Symptoms**: Script crashes with "RuntimeError: CUDA out of memory"

**Solution**:
```bash
# Reduce resolution
../python.sh generate_synthetic_data.py --resolution 640x480 --num-images 1000

# Or use headless mode (edit generate_synthetic_data.py)
# Change: SimulationApp({"headless": True})
```

### Issue: Low FPS (<1 image/second)

**Symptoms**: Generation takes >20 minutes for 1000 images

**Solution**:
- Disable RTX ray tracing (Edit → Preferences → Rendering → Path Tracing: False)
- Reduce PhysX substeps (Edit → Preferences → Physics → Num Substeps: 1)
- Close other GPU applications (web browsers, video players)

### Issue: Docker Container Won't Start

**Symptoms**: "could not select device driver" error

**Solution**:
```bash
# Verify GPU access
nvidia-smi

# Reinstall NVIDIA Container Toolkit
sudo apt update
sudo apt install nvidia-docker2
sudo systemctl restart docker

# Test GPU in Docker
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

### Issue: Empty Bounding Boxes

**Symptoms**: `coco_annotations.json` has 0 annotations

**Solution**:
- Ensure humanoid robot is visible in camera view
- Check `/World/Humanoid` prim exists in scene
- Verify camera is attached to head and looking forward
- Increase lighting intensity (objects may be too dark to detect)

## 📚 Additional Resources

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/)
- [COCO Dataset Format](http://cocodataset.org/#format-data)
- [Domain Randomization Paper (ICRA 2019)](https://doi.org/10.1109/ICRA.2019.8794443)
- [NVIDIA Isaac Forum](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/69)

## 📝 License

Educational use only. See parent repository for full license.

---

**Tested On**: RTX 3060 (12GB VRAM), Ubuntu 22.04, Isaac Sim 5.1.0, Docker 20.10.21

**Chapter Reference**: Module 3, Chapter 1 - Isaac Sim Photorealistic Simulation
