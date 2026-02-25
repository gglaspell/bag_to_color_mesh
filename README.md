# ROS 2 Bag to Color 3D Mesh Converter

Convert ROS 2 bag files containing LiDAR point clouds and camera images into high-quality color 3D models with precision-tuned registration and drift correction.

<img width="378" height="281" alt="Screenshot from 2026-02-11 17-51-17" src="https://github.com/user-attachments/assets/a4d13eb0-691d-4f5c-bd26-6905261f9116" />
<img width="378" height="281" alt="Screenshot from 2026-02-11 18-02-10" src="https://github.com/user-attachments/assets/6dd92336-0500-42ec-930d-40f1bd15e371" />

---

## 🎨 What's New: Color Mesh Generation

This unified tool transforms ROS 2 bag files with both **LiDAR** and **RGB camera** data into:
- **Color Point Cloud** (`.ply`) - RGB-color 3D representation of your environment
- **Color Surface Mesh** (`.ply`) - Vertex-color 3D model ready for visualization

**Key Features:**
- **Camera-to-LiDAR Color Projection** - Projects RGB images onto point clouds using calibrated intrinsics
- **Smart Gray Filtering** - Removes gray (uncolor) points near color data for clean results
- **Intelligent Voxel Downsampling** - Keeps most saturated colors instead of averaging
- **Color Poisson Meshing** - Generates smooth meshes with vertex colors
- **Automatic Fallback** - Works without camera data (generates grayscale mesh like before)
- **View-Ray Normal Orientation** - Tracks sensor origins per frame to orient geometric normals, eliminating "inside-out" or flipped surfaces during Poisson meshing.
- **Advanced Noise Cleaning Pipeline** - Incorporates Voxel Downsampling, filters, and DBSCAN clustering to strip away wispy noise, floating artifacts, and distant outliers.


---

## 📋 What This Does

### Complete Pipeline

1. **Extract Data** - Point clouds, odometry, and camera images from ROS 2 bag
2. **Register Point Clouds** - Point-to-Plane ICP with optional loop closure detection
3. **Global Optimization** - Pose graph optimization to correct drift
4. **Color Projection** - Project camera colors onto registered point clouds
5. **Merge & Filter** - Smart filtering and downsampling of color points
6. **Mesh Generation** - Align geometric normals using historical view rays, followed by high-quality Poisson reconstruction with vertex colors

### Works in Two Modes

**🎨 Color Mode** (with camera data):
- Requires: Point clouds + Camera images + Camera intrinsics
- Output: Color point cloud + Color mesh

**⚫ Grayscale Mode** (without camera data):
- Requires: Point clouds only
- Output: Grayscale point cloud + Grayscale mesh (original functionality)

---

## 🛠 What You'll Need

### Required
- **Docker** ([Install Docker](https://docs.docker.com/get-docker/))
- **ROS 2 Bag File** with `sensor_msgs/PointCloud2` messages
- **8GB+ RAM** (16GB+ recommended for large datasets)

### For Color Output (Optional)
- **Camera topic** in bag (`sensor_msgs/Image` or `sensor_msgs/CompressedImage`)
- **Camera intrinsics** (fx, fy, cx, cy from calibration)
- **Synchronized camera and LiDAR data** (timestamps within ~100ms)

---

## 🚀 Getting Started

### 1. Set Up Your Project

```bash
mkdir bag-to-color-mesh && cd bag-to-color-mesh
# Place Dockerfile_color and bag_to_color_mesh.py here
```

### 2. Build the Docker Image

```bash
docker build -f Dockerfile -t bag-to-color-mesh .
```

This downloads dependencies (~2.5GB) and takes 5-10 minutes on first build.

### 3. Organize Your Data

```bash
mkdir -p data/input data/output
# Copy your ROS 2 bag file to data/input/
```

### 4. Run Your First Color Mesh Conversion

**Basic command** (grayscale mode - no camera):

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-color-mesh \
  /app/data/input/your_bag_file \
  /app/data/output
```

**Color mode** (with camera and intrinsics):

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-color-mesh \
  /app/data/input/your_bag_file \
  /app/data/output \
  --pc_topic /velodyne_points \
  --camera_topic /camera/image_raw/compressed \
  --camera_fx 615.123 \
  --camera_fy 615.456 \
  --camera_cx 320.0 \
  --camera_cy 240.0 \
  --camera_width 640 \
  --camera_height 480 \
  --odom_topic /odom
```

### 5. View Your Results

Output files will be in `data/output/`:

**Color Mode:**
- `your_bag_file_color_cloud.ply` - RGB-color point cloud
- `your_bag_file_color_mesh.ply` - Color mesh (PLY format)
- `your_bag_file_color_mesh.obj` - Color mesh (OBJ format)

**Grayscale Mode:**
- `your_bag_file_cloud.ply` - Point cloud
- `your_bag_file_mesh.obj` - Mesh

---

## 💡 Common Use Cases

### Indoor Mapping with TurtleBot + Camera

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-color-mesh \
  /app/data/tb3_office_scan \
  /app/data/output \
  --pc_topic /scan/points \
  --camera_topic /camera/rgb/image_raw \
  --camera_fx 525.0 \
  --camera_fy 525.0 \
  --camera_cx 319.5 \
  --camera_cy 239.5 \
  --camera_width 640 \
  --camera_height 480 \
  --odom_topic /odom \
  --voxel_size 0.02 \
  --icp_fitness_thresh 0.5 \
  --level_floor
```

**Why these settings?**
- Camera intrinsics from typical RGB-D camera (e.g., Kinect, RealSense)
- `voxel_size 0.02` - Fine detail for indoor spaces
- `level_floor` - Flattens floors in multi-room scans

---

### Outdoor Mapping with Velodyne + Camera

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-color-mesh \
  /app/data/outdoor_survey \
  /app/data/output \
  --pc_topic /velodyne_points \
  --camera_topic /camera/image_color/compressed \
  --camera_info_topic /camera/camera_info \
  --odom_topic /integrated_odom \
  --voxel_size 0.1 \
  --icp_fitness_thresh 0.3 \
  --max_time_diff 0.05

```

**Why these settings?**
- Higher resolution camera (1280x720)
- `max_time_diff 0.05` - Tighter sync for fast-moving vehicle
- `voxel_size 0.1` - Larger voxels for outdoor scale

---

### High-Quality Object Scanning (RGB-D)

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-color-mesh \
  /app/data/object_scan \
  /app/data/output \
  --pc_topic /camera/depth/color/points \
  --camera_topic /camera/color/image_raw \
  --camera_fx 615.3 \
  --camera_fy 615.7 \
  --camera_cx 324.2 \
  --camera_cy 238.1 \
  --camera_width 640 \
  --camera_height 480 \
  --voxel_size 0.005 \
  --icp_dist_thresh 0.05 \
  --icp_fitness_thresh 0.7 \
  --poisson_depth 10 \
  --max_vertex_distance 0.05
```

**Why these settings?**
- `voxel_size 0.005` - Maximum detail preservation
- `poisson_depth 10` - Higher mesh resolution
- `max_vertex_distance 0.05` - Tight mesh trimming

---

### Large Campus/Warehouse (with Loop Closure)

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-color-mesh \
  /app/data/warehouse_full \
  /app/data/output \
  --pc_topic /velodyne_points \
  --camera_topic /camera/image_raw/compressed \
  --camera_fx 700.0 \
  --camera_fy 700.0 \
  --camera_cx 512.0 \
  --camera_cy 384.0 \
  --camera_width 1024 \
  --camera_height 768 \
  --odom_topic /integrated_odom \
  --voxel_size 0.1 \
  --icp_fitness_thresh 0.3 \
  --enable_loop_closure \
  --level_floor
```

**Why these settings?**
- Loop closure for drift correction in large environments
- Aggressive downsampling for performance

---

## 📖 Parameter Reference

### Required Parameters

| Parameter | Description |
|-----------|-------------|
| `bag_path` | Path to ROS 2 bag file |
| `output_dir` | Where to save output files |

### Camera Parameters (Required for Color Output)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--camera_topic` | `None` | Camera topic (Image or CompressedImage) |
| `--camera_info_topic` | `None` | Camera infor topic |
| `--camera_fx` | `None` | Focal length x (pixels) |
| `--camera_fy` | `None` | Focal length y (pixels) |
| `--camera_cx` | `None` | Principal point x (pixels) |
| `--camera_cy` | `None` | Principal point y (pixels) |
| `--camera_width` | `640` | Image width (pixels) |
| `--camera_height` | `480` | Image height (pixels) |
| `--max_time_diff` | `0.1` | Max time diff (sec) for camera/lidar sync |

### Point Cloud & Odometry Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--pc_topic` | `/points` | PointCloud2 topic name |
| `--odom_topic` | `None` | Odometry topic (nav_msgs/Odometry) |

### Registration Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `--voxel_size` | `0.05` | 0.001-1.0 | Downsampling resolution (meters) |
| `--icp_dist_thresh` | `0.2` | 0.01-10.0 | Max point correspondence distance (m) |
| `--icp_fitness_thresh` | `0.6` | 0.0-1.0 | Min % of points aligned to accept frame |

### Loop Closure Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--enable_loop_closure` | `False` | Enable loop closure detection |
| `--loop_closure_radius` | `10.0` | Search radius for loop closure (m) |
| `--loop_closure_fitness_thresh` | `0.3` | Min fitness for loop closure |
| `--loop_closure_search_interval` | `10` | Search every N frames |

### Color Processing Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--gray_filter_radius` | `0.05` | Remove gray points within distance (m) of color points |

### Mesh Generation Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--poisson_depth` | `9` | Poisson octree depth (higher = more detail) |
| `--min_density_percentile` | `1.0` | Filter vertices below this density percentile |
| `--max_vertex_distance` | `0.15` | Max distance (m) from mesh vertex to point |

### Other Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--level_floor` | `False` | Apply post-processing Z-leveling |

---

## 🎯 Pro Tips

### Getting Camera Intrinsics

**From ROS CameraInfo:**

```bash
ros2 topic echo /camera/camera_info --once
```

Look for the `K` matrix:
```
K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
```

**From Calibration File:**

Check your camera's calibration YAML file (usually in `~/.ros/camera_info/`):
```yaml
camera_matrix:
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
```

### Finding Your Topics

Don't know your topic names? Run:

```bash
ros2 bag info /path/to/your/bag_file
```

Look for:
- `sensor_msgs/msg/PointCloud2` - Your point cloud topic
- `sensor_msgs/msg/Image` or `CompressedImage` - Your camera topic
- `nav_msgs/msg/Odometry` - Your odometry topic

### Camera-LiDAR Time Synchronization

The `--max_time_diff` parameter controls how closely camera and LiDAR timestamps must match:

- **Slow-moving robots** (indoor): `--max_time_diff 0.1` (default, 100ms)
- **Fast-moving vehicles** (outdoor): `--max_time_diff 0.05` (50ms)
- **Very fast or unsynchronized**: `--max_time_diff 0.2` (but expect more ghosting)

### Performance Tuning

**Speed vs Quality Trade-offs:**

| Goal | Voxel Size | Gray Filter | Poisson Depth |
|------|------------|-------------|---------------|
| 🚀 Fast Preview | 0.1 | 0.1 | 8 |
| ⚖️ Balanced | 0.05 | 0.05 | 9 |
| 🎨 Maximum Quality | 0.02 | 0.03 | 10 |

### Color Quality Tips

**Better colors:**
- Ensure camera and LiDAR are well-calibrated (extrinsic calibration)
- Use lower `--max_time_diff` for better sync
- Reduce `--gray_filter_radius` to keep more color data

**Cleaner mesh:**
- Increase `--gray_filter_radius` to remove more gray points
- Reduce `--max_vertex_distance` to trim outliers
- Increase `--min_density_percentile` to remove low-confidence areas

---

## 🔧 Troubleshooting

### "Error: --camera_topic requires camera intrinsics"

**Problem:** You specified a camera topic but not the intrinsics.

**Fix:** Add all camera parameters:
```bash
--camera_fx 615.0 \
--camera_fy 615.0 \
--camera_cx 320.0 \
--camera_cy 240.0
```

**Problem:** Final mesh is missing large parts of the environment (e.g., separate rooms disappeared).

**Fix:** The new DBSCAN filter isolates the largest continuous structure to remove floating noise. If your environment has disconnected areas, you may need to reduce your --voxelsize so gaps appear smaller to the clustering algorithm.

---

### Point Cloud Has No Colors / All Gray

**Problem:** Camera images not syncing with point clouds.

**Fixes (in order):**

1. **Check time synchronization:**
   ```bash
   --max_time_diff 0.2  # Increase tolerance
   ```

2. **Verify camera topic:**
   ```bash
   ros2 bag info your_bag_file
   # Ensure camera topic exists and has messages
   ```

3. **Check intrinsics:** Verify fx, fy, cx, cy match your camera

4. **Test without compression:** If using CompressedImage, try raw Image topic

---

### Mesh Colors Look Washed Out / Gray

**Problem:** Too many gray (uncolor) points in the merged cloud.

**Fix:**
```bash
--gray_filter_radius 0.1  # Increase to remove more gray points
--voxel_size 0.03  # Smaller voxels preserve more color detail
```

---

### "Error: No messages found for topics"

**Problem:** Topic names don't match your bag file.

**Fix:**
```bash
# Check available topics
ros2 bag info your_bag_file

# Use exact topic names
--pc_topic /exact/pointcloud/topic \
--camera_topic /exact/camera/topic
```

---

### Processing is Very Slow

**Problem:** Color projection and merging is slow for large datasets.

**Speed-up strategies:**

1. **Increase voxel size** (biggest impact):
   ```bash
   --voxel_size 0.1
   ```

2. **Relax gray filtering**:
   ```bash
   --gray_filter_radius 0.1
   ```

3. **Reduce mesh detail**:
   ```bash
   --poisson_depth 8
   ```

4. **Filter bag first** (pre-process):
   ```bash
   ros2 bag filter input.bag output.bag \
     "t.sec % 2 == 0"  # Keep every other second
   ```

---

### Mesh Has Holes or Missing Areas

**Problem:** Poisson reconstruction failing in sparse regions.

**Fixes:**

1. **Lower depth** (fills gaps better):
   ```bash
   --poisson_depth 8
   ```

2. **Reduce distance trimming**:
   ```bash
   --max_vertex_distance 0.25
   ```

3. **Lower density threshold**:
   ```bash
   --min_density_percentile 0.5
   ```

---

### Colors Don't Align with Geometry

**Problem:** Camera-LiDAR extrinsic calibration is off.

**This script assumes camera and LiDAR are co-located (same pose).**

If your setup has camera offset from LiDAR, you need to:
1. Perform extrinsic calibration (e.g., using `camera_lidar_calibration` package)
2. Modify the `camera_pose` calculation in the script to apply the transform

---

## ⚠️ Known Limitations

### Camera-LiDAR Alignment
- **Assumes co-located sensors** (camera and LiDAR at same position)
- For offset sensors, extrinsic calibration is required (manual code modification)

### Color Projection
- Uses **ROS body to optical frame transform**: `[-y, -z, x]`
- May need adjustment for non-standard camera orientations

### Time Synchronization
- Relies on **timestamp matching** (not hardware sync)
- Best results with hardware-synchronized sensors
- Fast-moving platforms may have color misalignment

### Scale
- Tested up to **5000 frames** with colors
- Larger datasets may require **32GB+ RAM**
- Color processing adds ~30% to processing time

### Loop Closure
- **Disabled by default** for speed (no change from original)
- Enable for large-scale mapping with `--enable_loop_closure`
- Adds 3-8x processing time

---

## 📊 Performance Guide

### Processing Times

| Dataset | Frames | Mode | Time | Peak RAM |
|---------|--------|------|------|----------|
| Small room | 500 | Grayscale | 1-2 min | 4 GB |
| Small room | 500 | Color | 2-3 min | 4 GB |
| Office floor | 1500 | Grayscale | 3-5 min | 8 GB |
| Office floor | 1500 | Color | 5-8 min | 8 GB |
| Large warehouse | 3000 | Grayscale | 8-12 min | 12 GB |
| Large warehouse | 3000 | Color | 12-18 min | 12 GB |

**Color processing adds ~30-50% to total processing time.**

---

## 🔗 Viewing Your Results

### Point Clouds (`.ply`)
- **CloudCompare** (free): https://www.cloudcompare.org/
- **MeshLab** (free): https://www.meshlab.net/
- **Open3D Viewer**: `python -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('file.ply')])"`

### Meshes (`.ply`, `.obj`)
- **Blender** (free, powerful): https://www.blender.org/
- **MeshLab** (free): https://www.meshlab.net/
- **Online viewer**: https://3dviewer.net/

---

## 🤝 Getting Help

**Check your logs for these key metrics:**

```
Extracted X point clouds          ← Should be >100
Extracted Y camera images          ← Should be >0 for color mode
Coloring Z point clouds            ← Shows color projection progress
Total points: A (B gray, C color) ← Should have C > 0 for good colors
Final mesh: V vertices, F faces    ← Mesh statistics
```

If colors aren't working:
1. Verify camera topic exists in bag
2. Check camera intrinsics are correct
3. Try increasing `--max_time_diff`
4. Verify camera and LiDAR are synchronized

---

**Built with:** Open3D • NumPy • SciPy • rosbags • Pillow • pandas • Python 3.10
