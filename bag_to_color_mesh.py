#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bag_to_color_mesh.py

Unified ROS 2 Bag -> Registered Point Cloud -> (Optional) Colored Point Cloud -> Poisson Mesh

Added (from bag_to_mesh.py):
1) View-ray-tracked normal orientation:
   - For each accepted frame, transform points into world.
   - Compute view rays = (sensor_origin_world - world_point), unit.
   - Store those rays in the point cloud's .normals channel BEFORE voxel downsampling.
   - Voxel downsample averages points AND these "view ray normals" per voxel.
   - After cleaning, estimate true geometric normals, then flip any geometric normal whose dot(view_ray) < 0.

2) Improved point cloud cleaning pipeline:
   - Voxel downsample (preserving view rays in normals)
   - Radius outlier removal (ROR)
   - Statistical outlier removal (SOR)
   - DBSCAN clustering (keep largest component)

Notes:
- View-ray orientation works best when your trajectory is reasonable (pose graph optimization helps).
- If view rays are unavailable/mismatched, we fall back to Open3D's consistent tangent plane orientation.
"""

import argparse
import copy
import logging
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm


# ----------------------------
# Logging
# ----------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)


# ----------------------------
# ROS2 message helpers
# ----------------------------
TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)

# sensor_msgs/PointField datatype -> numpy dtype
POINTFIELD_TO_DTYPE = {
    7: np.float32,  # FLOAT32
    8: np.float64,  # FLOAT64
}


def convert_ros_pc2_to_o3d(msg):
    """
    Convert ROS 2 sensor_msgs/PointCloud2 to Open3D PointCloud (XYZ only).
    Requires x,y,z to be present and use a supported float datatype.

    Returns: o3d.geometry.PointCloud or None
    """
    try:
        fields = {f.name: (int(f.offset), int(f.datatype)) for f in msg.fields}
        if "x" not in fields or "y" not in fields or "z" not in fields:
            return None

        xoff, xdt = fields["x"]
        yoff, ydt = fields["y"]
        zoff, zdt = fields["z"]

        if xdt not in POINTFIELD_TO_DTYPE or ydt not in POINTFIELD_TO_DTYPE or zdt not in POINTFIELD_TO_DTYPE:
            return None

        # Require consistent dtype across xyz for a clean structured view
        if not (xdt == ydt == zdt):
            return None

        npdt = POINTFIELD_TO_DTYPE[xdt]
        npoints = int(msg.width) * int(msg.height)
        if npoints <= 0:
            return None

        itemsize = int(msg.point_step)
        if itemsize <= 0:
            return None

        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [npdt, npdt, npdt],
                "offsets": [xoff, yoff, zoff],
                "itemsize": itemsize,
            }
        )

        arr = np.frombuffer(msg.data, dtype=dtype, count=npoints)
        pts = np.empty((npoints, 3), dtype=np.float64)
        pts[:, 0] = arr["x"]
        pts[:, 1] = arr["y"]
        pts[:, 2] = arr["z"]

        mask = np.isfinite(pts).all(axis=1)
        pts = pts[mask]
        if pts.shape[0] < 10:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd

    except Exception:
        return None


def convert_ros_image(msg, encoding: str):
    """Convert ROS Image message to PIL Image."""
    try:
        width = msg.width
        height = msg.height

        if encoding in ("rgb8", "bgr8"):
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            if encoding == "bgr8":
                data = data[:, :, ::-1]  # BGR -> RGB
            return Image.fromarray(data, "RGB")

        if encoding in ("mono8", "mono16"):
            dtype = np.uint8 if encoding == "mono8" else np.uint16
            data = np.frombuffer(msg.data, dtype=dtype).reshape(height, width)
            return Image.fromarray(data)

        logging.warning(f"Unsupported image encoding: {encoding}")
        return None

    except Exception as e:
        logging.warning(f"Error converting image: {e}")
        return None


def convert_compressed_image(msg):
    """Convert ROS CompressedImage message to PIL Image."""
    try:
        from io import BytesIO

        img = Image.open(BytesIO(bytes(msg.data)))
        return img.convert("RGB")
    except Exception as e:
        logging.warning(f"Error converting compressed image: {e}")
        return None


def intrinsics_from_camera_info(msg):
    """Extract fx, fy, cx, cy, width, height from sensor_msgs/CameraInfo"""
    k = list(msg.k)
    fx = float(k[0])
    fy = float(k[4])
    cx = float(k[2])
    cy = float(k[5])
    w = int(msg.width)
    h = int(msg.height)
    return fx, fy, cx, cy, w, h


def get_odom_transform(odom_msg):
    """Extract 4x4 transform from nav_msgs/Odometry pose."""
    try:
        pos = odom_msg.pose.pose.position
        quat = odom_msg.pose.pose.orientation
        t = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        rot = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rot
        T[:3, 3] = t
        return T
    except Exception:
        return None


def get_closest_timestamp(ts, ts_to_value: dict):
    """Return closest timestamp key in dict to `ts`."""
    if not ts_to_value:
        return None
    return min(ts_to_value.keys(), key=lambda k: abs(k - ts))


# ----------------------------
# Registration helpers
# ----------------------------
def compute_fpfh_descriptor(pcd, voxel_size: float):
    radius_normal = voxel_size * 2.0
    radius_feature = voxel_size * 5.0

    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
        )

    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100),
    )
    return fpfh


def ransac_coarse_alignment(source, target, source_fpfh, target_fpfh, voxel_size: float, ransac_thresh_mult: float = 5.0):
    distance_threshold = voxel_size * float(ransac_thresh_mult)

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source,
        target,
        source_fpfh,
        target_fpfh,
        mutual_filter=False,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000, 0.999),
    )

    if result.fitness > 0.1:
        return result.transformation
    return None


def detect_loop_closure(
    current_idx: int,
    current_pcd,
    current_fpfh,
    historical_pcds,
    historical_fpfhs,
    historical_poses,
    voxel_size: float,
    search_radius: float = 10.0,
    loop_fitness_thresh: float = 0.3,
    temporal_window: int = 100,
):
    """
    Find loop closures by:
    - only searching older frames outside temporal window
    - KD-tree radius search in pose positions
    - RANSAC coarse + ICP refine
    Returns list of (candidate_idx, transform, fitness)
    """
    if current_idx < temporal_window:
        return []

    search_indices = list(range(0, current_idx - temporal_window))
    if not search_indices:
        return []

    current_pos = historical_poses[current_idx][:3, 3]
    hist_positions = np.array([historical_poses[i][:3, 3] for i in search_indices], dtype=np.float64)
    if hist_positions.shape[0] == 0:
        return []

    pos_pcd = o3d.geometry.PointCloud()
    pos_pcd.points = o3d.utility.Vector3dVector(hist_positions)
    kdtree = o3d.geometry.KDTreeFlann(pos_pcd)
    _k, idxs, _dist2 = kdtree.search_radius_vector_3d(current_pos, float(search_radius))
    if not idxs:
        return []

    candidate_indices = [search_indices[i] for i in idxs]

    loop_closures = []
    for cand_idx in candidate_indices:
        cand_fpfh = historical_fpfhs[cand_idx]
        if cand_fpfh is None:
            continue

        coarse = ransac_coarse_alignment(current_pcd, historical_pcds[cand_idx], current_fpfh, cand_fpfh, voxel_size)
        if coarse is None:
            continue

        icp = o3d.pipelines.registration.registration_icp(
            current_pcd,
            historical_pcds[cand_idx],
            voxel_size * 2.0,
            coarse,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        if float(icp.fitness) >= float(loop_fitness_thresh):
            loop_closures.append((cand_idx, icp.transformation, float(icp.fitness)))

    return loop_closures


# ----------------------------
# View-ray normal orientation (ported)
# ----------------------------
def _safe_normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v, axis=1, keepdims=True)
    return v / np.clip(n, eps, None)


def attach_view_rays_as_normals(pcd_world: o3d.geometry.PointCloud, sensor_origin_world: np.ndarray) -> None:
    """
    Compute unit viewing rays (sensor_origin_world - point_world) and store them in pcd_world.normals.
    This should be called BEFORE voxel downsampling so the downsampler averages rays per voxel.
    """
    pts = np.asarray(pcd_world.points, dtype=np.float64)
    if pts.shape[0] == 0:
        return
    ray_dirs = sensor_origin_world.reshape(1, 3) - pts
    ray_dirs = _safe_normalize(ray_dirs, eps=1e-6)
    pcd_world.normals = o3d.utility.Vector3dVector(ray_dirs)


def orient_geometric_normals_with_view_rays(pcd: o3d.geometry.PointCloud, view_rays: np.ndarray) -> None:
    """
    Given pcd with estimated geometric normals, flip any normal that points away from view_rays.
    """
    geom = np.asarray(pcd.normals, dtype=np.float64)
    if geom.shape[0] == 0:
        return

    vr = np.asarray(view_rays, dtype=np.float64)
    if vr.shape != geom.shape:
        return

    vr = _safe_normalize(vr, eps=1e-6)
    geom = _safe_normalize(geom, eps=1e-12)

    dots = np.sum(geom * vr, axis=1)
    geom[dots < 0.0] *= -1.0

    pcd.normals = o3d.utility.Vector3dVector(geom)


def estimate_geometric_normals_oriented(
    pcd: o3d.geometry.PointCloud,
    voxel_size: float,
    view_rays: np.ndarray | None,
) -> None:
    """
    Estimate geometric normals and orient them using view rays when available.
    """
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3.0, max_nn=30)
    )
    pcd.normalize_normals()

    if view_rays is not None and view_rays.shape[0] == len(pcd.points):
        orient_geometric_normals_with_view_rays(pcd, view_rays)
    else:
        # fallback: Open3D's consistency heuristic
        try:
            pcd.orient_normals_consistent_tangent_plane(100)
        except Exception:
            pass


# ----------------------------
# Cleaning pipeline (ported)
# ----------------------------
def clean_point_cloud(
    pcd: o3d.geometry.PointCloud,
    voxel_size: float,
    do_voxel_downsample: bool = True,
) -> o3d.geometry.PointCloud:
    """
    Cleaning pipeline while preserving normals (view rays) if present:
    - (optional) voxel downsample
    - radius outlier removal
    - statistical outlier removal
    - DBSCAN clustering (keep largest component)
    """
    pcd_clean = pcd

    if do_voxel_downsample:
        logging.info("Voxel downsampling (preserves/averages normals if present)...")
        pcd_clean = pcd_clean.voxel_down_sample(voxel_size)

    # ROR
    logging.info("Radius outlier removal (cleaning wispy noise)...")
    try:
        pcd_tmp, _ = pcd_clean.remove_radius_outlier(nb_points=12, radius=voxel_size * 3.0)
        if len(pcd_tmp.points) > 0:
            pcd_clean = pcd_tmp
        else:
            logging.warning("ROR produced empty cloud; skipping this filter.")
    except Exception as e:
        logging.warning(f"ROR failed ({e}); skipping ROR.")

    # SOR
    logging.info("Statistical outlier removal (cleaning distant noise)...")
    try:
        pcd_tmp, _ = pcd_clean.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        if len(pcd_tmp.points) > 0:
            pcd_clean = pcd_tmp
        else:
            logging.warning("SOR produced empty cloud; skipping this filter.")
    except Exception as e:
        logging.warning(f"SOR failed ({e}); skipping SOR.")

    # DBSCAN
    logging.info("DBSCAN clustering (keeping largest component)...")
    try:
        labels = np.array(
            pcd_clean.cluster_dbscan(
                eps=voxel_size * 4.0,
                min_points=30,
                print_progress=False,
            )
        )
        if len(labels) > 0 and labels.max() >= 0:
            largest_cluster_idx = np.bincount(labels[labels >= 0]).argmax()
            pcd_tmp = pcd_clean.select_by_index(np.where(labels == largest_cluster_idx)[0])
            if len(pcd_tmp.points) > 0:
                pcd_clean = pcd_tmp
            else:
                logging.warning("DBSCAN isolated 0 points; skipping clustering filter.")
        else:
            logging.warning("DBSCAN found no valid clusters; skipping.")
    except Exception as e:
        logging.warning(f"DBSCAN failed ({e}); skipping DBSCAN.")

    return pcd_clean


# ----------------------------
# Color projection + merging
# ----------------------------
def color_point_cloud_from_image(pcd: o3d.geometry.PointCloud, img: Image.Image, camera_pose: np.ndarray, intrinsics: dict):
    """
    Project camera image colors onto point cloud.
    Assumes camera is co-located with the point cloud sensor unless you apply an extrinsic transform yourself.
    """
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return pcd

    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]
    w, h = intrinsics["width"], intrinsics["height"]

    img_data = np.asarray(img)

    cam_p = camera_pose[:3, 3]
    cam_R = R.from_matrix(camera_pose[:3, :3])

    rel = pts - cam_p
    cam = cam_R.inv().apply(rel)

    # ROS body -> optical approximation used in your original script: (-y, -z, x)
    opt = np.column_stack((-cam[:, 1], -cam[:, 2], cam[:, 0]))
    z = opt[:, 2]
    valid = z > 0.1

    u = fx * (opt[:, 0] / z) + cx
    v = fy * (opt[:, 1] / z) + cy

    valid &= (u >= 0) & (u < w) & (v >= 0) & (v < h)

    colors = np.full((len(pts), 3), 0.5, dtype=np.float64)  # default gray
    if np.any(valid):
        ui = np.clip(u[valid].astype(int), 0, w - 1)
        vi = np.clip(v[valid].astype(int), 0, h - 1)
        sampled = img_data[vi, ui] / 255.0
        colors[valid] = sampled

    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def merge_colored_point_clouds(colored_pcds, voxel_size: float = 0.02, gray_filter_radius: float = 0.05):
    """
    Merge colored point clouds with smart gray filtering:
    - Gray points are kept ONLY if there are no colored points within `gray_filter_radius`.
    - Preserves normals if present (used for view rays).
    - Uses FAST native voxel downsampling.
    """
    if not colored_pcds:
        raise ValueError("No point clouds to merge")

    all_pts = []
    all_cols = []
    all_nors = []
    all_is_gray = []

    gray_threshold = 0.1

    for pcd in colored_pcds:
        if len(pcd.points) == 0:
            continue
        if not pcd.has_colors():
            continue

        pts = np.asarray(pcd.points)
        cols = np.asarray(pcd.colors)
        nors = np.asarray(pcd.normals) if pcd.has_normals() else None

        color_var = np.std(cols, axis=1)
        is_gray = (color_var < gray_threshold) | (np.abs(cols[:, 0] - 0.5) < gray_threshold)

        all_pts.append(pts)
        all_cols.append(cols)
        all_is_gray.append(is_gray)

        # Preserve normals if present; if missing, fill zeros so indexing stays consistent
        if nors is None:
            nors = np.zeros((pts.shape[0], 3), dtype=np.float64)
        all_nors.append(nors)

    if not all_pts:
        raise ValueError("No valid colored point clouds")

    merged_pts = np.vstack(all_pts)
    merged_cols = np.vstack(all_cols)
    merged_nors = np.vstack(all_nors)
    merged_is_gray = np.hstack(all_is_gray)

    colored_mask = ~merged_is_gray
    colored_pts = merged_pts[colored_mask]

    if len(colored_pts) > 0:
        logging.info(f"Gray filtering radius={gray_filter_radius}m...")
        tree = cKDTree(colored_pts)

        gray_indices = np.where(merged_is_gray)[0]
        gray_pts = merged_pts[gray_indices]

        neighbors = tree.query_ball_point(gray_pts, r=gray_filter_radius)
        has_col_neighbor = np.array([len(n) > 0 for n in neighbors], dtype=bool)

        gray_to_remove = gray_indices[has_col_neighbor]
        keep = np.ones(len(merged_pts), dtype=bool)
        keep[gray_to_remove] = False

        merged_pts = merged_pts[keep]
        merged_cols = merged_cols[keep]
        merged_nors = merged_nors[keep]
    else:
        logging.warning("No colored points found; keeping all gray points.")

    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(merged_pts)
    merged.colors = o3d.utility.Vector3dVector(merged_cols)

    # Only attach normals if there were real normals in inputs (non-zero)
    if np.any(np.linalg.norm(merged_nors, axis=1) > 0):
        merged.normals = o3d.utility.Vector3dVector(_safe_normalize(merged_nors, eps=1e-12))

    logging.info(f"Fast voxel downsampling (voxel_size={voxel_size})...")
    # Native Open3D C++ downsampling (takes < 1 second instead of 10+ minutes)
    down = merged.voxel_down_sample(voxel_size)
    return down


# ----------------------------
# Meshing
# ----------------------------
def create_colored_mesh(
    pcd: o3d.geometry.PointCloud,
    depth: int = 9,
    min_density_percentile: float = 1.0,
    max_vertex_distance: float = 0.15,
):
    """
    Poisson reconstruction + trimming.
    Expects pcd to already have good normals (we estimate+orient them in the main pipeline).
    """
    logging.info(f"Input point cloud: {len(pcd.points)} points")

    if not pcd.has_normals():
        logging.info("Normals missing; estimating + orienting (fallback)...")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        try:
            pcd.orient_normals_consistent_tangent_plane(100)
        except Exception:
            pass

    logging.info(f"Running Poisson Reconstruction (depth={depth})...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=int(depth), linear_fit=True
    )

    logging.info(f"Initial mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} faces")

    densities = np.asarray(densities, dtype=np.float64)
    if densities.size > 0:
        thr = np.percentile(densities, float(min_density_percentile))
        logging.info(f"Density trim: bottom {min_density_percentile}% (threshold={thr:.4f})")
        mesh.remove_vertices_by_mask(densities < thr)

    if max_vertex_distance and float(max_vertex_distance) > 0:
        logging.info(f"Distance trim: removing vertices farther than {max_vertex_distance}m from points...")
        v = np.asarray(mesh.vertices)
        p = np.asarray(pcd.points)
        if len(v) > 0 and len(p) > 0:
            tree = cKDTree(p)
            d, _ = tree.query(v, k=1)
            mesh.remove_vertices_by_mask(d > float(max_vertex_distance))

    # Ensure vertex colors exist if pcd has colors
    if pcd.has_colors() and not mesh.has_vertex_colors():
        logging.info("Mesh has no vertex colors; painting from nearest point colors...")
        p = np.asarray(pcd.points)
        c = np.asarray(pcd.colors)
        if len(p) > 0 and len(mesh.vertices) > 0:
            tree = cKDTree(p)
            v = np.asarray(mesh.vertices)
            _, idx = tree.query(v, k=1)
            mesh.vertex_colors = o3d.utility.Vector3dVector(c[idx])

    # Cleanup
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()

    logging.info(f"Final mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} faces")
    return mesh


# ----------------------------
# Main pipeline
# ----------------------------
def process_bag(args):
    bag_path = Path(args.bagpath)
    out_dir = Path(args.outputdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not bag_path.exists():
        sys.exit(f"Error: Bag file not found: {bag_path}")

    pc_topic = args.pc_topic
    camera_topic = args.camera_topic
    odom_topic = args.odom_topic
    camera_info_topic = args.camera_info_topic

    logging.info(f"Reading bag: {bag_path}")
    logging.info(f"Output dir: {out_dir}")
    logging.info(f"PointCloud2 topic: {pc_topic}")
    if camera_topic:
        logging.info(f"Camera topic: {camera_topic}")
    if camera_info_topic:
        logging.info(f"Camera info topic: {camera_info_topic}")
    if odom_topic:
        logging.info(f"Odometry topic: {odom_topic}")
    if args.enable_loop_closure:
        logging.info("Loop closure: ENABLED")
    else:
        logging.info("Loop closure: disabled")

    # 1) Extract data
    point_clouds = []        # [(ts, pcd)]
    odom_data = {}           # ts -> 4x4
    camera_images = {}       # ts -> PIL Image
    camera_info_msgs = {}    # ts -> (fx, fy, cx, cy, w, h)

    topics_to_read = [pc_topic]
    if odom_topic:
        topics_to_read.append(odom_topic)
    if camera_topic:
        topics_to_read.append(camera_topic)
    if camera_info_topic:
        topics_to_read.append(camera_info_topic)

    with AnyReader([bag_path], default_typestore=TYPESTORE) as reader:
        conns = [c for c in reader.connections if c.topic in topics_to_read]
        if not conns:
            sys.exit(f"Error: No messages found for topics: {topics_to_read}")

        for conn, ts, raw in tqdm(reader.messages(connections=conns), desc="Reading messages"):
            try:
                msg = reader.deserialize(raw, conn.msgtype)

                if conn.topic == pc_topic:
                    pcd = convert_ros_pc2_to_o3d(msg)
                    if pcd is not None and len(pcd.points) >= 100:
                        point_clouds.append((ts, pcd))

                elif odom_topic and conn.topic == odom_topic:
                    T = get_odom_transform(msg)
                    if T is not None:
                        odom_data[ts] = T

                elif camera_topic and conn.topic == camera_topic:
                    if "CompressedImage" in conn.msgtype:
                        img = convert_compressed_image(msg)
                    else:
                        enc = getattr(msg, "encoding", "rgb8")
                        img = convert_ros_image(msg, enc)
                    if img is not None:
                        camera_images[ts] = img
                        
                elif camera_info_topic and conn.topic == camera_info_topic:
                    camera_info_msgs[ts] = intrinsics_from_camera_info(msg)

            except Exception:
                continue

    # Fill intrinsics from CameraInfo if not provided
    if camera_topic:
        have_cli_intrinsics = all(v is not None for v in [args.camera_fx, args.camera_fy, args.camera_cx, args.camera_cy])
        
        if not have_cli_intrinsics:
            if not camera_info_msgs:
                sys.exit(
                    f"Error: camera intrinsics not provided and no CameraInfo messages found on '{camera_info_topic}'. "
                    "Provide --camera_fx/fy/cx/cy or set --camera_info_topic."
                )
            
            # Intrinsics are usually constant; pick the earliest CameraInfo
            first_ts = min(camera_info_msgs.keys())
            fx, fy, cx, cy, w, h = camera_info_msgs[first_ts]
            
            args.camera_fx = fx
            args.camera_fy = fy
            args.camera_cx = cx
            args.camera_cy = cy
            args.camera_width = args.camera_width or w
            args.camera_height = args.camera_height or h
            
            logging.info(f"Loaded camera intrinsics from {camera_info_topic}: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        else:
            # Optionally fill missing width/height
            if (args.camera_width is None or args.camera_height is None) and camera_info_msgs:
                first_ts = min(camera_info_msgs.keys())
                _, _, _, _, w, h = camera_info_msgs[first_ts]
                args.camera_width = args.camera_width or w
                args.camera_height = args.camera_height or h

    if not point_clouds:
        sys.exit("Error: No valid point clouds were extracted.")

    logging.info(f"Extracted {len(point_clouds)} point clouds")
    if odom_topic:
        logging.info(f"Extracted {len(odom_data)} odometry messages")
    if camera_topic:
        logging.info(f"Extracted {len(camera_images)} camera images")

    # 2) Pairwise registration + pose graph
    pose_graph = o3d.pipelines.registration.PoseGraph()
    current_transform = np.eye(4, dtype=np.float64)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(current_transform.copy()))

    # First frame
    _ts0, src_raw = point_clouds[0]
    src = src_raw.voxel_down_sample(args.voxel_size)
    src.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel_size * 2.0, max_nn=30)
    )

    src_fpfh = None
    if args.enable_loop_closure and (0 % args.loop_closure_search_interval == 0):
        src_fpfh = compute_fpfh_descriptor(src, args.voxel_size)

    accumulated_pcds = [src]
    accumulated_fpfhs = [src_fpfh]
    accumulated_poses = [current_transform.copy()]

    previous_odom_T = None
    if odom_topic:
        closest_ts = get_closest_timestamp(_ts0, odom_data)
        if closest_ts is not None:
            previous_odom_T = odom_data[closest_ts]

    successful_pc_indices = [0]
    loop_closures_found = 0

    logging.info("Registering point clouds...")
    for i in tqdm(range(1, len(point_clouds)), desc="Registering"):
        ts, tgt_raw = point_clouds[i]
        tgt = tgt_raw.voxel_down_sample(args.voxel_size)
        tgt.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel_size * 2.0, max_nn=30)
        )

        initial_guess = np.eye(4, dtype=np.float64)
        if odom_topic:
            closest_ts = get_closest_timestamp(ts, odom_data)
            if closest_ts is not None:
                current_odom_T = odom_data[closest_ts]
                if previous_odom_T is not None:
                    initial_guess = np.linalg.inv(previous_odom_T) @ current_odom_T
                previous_odom_T = current_odom_T

        try:
            reg = o3d.pipelines.registration.registration_icp(
                src,
                tgt,
                args.icp_dist_thresh,
                initial_guess,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
            )
        except Exception:
            continue

        if float(reg.fitness) < float(args.icp_fitness_thresh):
            continue

        current_transform = reg.transformation @ current_transform

        # PoseGraphNode stores the inverse pose (Open3D convention in many examples)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(current_transform)))

        info = np.eye(6, dtype=np.float64) * max(float(reg.fitness), 1e-6)
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                len(pose_graph.nodes) - 2,
                len(pose_graph.nodes) - 1,
                reg.transformation,
                info,
                uncertain=False,
            )
        )

        accumulated_pcds.append(tgt)
        accumulated_poses.append(current_transform.copy())

        do_lc = args.enable_loop_closure and (i % args.loop_closure_search_interval == 0)
        tgt_fpfh = None
        if do_lc:
            tgt_fpfh = compute_fpfh_descriptor(tgt, args.voxel_size)
        accumulated_fpfhs.append(tgt_fpfh)

        if do_lc:
            lcs = detect_loop_closure(
                current_idx=len(accumulated_pcds) - 1,
                current_pcd=tgt,
                current_fpfh=tgt_fpfh,
                historical_pcds=accumulated_pcds,
                historical_fpfhs=accumulated_fpfhs,
                historical_poses=accumulated_poses,
                voxel_size=args.voxel_size,
                search_radius=args.loop_closure_radius,
                loop_fitness_thresh=args.loop_closure_fitness_thresh,
                temporal_window=100,
            )
            for cand_idx, lc_T, lc_fit in lcs:
                lc_info = np.eye(6, dtype=np.float64) * (max(float(lc_fit), 1e-6) * 100.0)
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        cand_idx,
                        len(pose_graph.nodes) - 1,
                        lc_T,
                        lc_info,
                        uncertain=True,
                    )
                )
                loop_closures_found += 1

        successful_pc_indices.append(i)
        src = tgt

    if len(pose_graph.nodes) < 2:
        sys.exit("Error: Registration failed (too few successful registrations).")

    if args.enable_loop_closure:
        logging.info(f"Loop closures detected: {loop_closures_found}")

    # 3) Global pose graph optimization
    logging.info("Optimizing pose graph...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=args.icp_dist_thresh,
        edge_prune_threshold=0.25,
        reference_node=0,
    )
    try:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option,
        )
    except Exception as e:
        logging.warning(f"Warning: Global optimization failed: {e}")

    # Helper: get pose used for transforming raw clouds (this matches your original usage)
    def node_pose(node_idx: int) -> np.ndarray:
        return np.asarray(pose_graph.nodes[node_idx].pose, dtype=np.float64)

    # 4) Color or grayscale mode
    colored_pcds = []
    have_color_mode = bool(camera_topic and camera_images and args.camera_fx is not None)

    if have_color_mode:
        logging.info("--- Coloring point clouds ---")

        intrinsics = {
            "fx": float(args.camera_fx),
            "fy": float(args.camera_fy),
            "cx": float(args.camera_cx),
            "cy": float(args.camera_cy),
            "width": int(args.camera_width),
            "height": int(args.camera_height),
        }
        cam_ts_sorted = sorted(camera_images.keys())

        for node_idx, pc_idx in tqdm(
            list(enumerate(successful_pc_indices)),
            desc="Coloring",
            total=len(successful_pc_indices),
        ):
            pc_ts, pc_raw = point_clouds[pc_idx]
            T = node_pose(node_idx)

            pcd_world = copy.deepcopy(pc_raw)
            pcd_world.transform(T)

            # Attach view rays (stored as normals) BEFORE any downsampling
            sensor_origin = T[:3, 3].copy()
            attach_view_rays_as_normals(pcd_world, sensor_origin)

            # Find closest camera frame
            closest_cam_ts = min(cam_ts_sorted, key=lambda t: abs(t - pc_ts))
            time_diff_s = abs(closest_cam_ts - pc_ts) / 1e9
            if time_diff_s <= float(args.max_time_diff):
                img = camera_images[closest_cam_ts]
                pcd_world = color_point_cloud_from_image(pcd_world, img, T, intrinsics)
            else:
                # no close image -> gray
                pcd_world.colors = o3d.utility.Vector3dVector(
                    np.full((len(pcd_world.points), 3), 0.5, dtype=np.float64)
                )

            colored_pcds.append(pcd_world)

        logging.info("--- Merging colored point clouds ---")
        merged = merge_colored_point_clouds(
            colored_pcds,
            voxel_size=float(args.voxel_size),
            gray_filter_radius=float(args.gray_filter_radius),
        )

        # Clean (ROR/SOR/DBSCAN) WITHOUT another voxel stage (already downsampled by merge)
        cleaned = clean_point_cloud(merged, float(args.voxel_size), do_voxel_downsample=False)

    else:
        logging.info("--- Generating point cloud + mesh (no color data) ---")
        pcd_combined = o3d.geometry.PointCloud()

        for node_idx, pc_idx in tqdm(
            list(enumerate(successful_pc_indices)),
            desc="Merging clouds",
            total=len(successful_pc_indices),
        ):
            _pc_ts, pc_raw = point_clouds[pc_idx]
            T = node_pose(node_idx)

            pcd_world = copy.deepcopy(pc_raw)
            pcd_world.transform(T)

            sensor_origin = T[:3, 3].copy()
            attach_view_rays_as_normals(pcd_world, sensor_origin)

            pcd_combined += pcd_world

        if len(pcd_combined.points) == 0:
            sys.exit("Error: Combined point cloud is empty.")

        # Optional floor leveling (apply rotation to both points AND view-ray normals)
        if args.level_floor:
            logging.info("Attempting to level the floor...")
            try:
                pcd_tmp = pcd_combined.voxel_down_sample(float(args.voxel_size) * 2.0)
                plane_model, _inliers = pcd_tmp.segment_plane(
                    distance_threshold=float(args.voxel_size) * 2.0,
                    ransac_n=3,
                    num_iterations=1000,
                )
                a, b, c, _d = plane_model
                n = np.array([a, b, c], dtype=np.float64)
                n = n / (np.linalg.norm(n) + 1e-12)

                target_n = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                if np.dot(n, target_n) < 0:
                    n = -n

                v = np.cross(n, target_n)
                s = np.linalg.norm(v)

                if s > 1e-12:
                    cang = float(np.dot(n, target_n))
                    vx = np.array(
                        [[0.0, -v[2], v[1]],
                         [v[2], 0.0, -v[0]],
                         [-v[1], v[0], 0.0]],
                        dtype=np.float64,
                    )
                    R3 = np.eye(3, dtype=np.float64) + vx + (vx @ vx) * ((1.0 - cang) / (s * s))

                    pts = np.asarray(pcd_combined.points, dtype=np.float64)
                    pts = pts @ R3.T
                    pcd_combined.points = o3d.utility.Vector3dVector(pts)

                    if pcd_combined.has_normals():
                        nr = np.asarray(pcd_combined.normals, dtype=np.float64)
                        nr = nr @ R3.T
                        pcd_combined.normals = o3d.utility.Vector3dVector(nr)

                    logging.info("✓ Floor leveling applied.")
                else:
                    logging.info("✓ Map is already level.")
            except Exception as e:
                logging.warning(f"Warning: Floor leveling failed: {e}")

        cleaned = clean_point_cloud(pcd_combined, float(args.voxel_size), do_voxel_downsample=True)

    # Save point cloud (keep view rays in normals for debugging/orientation traceability)
    ply_path = out_dir / f"{bag_path.stem}_cloud.ply"
    o3d.io.write_point_cloud(str(ply_path), cleaned)
    logging.info(f"Saved point cloud: {ply_path}")

    # Extract view rays before overwriting normals with geometric normals
    view_rays = None
    if cleaned.has_normals():
        view_rays = np.asarray(cleaned.normals, dtype=np.float64).copy()

    # Estimate + orient geometric normals for Poisson
    logging.info("Estimating geometric normals for meshing (and orienting with view rays when available)...")
    estimate_geometric_normals_oriented(cleaned, float(args.voxel_size), view_rays)

    # Mesh
    mesh = create_colored_mesh(
        cleaned,
        depth=int(args.poisson_depth),
        min_density_percentile=float(args.min_density_percentile),
        max_vertex_distance=float(args.max_vertex_distance),
    )

    mesh_ply_path = out_dir / f"{bag_path.stem}_mesh.ply"
    o3d.io.write_triangle_mesh(str(mesh_ply_path), mesh)

    logging.info(f"Saved mesh: {mesh_ply_path}")
    logging.info("Done.")


def main():
    p = argparse.ArgumentParser(description="Convert ROS 2 bag to (optional colored) point cloud + Poisson mesh.")

    # Positional
    p.add_argument("bagpath", help="Path to the ROS 2 bag file.")
    p.add_argument("outputdir", help="Directory to save outputs.")

    # Topic args (add aliases so older README/usage still works)
    p.add_argument("--pctopic", "--pc_topic", dest="pc_topic", default="points", help="PointCloud2 topic name.")
    p.add_argument("--cameratopic", "--camera_topic", dest="camera_topic", default=None, help="Camera topic (Image or CompressedImage).")
    p.add_argument("--camerainfotopic", "--camera_info_topic", dest="camera_info_topic", default=None, help="CameraInfo topic (sensor_msgs/CameraInfo).")
    p.add_argument("--odomtopic", "--odom_topic", dest="odom_topic", default=None, help="Odometry topic (nav_msgs/Odometry).")

    # Camera intrinsics
    p.add_argument("--camerafx", "--camera_fx", dest="camera_fx", type=float, default=None, help="Camera fx (pixels).")
    p.add_argument("--camerafy", "--camera_fy", dest="camera_fy", type=float, default=None, help="Camera fy (pixels).")
    p.add_argument("--cameracx", "--camera_cx", dest="camera_cx", type=float, default=None, help="Camera cx (pixels).")
    p.add_argument("--cameracy", "--camera_cy", dest="camera_cy", type=float, default=None, help="Camera cy (pixels).")
    p.add_argument("--camerawidth", "--camera_width", dest="camera_width", type=int, default=None, help="Camera image width (pixels).")
    p.add_argument("--cameraheight", "--camera_height", dest="camera_height", type=int, default=None, help="Camera image height (pixels).")
    p.add_argument("--maxtimediff", "--max_time_diff", dest="max_time_diff", type=float, default=0.1, help="Max cam-lidar timestamp difference (seconds).")

    # Pipeline params
    p.add_argument("--voxelsize", "--voxel_size", dest="voxel_size", type=float, default=0.05, help="Voxel size (meters).")
    p.add_argument("--icpdistthresh", "--icp_dist_thresh", dest="icp_dist_thresh", type=float, default=0.2, help="ICP max correspondence distance (meters).")
    p.add_argument("--icpfitnessthresh", "--icp_fitness_thresh", dest="icp_fitness_thresh", type=float, default=0.6, help="Min ICP fitness to accept a frame.")

    # Loop closure
    p.add_argument("--enableloopclosure", "--enable_loop_closure", dest="enable_loop_closure", action="store_true", default=False, help="Enable loop closure.")
    p.add_argument("--loopclosureradius", "--loop_closure_radius", dest="loop_closure_radius", type=float, default=10.0, help="Loop closure search radius (m).")
    p.add_argument("--loopclosurefitnessthresh", "--loop_closure_fitness_thresh", dest="loop_closure_fitness_thresh", type=float, default=0.3, help="Loop closure fitness threshold.")
    p.add_argument("--loopclosuresearchinterval", "--loop_closure_search_interval", dest="loop_closure_search_interval", type=int, default=10, help="Loop closure search every N frames.")

    # Color merge params
    p.add_argument("--grayfilterradius", "--gray_filter_radius", dest="gray_filter_radius", type=float, default=0.05, help="Remove gray points within this distance of colored points (m).")

    # Mesh params
    p.add_argument("--poissondepth", "--poisson_depth", dest="poisson_depth", type=int, default=9, help="Poisson depth.")
    p.add_argument("--mindensitypercentile", "--min_density_percentile", dest="min_density_percentile", type=float, default=1.0, help="Trim bottom density percentile.")
    p.add_argument("--maxvertexdistance", "--max_vertex_distance", dest="max_vertex_distance", type=float, default=0.15, help="Trim mesh vertices farther than this from points (m).")

    # Other
    p.add_argument("--levelfloor", "--level_floor", dest="level_floor", action="store_true", help="Attempt floor leveling.")

    args = p.parse_args()
    process_bag(args)

if __name__ == "__main__":
    main()

