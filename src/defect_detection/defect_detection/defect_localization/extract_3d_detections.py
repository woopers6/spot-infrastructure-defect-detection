
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Optional

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


@dataclass
class BBox3D:

    xyz_min: np.ndarray  # shape: (3,)
    xyz_max: np.ndarray  # shape: (3,)

    @property
    def center(self) -> np.ndarray:
        return (self.xyz_min + self.xyz_max) / 2.0

    @property
    def size(self) -> np.ndarray:
        return self.xyz_max - self.xyz_min


@dataclass
class CustomDetection3D:

    class_name: str
    class_id: Optional[int]
    confidence: float
    bbox_2d: tuple[int, int, int, int]

    points_lidar: np.ndarray   # shape: (N, 3), original LiDAR frame
    points_camera: np.ndarray  # shape: (N, 3), transformed camera frame

    centroid_lidar: np.ndarray
    median_depth_camera: float
    bbox_3d_lidar: Optional[BBox3D]

    num_points: int


def pointcloud2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points_numpy(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )

    points = np.asarray(points, dtype=np.float64)

    if points.ndim != 2 or points.shape[1] != 3:
        points = points.reshape(-1, 3)

    # Drop all zero points
    points = points[~np.all(points == 0.0, axis=1)]

    return points


def transform_points(points_xyz: np.ndarray, transform_4x4: np.ndarray) -> np.ndarray:
    if points_xyz.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    if transform_4x4.shape != (4, 4):
        raise ValueError(f"Expected 4x4 transform, got shape {transform_4x4.shape}")

    points_h = np.hstack(
        [points_xyz, np.ones((points_xyz.shape[0], 1), dtype=np.float64)]
    )

    transformed = (transform_4x4 @ points_h.T).T[:, :3]
    return transformed


def project_camera_points_to_pixels(
    points_camera: np.ndarray,
    intrinsics_3x3: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if intrinsics_3x3.shape != (3, 3):
        raise ValueError(f"Expected 3x3 intrinsics, got shape {intrinsics_3x3.shape}")

    if points_camera.size == 0:
        return np.array([]), np.array([])

    projected = (intrinsics_3x3 @ points_camera.T).T

    z = projected[:, 2]
    u = projected[:, 0] / z
    v = projected[:, 1] / z

    return u, v


def compute_3d_bbox(points_lidar: np.ndarray) -> Optional[BBox3D]:
    if points_lidar.size == 0 or len(points_lidar) == 0:
        return None

    xyz_min = np.min(points_lidar, axis=0)
    xyz_max = np.max(points_lidar, axis=0)

    return BBox3D(xyz_min=xyz_min, xyz_max=xyz_max)


def filter_depth_outliers(
    points_lidar: np.ndarray,
    points_camera: np.ndarray,
    max_depth_deviation: float = 0.50,
    depth_cluster_tolerance: float = 0.20,
    min_cluster_points: int = 3,
) -> tuple[np.ndarray, np.ndarray]:
    if len(points_camera) == 0:
        return points_lidar, points_camera

    depths = points_camera[:, 2]
    sorted_indices = np.argsort(depths)
    sorted_depths = depths[sorted_indices]

    split_indices = np.flatnonzero(
        np.diff(sorted_depths) > depth_cluster_tolerance
    ) + 1
    depth_clusters = np.split(sorted_indices, split_indices)
    valid_clusters = [
        cluster
        for cluster in depth_clusters
        if len(cluster) >= min_cluster_points
    ]

    if not valid_clusters:
        return (
            np.empty((0, 3), dtype=points_lidar.dtype),
            np.empty((0, 3), dtype=points_camera.dtype),
        )

    closest_cluster = min(
        valid_clusters,
        key=lambda cluster: float(np.median(depths[cluster])),
    )
    cluster_median_depth = np.median(depths[closest_cluster])
    keep_indices = closest_cluster[
        np.abs(depths[closest_cluster] - cluster_median_depth)
        <= max_depth_deviation
    ]

    return points_lidar[keep_indices], points_camera[keep_indices]


def extract_xyxy_from_yolo_box(box: Any) -> tuple[int, int, int, int]:
    # vision_msgs/Detection2D
    if hasattr(box, "bbox") and hasattr(box.bbox, "center"):
        cx = box.bbox.center.position.x
        cy = box.bbox.center.position.y
        width = box.bbox.size_x
        height = box.bbox.size_y

        x1 = int(cx - width / 2.0)
        y1 = int(cy - height / 2.0)
        x2 = int(cx + width / 2.0)
        y2 = int(cy + height / 2.0)

        return x1, y1, x2, y2

    # Ultralytics YOLO box
    if hasattr(box, "xyxy"):
        xyxy = box.xyxy

        if hasattr(xyxy, "detach"):
            xyxy = xyxy.detach().cpu().numpy()
        else:
            xyxy = np.asarray(xyxy)

        xyxy = xyxy.reshape(-1, 4)[0]
        return tuple(map(int, xyxy.tolist()))

    # raw array/list
    xyxy = np.asarray(box).reshape(-1)
    if xyxy.shape[0] < 4:
        raise ValueError("Bounding box must contain at least 4 values: x1, y1, x2, y2")

    return tuple(map(int, xyxy[:4].tolist()))


def extract_confidence_from_yolo_box(box: Any) -> float:
    # vision_msgs/Detection2D
    if hasattr(box, "results") and len(box.results) > 0:
        return float(box.results[0].hypothesis.score)

    # Ultralytics YOLO box
    if not hasattr(box, "conf"):
        return 0.0

    conf = box.conf
    if hasattr(conf, "detach"):
        conf = conf.detach().cpu().numpy()
    else:
        conf = np.asarray(conf)

    return float(conf.reshape(-1)[0])


def extract_class_id_from_yolo_box(box: Any) -> Optional[int]:
    # vision_msgs/Detection2D
    if hasattr(box, "results") and len(box.results) > 0:
        class_id = box.results[0].hypothesis.class_id
        try:
            return int(class_id)
        except ValueError:
            return None

    # Ultralytics YOLO box
    if not hasattr(box, "cls"):
        return None

    cls = box.cls
    if hasattr(cls, "detach"):
        cls = cls.detach().cpu().numpy()
    else:
        cls = np.asarray(cls)

    return int(cls.reshape(-1)[0])


def get_class_name(class_id: Optional[int], class_names: Optional[list[str]]) -> str:
    if class_id is None:
        return "unknown"

    if class_names is None:
        return str(class_id)

    if 0 <= class_id < len(class_names):
        return class_names[class_id]

    return str(class_id)


def points_in_2d_bbox(
    u: np.ndarray,
    v: np.ndarray,
    bbox_2d: tuple[int, int, int, int],
    image_shape: Optional[tuple[int, int]] = None,
) -> np.ndarray:
    x1, y1, x2, y2 = bbox_2d

    mask = (
        (u >= x1) & (u <= x2) &
        (v >= y1) & (v <= y2)
    )

    if image_shape is not None:
        height, width = image_shape
        mask &= (
            (u >= 0) & (u < width) &
            (v >= 0) & (v < height)
        )

    return mask

def extract_xyxy_from_detection2d(detection) -> tuple[int, int, int, int]:
    cx = detection.bbox.center.position.x
    cy = detection.bbox.center.position.y
    width = detection.bbox.size_x
    height = detection.bbox.size_y

    x1 = int(cx - width / 2.0)
    y1 = int(cy - height / 2.0)
    x2 = int(cx + width / 2.0)
    y2 = int(cy + height / 2.0)

    return x1, y1, x2, y2


def extract_detection_3d(
    pointcloud_msg: PointCloud2,
    intrinsics_3x3: np.ndarray,
    T_lidar_to_camera: np.ndarray,
    box: Any,
    class_names: Optional[list[str]] = None,
    image_shape: Optional[tuple[int, int]] = None,
    filter_outliers: bool = True,
    max_depth_deviation: float = 0.50,
    depth_cluster_tolerance: float = 0.20,
    min_points: int = 3,
) -> Optional[CustomDetection3D]:
    pc_lidar = pointcloud2_to_xyz_array(pointcloud_msg)

    if len(pc_lidar) == 0:
        return None

    pc_camera = transform_points(pc_lidar, T_lidar_to_camera)

    # Keep only LiDAR points in front of the camera
    front_mask = pc_camera[:, 2] > 0.0

    pc_lidar_front = pc_lidar[front_mask]
    pc_camera_front = pc_camera[front_mask]

    if len(pc_lidar_front) == 0:
        return None

    u, v = project_camera_points_to_pixels(pc_camera_front, intrinsics_3x3)

    bbox_2d = extract_xyxy_from_yolo_box(box)
    bbox_mask = points_in_2d_bbox(u, v, bbox_2d, image_shape=image_shape)

    defect_points_lidar = pc_lidar_front[bbox_mask]
    defect_points_camera = pc_camera_front[bbox_mask]

    if filter_outliers:
        defect_points_lidar, defect_points_camera = filter_depth_outliers(
            defect_points_lidar,
            defect_points_camera,
            max_depth_deviation=max_depth_deviation,
            depth_cluster_tolerance=depth_cluster_tolerance,
            min_cluster_points=min_points,
        )

    if len(defect_points_lidar) < min_points:
        return None

    class_id = extract_class_id_from_yolo_box(box)
    class_name = get_class_name(class_id, class_names)
    confidence = extract_confidence_from_yolo_box(box)

    centroid_lidar = np.mean(defect_points_lidar, axis=0)
    median_depth_camera = float(np.median(defect_points_camera[:, 2]))
    bbox_3d_lidar = compute_3d_bbox(defect_points_lidar)

    return CustomDetection3D(
        class_name=class_name,
        class_id=class_id,
        confidence=confidence,
        bbox_2d=bbox_2d,
        points_lidar=defect_points_lidar,
        points_camera=defect_points_camera,
        centroid_lidar=centroid_lidar,
        median_depth_camera=median_depth_camera,
        bbox_3d_lidar=bbox_3d_lidar,
        num_points=len(defect_points_lidar),
    )


def extract_detections_3d(
    pointcloud_msg: PointCloud2,
    intrinsics_3x3: np.ndarray,
    T_lidar_to_camera: np.ndarray,
    boxes: Iterable[Any],
    class_names: Optional[list[str]] = None,
    image_shape: Optional[tuple[int, int]] = None,
    filter_outliers: bool = True,
    max_depth_deviation: float = 0.50,
    depth_cluster_tolerance: float = 0.20,
    min_points: int = 3,
) -> list[CustomDetection3D]:
    detections: list[CustomDetection3D] = []

    for box in boxes:
        det = extract_detection_3d(
            pointcloud_msg=pointcloud_msg,
            intrinsics_3x3=intrinsics_3x3,
            T_lidar_to_camera=T_lidar_to_camera,
            box=box,
            class_names=class_names,
            image_shape=image_shape,
            filter_outliers=filter_outliers,
            max_depth_deviation=max_depth_deviation,
            depth_cluster_tolerance=depth_cluster_tolerance,
            min_points=min_points,
        )

        if det is not None:
            detections.append(det)

    return detections


# Example usage
#
# results = yolo_model(image)
# boxes = results[0].boxes
#
# detections_3d = extract_detections_3d(
#     pointcloud_msg=cloud_msg,
#     intrinsics_3x3=K,
#     T_lidar_to_camera=T_lidar_to_camera,
#     boxes=boxes,
#     class_names=class_names,
#     image_shape=(image_height, image_width),
#     filter_outliers=True,
#     max_depth_deviation=0.50,
#     depth_cluster_tolerance=0.20,
#     min_points=3,
# )
#
# for det in detections_3d:
#     print(det.class_name, det.confidence, det.centroid_lidar, det.median_depth_camera)
