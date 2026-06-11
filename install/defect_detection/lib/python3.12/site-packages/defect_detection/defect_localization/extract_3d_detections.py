
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
class Detection3D:

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
) -> tuple[np.ndarray, np.ndarray]:
    if len(points_camera) == 0:
        return points_lidar, points_camera

    depths = points_camera[:, 2]
    median_depth = np.median(depths)

    keep = np.abs(depths - median_depth) <= max_depth_deviation

    return points_lidar[keep], points_camera[keep]


def extract_xyxy_from_yolo_box(box: Any) -> tuple[int, int, int, int]:
    if hasattr(box, "xyxy"):
        xyxy = box.xyxy

        # Ultralytics often stores xyxy as shape (1, 4)
        if hasattr(xyxy, "detach"):
            xyxy = xyxy.detach().cpu().numpy()
        else:
            xyxy = np.asarray(xyxy)

        xyxy = xyxy.reshape(-1, 4)[0]
        return tuple(map(int, xyxy.tolist()))

    xyxy = np.asarray(box).reshape(-1)
    if xyxy.shape[0] < 4:
        raise ValueError("Bounding box must contain at least 4 values: x1, y1, x2, y2")

    return tuple(map(int, xyxy[:4].tolist()))


def extract_confidence_from_yolo_box(box: Any) -> float:
    if not hasattr(box, "conf"):
        return 0.0

    conf = box.conf
    if hasattr(conf, "detach"):
        conf = conf.detach().cpu().numpy()
    else:
        conf = np.asarray(conf)

    return float(conf.reshape(-1)[0])


def extract_class_id_from_yolo_box(box: Any) -> Optional[int]:
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


def extract_detection_3d(
    pointcloud_msg: PointCloud2,
    intrinsics_3x3: np.ndarray,
    T_lidar_to_camera: np.ndarray,
    box: Any,
    class_names: Optional[list[str]] = None,
    image_shape: Optional[tuple[int, int]] = None,
    filter_outliers: bool = True,
    max_depth_deviation: float = 0.50,
    min_points: int = 3,
) -> Optional[Detection3D]:
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
        )

    if len(defect_points_lidar) < min_points:
        return None

    class_id = extract_class_id_from_yolo_box(box)
    class_name = get_class_name(class_id, class_names)
    confidence = extract_confidence_from_yolo_box(box)

    centroid_lidar = np.mean(defect_points_lidar, axis=0)
    median_depth_camera = float(np.median(defect_points_camera[:, 2]))
    bbox_3d_lidar = compute_3d_bbox(defect_points_lidar)

    return Detection3D(
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
    min_points: int = 3,
) -> list[Detection3D]:
    detections: list[Detection3D] = []

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
#     min_points=3,
# )
#
# for det in detections_3d:
#     print(det.class_name, det.confidence, det.centroid_lidar, det.median_depth_camera)
