import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

from defect_detection.defect_detection.fusion_node import DetectionFusionNode
from defect_detection.defect_localization.extract_3d_detections import (
    extract_detections_3d,
)


def make_detection():
    detection = Detection2D()
    detection.bbox.center.position.x = 320.0
    detection.bbox.center.position.y = 240.0
    detection.bbox.size_x = 40.0
    detection.bbox.size_y = 40.0

    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = '1'
    result.hypothesis.score = 0.87
    detection.results.append(result)
    return detection


def test_synthetic_2d_lidar_fusion_pipeline():
    intrinsics = np.array([
        [100.0, 0.0, 320.0],
        [0.0, 100.0, 240.0],
        [0.0, 0.0, 1.0],
    ])
    lidar_to_camera = np.eye(4)
    lidar_to_camera[0, 3] = 1.0

    foreground_camera = np.array([
        [-0.40, -0.40, 4.00],
        [0.40, -0.40, 4.05],
        [-0.40, 0.40, 4.10],
        [0.40, 0.40, 4.15],
    ])
    background_camera = np.array([
        [-0.80, -0.80, 8.00],
        [0.80, -0.80, 8.05],
        [-0.80, 0.80, 8.10],
        [0.80, 0.80, 8.15],
        [0.00, 0.00, 8.20],
    ])
    sparse_near_noise_camera = np.array([[0.0, 0.0, 1.0]])
    outside_box_camera = np.array([[4.0, 0.0, 4.0]])
    behind_camera = np.array([[0.0, 0.0, -2.0]])

    camera_points = np.vstack([
        background_camera,
        sparse_near_noise_camera,
        outside_box_camera,
        behind_camera,
        foreground_camera,
    ])
    lidar_points = camera_points.copy()
    lidar_points[:, 0] -= 1.0

    header = Header()
    header.frame_id = 'lidar'
    cloud = point_cloud2.create_cloud_xyz32(header, lidar_points)

    detections = extract_detections_3d(
        pointcloud_msg=cloud,
        intrinsics_3x3=intrinsics,
        T_lidar_to_camera=lidar_to_camera,
        boxes=[make_detection()],
        class_names=['spalling', 'crack'],
        image_shape=(480, 640),
        filter_outliers=True,
        max_depth_deviation=0.50,
        depth_cluster_tolerance=0.20,
        min_points=3,
    )

    assert len(detections) == 1
    detection = detections[0]
    expected_lidar_points = foreground_camera.copy()
    expected_lidar_points[:, 0] -= 1.0

    assert detection.class_id == 1
    assert detection.class_name == 'crack'
    assert detection.confidence == 0.87
    assert detection.bbox_2d == (300, 220, 340, 260)
    assert detection.num_points == 4
    np.testing.assert_allclose(
        np.sort(detection.points_lidar, axis=0),
        np.sort(expected_lidar_points, axis=0),
        atol=1e-6,
    )
    np.testing.assert_allclose(
        detection.centroid_lidar,
        np.mean(expected_lidar_points, axis=0),
        atol=1e-6,
    )
    assert np.isclose(
        detection.median_depth_camera,
        np.median(foreground_camera[:, 2]),
        atol=1e-6,
    )
    np.testing.assert_allclose(
        detection.bbox_3d_lidar.xyz_min,
        np.min(expected_lidar_points, axis=0),
        atol=1e-6,
    )
    np.testing.assert_allclose(
        detection.bbox_3d_lidar.xyz_max,
        np.max(expected_lidar_points, axis=0),
        atol=1e-6,
    )

    output = DetectionFusionNode.convert_to_ros_detection3d(None, detection)

    assert output.results[0].hypothesis.class_id == '1'
    assert output.results[0].hypothesis.score == 0.87
    np.testing.assert_allclose([
        output.results[0].pose.pose.position.x,
        output.results[0].pose.pose.position.y,
        output.results[0].pose.pose.position.z,
    ], detection.centroid_lidar)
    np.testing.assert_allclose([
        output.bbox.size.x,
        output.bbox.size.y,
        output.bbox.size.z,
    ], detection.bbox_3d_lidar.size)
