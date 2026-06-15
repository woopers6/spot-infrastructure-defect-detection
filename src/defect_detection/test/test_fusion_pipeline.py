from defect_detection.defect_detection.fusion_node import DetectionFusionNode
from defect_detection.defect_detection.fusion_node import (
    load_calibration,
    timestamp_delta_seconds,
)
from defect_detection.defect_localization.extract_3d_detections import (
    extract_detections_3d,
)
import message_filters
import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
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


def test_timestamp_delta_uses_acquisition_stamps():
    detection = make_detection()
    cloud_header = Header()

    detection.header.stamp.sec = 100
    detection.header.stamp.nanosec = 20_000_000
    cloud_header.stamp.sec = 100
    cloud_header.stamp.nanosec = 75_000_000

    assert timestamp_delta_seconds(
        detection.header.stamp,
        cloud_header.stamp,
    ) == pytest.approx(0.055)


def test_timestamp_delta_rejects_missing_stamp():
    detection = make_detection()
    cloud_header = Header()
    cloud_header.stamp.sec = 1

    with pytest.raises(ValueError, match='non-zero'):
        timestamp_delta_seconds(
            detection.header.stamp,
            cloud_header.stamp,
        )


def test_load_calibration_rejects_uncalibrated_file(tmp_path):
    calibration = tmp_path / 'calibration.yaml'
    calibration.write_text(
        'calibrated: false\n',
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='not marked calibrated'):
        load_calibration(calibration)


def test_load_calibration(tmp_path):
    calibration = tmp_path / 'calibration.yaml'
    calibration.write_text(
        '\n'.join([
            'calibrated: true',
            'image_width: 640',
            'image_height: 480',
            'camera_matrix:',
            '  - [100.0, 0.0, 320.0]',
            '  - [0.0, 100.0, 240.0]',
            '  - [0.0, 0.0, 1.0]',
            'lidar_to_camera:',
            '  - [1.0, 0.0, 0.0, 0.0]',
            '  - [0.0, 1.0, 0.0, 0.0]',
            '  - [0.0, 0.0, 1.0, 0.0]',
            '  - [0.0, 0.0, 0.0, 1.0]',
        ]),
        encoding='utf-8',
    )

    intrinsics, transform, image_shape = load_calibration(calibration)

    assert intrinsics.shape == (3, 3)
    assert transform.shape == (4, 4)
    assert image_shape == (480, 640)


def test_ros_approximate_synchronizer_respects_tolerance():
    detections_filter = message_filters.SimpleFilter()
    cloud_filter = message_filters.SimpleFilter()
    synchronizer = message_filters.ApproximateTimeSynchronizer(
        [detections_filter, cloud_filter],
        queue_size=10,
        slop=0.10,
    )
    matched_pairs = []
    synchronizer.registerCallback(
        lambda detections, cloud: matched_pairs.append(
            (detections, cloud)
        )
    )

    detections = Detection2DArray()
    detections.header.stamp.sec = 20
    detections.header.stamp.nanosec = 20_000_000
    matching_cloud = PointCloud2()
    matching_cloud.header.stamp.sec = 20
    matching_cloud.header.stamp.nanosec = 75_000_000

    detections_filter.signalMessage(detections)
    cloud_filter.signalMessage(matching_cloud)

    assert len(matched_pairs) == 1

    late_detections = Detection2DArray()
    late_detections.header.stamp.sec = 21
    late_cloud = PointCloud2()
    late_cloud.header.stamp.sec = 21
    late_cloud.header.stamp.nanosec = 150_000_000

    detections_filter.signalMessage(late_detections)
    cloud_filter.signalMessage(late_cloud)

    assert len(matched_pairs) == 1
