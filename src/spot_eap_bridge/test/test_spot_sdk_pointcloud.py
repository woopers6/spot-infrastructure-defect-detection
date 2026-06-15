from bosdyn.api import point_cloud_pb2
from google.protobuf.timestamp_pb2 import Timestamp
import numpy as np
import pytest

from spot_eap_bridge.spot_sdk_pointcloud import (
    decode_xyz32,
    robot_timestamp_to_ros_time,
    select_point_cloud_source,
)


def test_select_point_cloud_source_prefers_velodyne():
    sources = [
        point_cloud_pb2.PointCloudSource(name='other-source'),
        point_cloud_pb2.PointCloudSource(name='velodyne-point-cloud'),
    ]

    assert select_point_cloud_source(sources) == 'velodyne-point-cloud'


def test_select_point_cloud_source_reports_available_names():
    sources = [point_cloud_pb2.PointCloudSource(name='only-source')]

    with pytest.raises(ValueError, match='only-source'):
        select_point_cloud_source(sources, 'missing-source')


def test_decode_xyz32():
    expected = np.array(
        [[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0]],
        dtype='<f4',
    )
    cloud = point_cloud_pb2.PointCloud(
        num_points=2,
        encoding=point_cloud_pb2.PointCloud.ENCODING_XYZ_32F,
        data=expected.tobytes(),
    )

    np.testing.assert_array_equal(decode_xyz32(cloud), expected)


def test_decode_xyz32_rejects_bad_data_size():
    cloud = point_cloud_pb2.PointCloud(
        num_points=2,
        encoding=point_cloud_pb2.PointCloud.ENCODING_XYZ_32F,
        data=b'bad',
    )

    with pytest.raises(ValueError, match='byte count'):
        decode_xyz32(cloud)


def test_robot_timestamp_to_ros_time_removes_robot_clock_skew():
    timestamp = Timestamp(seconds=100, nanos=250_000_000)

    stamp = robot_timestamp_to_ros_time(
        timestamp,
        clock_skew_nsec=1_500_000_000,
    )

    assert stamp.sec == 98
    assert stamp.nanosec == 750_000_000
