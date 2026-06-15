import math

from defect_detection.autonomous_navigation.planning import (
    DefectWaypoint,
    generate_standoff_candidates,
    merge_detection,
    order_candidates_by_distance,
    path_length,
    priority_score,
    select_next_waypoint,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def make_waypoint(
    waypoint_id='defect_1',
    class_id='0',
    confidence=0.8,
    x=2.0,
    y=3.0,
):
    return DefectWaypoint(
        waypoint_id=waypoint_id,
        class_id=class_id,
        confidence=confidence,
        x=x,
        y=y,
        z=0.5,
        size_x=0.4,
        size_y=0.3,
        size_z=0.2,
        first_seen_sec=1.0,
        last_seen_sec=1.0,
    )


def test_merge_detection_deduplicates_nearby_waypoints():
    waypoints = {'defect_1': make_waypoint()}
    repeated = make_waypoint(
        waypoint_id='defect_2',
        confidence=0.9,
        x=2.2,
        y=3.1,
    )

    merged = merge_detection(waypoints, repeated, merge_radius=0.5)

    assert len(waypoints) == 1
    assert merged.observations == 2
    assert merged.confidence == 0.9
    assert merged.x == 2.1


def test_priority_selects_urgent_defect():
    low = make_waypoint(waypoint_id='low', class_id='0')
    high = make_waypoint(waypoint_id='high', class_id='1')
    waypoints = {'low': low, 'high': high}

    selected = select_next_waypoint(
        waypoints,
        now_sec=10.0,
        class_priorities={'0': 1.0, '1': 5.0},
        default_class_priority=1.0,
        confidence_weight=1.0,
        size_weight=0.0,
        observation_weight=0.0,
    )

    assert selected is high


def test_priority_ignores_completed_and_cooling_waypoints():
    completed = make_waypoint(waypoint_id='completed')
    completed.completed = True
    cooling = make_waypoint(waypoint_id='cooling')
    cooling.cooldown_until_sec = 20.0

    selected = select_next_waypoint(
        {'completed': completed, 'cooling': cooling},
        now_sec=10.0,
        class_priorities={},
        default_class_priority=1.0,
        confidence_weight=1.0,
        size_weight=1.0,
        observation_weight=1.0,
    )

    assert selected is None


def test_priority_ignores_abandoned_waypoints():
    abandoned = make_waypoint(waypoint_id='abandoned')
    abandoned.abandoned = True

    selected = select_next_waypoint(
        {'abandoned': abandoned},
        now_sec=10.0,
        class_priorities={},
        default_class_priority=1.0,
        confidence_weight=1.0,
        size_weight=1.0,
        observation_weight=1.0,
    )

    assert selected is None


def test_standoff_candidates_face_defect():
    waypoint = make_waypoint(x=0.0, y=0.0)
    candidates = generate_standoff_candidates(
        waypoint,
        standoff_distance=1.5,
        candidate_count=8,
    )

    assert len(candidates) == 8
    first_x, first_y, first_yaw = candidates[0]
    assert first_x > 1.5
    assert first_y == 0.0
    assert first_yaw == math.pi


def test_candidates_are_ordered_from_robot_position():
    candidates = [
        (5.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (3.0, 0.0, 0.0),
    ]

    ordered = order_candidates_by_distance(candidates, 0.0, 0.0)

    assert [candidate[0] for candidate in ordered] == [1.0, 3.0, 5.0]


def test_priority_score_uses_observations():
    waypoint = make_waypoint()
    initial = priority_score(
        waypoint,
        {},
        1.0,
        0.0,
        0.0,
        1.0,
    )
    waypoint.observations = 5
    repeated = priority_score(
        waypoint,
        {},
        1.0,
        0.0,
        0.0,
        1.0,
    )

    assert repeated > initial


def test_path_length():
    path = Path()
    for x, y in [(0.0, 0.0), (3.0, 4.0), (6.0, 4.0)]:
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)

    assert path_length(path) == 8.0
