from dataclasses import dataclass, field
import math
from typing import Optional


@dataclass
class DefectWaypoint:

    waypoint_id: str
    class_id: str
    confidence: float
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float
    first_seen_sec: float
    last_seen_sec: float
    observations: int = 1
    attempts: int = 0
    completed: bool = False
    abandoned: bool = False
    cooldown_until_sec: float = 0.0
    selected_goal: Optional[tuple[float, float, float]] = None
    metadata: dict = field(default_factory=dict)


def distance_2d(first_x, first_y, second_x, second_y):
    return math.hypot(first_x - second_x, first_y - second_y)


def merge_detection(
    waypoints,
    detection,
    merge_radius,
):
    best_match = None
    best_distance = math.inf

    for waypoint in waypoints.values():
        if waypoint.class_id != detection.class_id:
            continue
        separation = distance_2d(
            waypoint.x,
            waypoint.y,
            detection.x,
            detection.y,
        )
        if separation <= merge_radius and separation < best_distance:
            best_match = waypoint
            best_distance = separation

    if best_match is None:
        waypoints[detection.waypoint_id] = detection
        return detection

    count = best_match.observations
    next_count = count + 1
    best_match.x = (best_match.x * count + detection.x) / next_count
    best_match.y = (best_match.y * count + detection.y) / next_count
    best_match.z = (best_match.z * count + detection.z) / next_count
    best_match.confidence = max(
        best_match.confidence,
        detection.confidence,
    )
    best_match.size_x = max(best_match.size_x, detection.size_x)
    best_match.size_y = max(best_match.size_y, detection.size_y)
    best_match.size_z = max(best_match.size_z, detection.size_z)
    best_match.last_seen_sec = detection.last_seen_sec
    best_match.observations = next_count
    return best_match


def priority_score(
    waypoint,
    class_priorities,
    default_class_priority,
    confidence_weight,
    size_weight,
    observation_weight,
):
    class_priority = class_priorities.get(
        waypoint.class_id,
        default_class_priority,
    )
    planar_size = math.hypot(waypoint.size_x, waypoint.size_y)
    observation_score = math.log1p(waypoint.observations)
    return (
        class_priority
        + confidence_weight * waypoint.confidence
        + size_weight * planar_size
        + observation_weight * observation_score
    )


def select_next_waypoint(
    waypoints,
    now_sec,
    class_priorities,
    default_class_priority,
    confidence_weight,
    size_weight,
    observation_weight,
):
    candidates = [
        waypoint
        for waypoint in waypoints.values()
        if not waypoint.completed
        and not waypoint.abandoned
        and waypoint.cooldown_until_sec <= now_sec
    ]
    if not candidates:
        return None

    return max(
        candidates,
        key=lambda waypoint: priority_score(
            waypoint,
            class_priorities,
            default_class_priority,
            confidence_weight,
            size_weight,
            observation_weight,
        ),
    )


def generate_standoff_candidates(
    waypoint,
    standoff_distance,
    candidate_count,
):
    obstacle_radius = 0.5 * math.hypot(
        waypoint.size_x,
        waypoint.size_y,
    )
    radius = standoff_distance + obstacle_radius
    candidates = []

    for index in range(candidate_count):
        angle = 2.0 * math.pi * index / candidate_count
        x = waypoint.x + radius * math.cos(angle)
        y = waypoint.y + radius * math.sin(angle)
        yaw = math.atan2(waypoint.y - y, waypoint.x - x)
        candidates.append((x, y, yaw))

    return candidates


def order_candidates_by_distance(candidates, robot_x, robot_y):
    return sorted(
        candidates,
        key=lambda candidate: distance_2d(
            candidate[0],
            candidate[1],
            robot_x,
            robot_y,
        ),
    )


def path_length(path):
    poses = path.poses
    if len(poses) < 2:
        return 0.0

    return sum(
        distance_2d(
            first.pose.position.x,
            first.pose.position.y,
            second.pose.position.x,
            second.pose.position.y,
        )
        for first, second in zip(poses, poses[1:])
    )
