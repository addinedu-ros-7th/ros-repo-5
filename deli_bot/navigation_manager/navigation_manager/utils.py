import math
import geometry_msgs.msg

def format_pickup_tasks_log(pickups):
    """
    Formats a log message for received pickup tasks.

    :param handler: The robot ID making the request.
    :param pickups: List of PickUp messages received from the service request.
    :return: Formatted log string.
    """
    log_lines = []
    for pickup in pickups:
        payloads = ", ".join(
            [f"(SKU: {p.sku}, Quantity: {p.quantity})" for p in pickup.payload]
        )

        log_lines.append(
            f"\n- Station: {pickup.station}, \n"
            f"- Handler: {pickup.handler}, \n"
            f"- Payload: [{payloads}]\n"
        )
    return "\n".join(log_lines)


def format_station_waypoints_log(station_waypoints):
    """
    Formats a log message for received station waypoints.

    :param handler: The robot ID making the request.
    :param station_waypoints: List of StationWaypoint messages received from the service response.
    :return: Formatted log string.
    """
    log_lines = []
    for waypoint in station_waypoints:
        position = waypoint.waypoint.pose.position
        orientation = waypoint.waypoint.pose.orientation

        log_lines.append(
            f"\n- Station: {waypoint.station}, \n"
            f"- Position: (x={position.x}, y={position.y}, z={position.z}), \n"
            f"- Orientation: (x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w})\n"
        )
    return "\n".join(log_lines)


def format_pose_log(pose_stamped: geometry_msgs.msg.PoseStamped) -> str:
    """Format PoseStamped and distance remaining for logging."""
    position = pose_stamped.pose.position
    orientation = pose_stamped.pose.orientation

    log_message = (
        f"\n- pose={{position: {{x: {position.x}, y: {position.y}, z: {position.z}}},\n"
        f"- orientation: {{x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}}}}})\n"
    )
    return log_message


def format_target_pose_log(target_pose):
    """
    Formats a log message for target pose.

    :target_pose: t
    :return: Formatted log string.
    """
    log_lines = []

    position = target_pose.pose.position
    orientation = target_pose.pose.orientation

    log_lines.append(
        f"\n- Position: (x={position.x}, y={position.y}, z={position.z}), \n"
        f"- Orientation: (x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w})\n"
    )
    return "\n".join(log_lines)


def format_feedback_log(current_pose: geometry_msgs.msg.PoseStamped, distance_remaining: float) -> str:
    """Format PoseStamped and distance remaining for logging."""
    position = current_pose.pose.position
    orientation = current_pose.pose.orientation
    header = current_pose.header

    log_message = (
        f"\n- Current pose: \n"
        f"  - pose={{position: {{x: {position.x}, y: {position.y}, z: {position.z}}},\n"
        f"  - orientation: {{x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}}}}})\n"
        f"- Distance remaining: {distance_remaining:.2f}\n"
    )
    return log_message