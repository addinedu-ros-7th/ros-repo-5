
def format_pickup_tasks_log(handler, pickups):
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
            f"- Station: {pickup.station}, "
            f"Handler: {pickup.handler}, "
            f"Payload: [{payloads}]"
        )

    return "\n".join(log_lines)


def format_station_waypoints_log(handler, station_waypoints):
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
            f"- Station: {waypoint.station}, "
            f"Position: (x={position.x}, y={position.y}, z={position.z}), "
            f"Orientation: (x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w})"
        )

    return "\n".join(log_lines)