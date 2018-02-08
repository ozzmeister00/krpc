def get_active_vessel(conn):
    """
    Convenience function to get the active vessel.

    Arguments:
        *conn* : a krpc connection

    Returns:
        *vessel* : an object representing the active vessel

    """

    space_center = conn.space_center
    vessel = space_center.active_vessel

    return vessel
