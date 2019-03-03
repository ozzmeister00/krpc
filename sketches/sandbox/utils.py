import time
from collections import deque
from math import cos, sin, radians, degrees, atan2, asin, pi


def getLandingReferenceFrame(connection, body, vessel, landing_longitude, landing_latitude):

    ReferenceFrame = connection.space_center.ReferenceFrame

    # Define landing site
    landing_latitude = -(0+(5.0/60)+(48.38/60/60)) # Top of the VAB
    landing_longitude = -(74+(37.0/60)+(12.2/60/60))
    landing_altitude = 2000

    # Determine landing site reference frame (orientation: x=zenith, y=north, z=east)
    landing_position = body.surface_position(landing_latitude, landing_longitude, body.reference_frame)
    q_long = (0, sin(-landing_longitude * 0.5 * pi / 180), 0, cos(-landing_longitude * 0.5 * pi / 180))
    q_lat = (0, 0, sin(landing_latitude * 0.5 * pi / 180), cos(landing_latitude * 0.5 * pi / 180))
    landing_reference_frame = ReferenceFrame.create_relative(
                                ReferenceFrame.create_relative(
                                  ReferenceFrame.create_relative(
                                    body.reference_frame,
                                    landing_position,
                                    q_long),
                                (0,0,0),
                                q_lat),
                                (landing_altitude, 0, 0)
                                )

    # Up, North, East
    vessel.velocity(landing_reference_frame)
    return landing_reference_frame