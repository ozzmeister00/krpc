"""
Run this script to execute your next maneuver
"""
import math

import kspy.utils
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("LaunchToOrbit")
vessel = connection.space_center.active_vessel

kspy.programs.ExecuteNextManeuver(connection, vessel)
