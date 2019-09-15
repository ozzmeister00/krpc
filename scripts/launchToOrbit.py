"""
Run this script to launch into a specified orbit
"""
import math

import kspy.utils
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("LaunchToOrbit")
vessel = connection.space_center.active_vessel

targetInclination = math.radians(2.1)
targetLAN = math.radians(67.8)
targetAltitude = 3000000

# launch
kspy.programs.Launch(connection, vessel,
                     altitude=targetAltitude,
                     targetInclination=math.degrees(targetInclination),
                     longitudeOfAscendingNode=targetLAN)

# tune our inclination
# node = kspy.maneuvers.changeInclination(targetInclination, connection, vessel)
# kspy.programs.ExecuteNextManeuver(connection, vessel, node)

# fine-tune our orbit to reduce eccentricity
# node = kspy.maneuvers.changePeriapsis(targetApoapsis, connection, vessel)
# kspy.programs.ExecuteNextManeuver(connection, vessel, node)
