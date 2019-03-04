"""
Run this script to launch into a specified orbit
"""
import math

import kspy.utils
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("LaunchToOrbit")
vessel = connection.space_center.active_vessel

targetInclination = math.radians(15.2)
targetLAN = math.radians(58.6)
targetAltitude = 2887042
launchAltitude = max(targetAltitude * .66, vessel.orbit.body.atmosphere_depth)

# launch
kspy.programs.Launch(connection, vessel,
                     altitude=launchAltitude,
                     targetInclination=math.degrees(targetInclination),
                     longitudeOfAscendingNode=targetLAN)

# tune our inclination
node = kspy.maneuvers.changeInclination(targetInclination, connection, vessel)
kspy.programs.ExecuteNextManeuver(connection, vessel, node)

# fine-tune our orbit to reduce eccentricity
node = kspy.maneuvers.changeApoapsis(targetAltitude, connection, vessel)
kspy.programs.ExecuteNextManeuver(connection, vessel, node)

# then initiate our rendezvous maneuver
node = kspy.maneuvers.circularizeAtApoapsis(connection, vessel)
kspy.programs.ExecuteNextManeuver(connection, vessel, node)
