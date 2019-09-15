"""
Run this script to launch into orbit and then initiate a rendezvous with a target vessel
"""
import math

import kspy.utils
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("lowerOrbitForLanding")
vessel = connection.space_center.active_vessel

# fine-tune our orbit to reduce eccentricity
node = kspy.maneuvers.changeApoapsis(29000, connection, vessel)
kspy.programs.ExecuteNextManeuver(connection, vessel, node)

node = kspy.maneuvers.circularizeAtPeriapsis(connection, vessel)
kspy.programs.ExecuteNextManeuver(connection, vessel, node)
