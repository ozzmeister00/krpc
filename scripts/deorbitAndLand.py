"""
Run this script to launch into orbit and then initiate a rendezvous with a target vessel
"""
import math

import kspy.utils
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("DeorbitAndLand")
vessel = connection.space_center.active_vessel

kspy.programs.ExecuteNextManeuver(connection, vessel)

kspy.programs.SoftLanding(connection, vessel)
