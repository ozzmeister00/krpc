"""
Run this script to launch into orbit and then initiate a rendezvous with a target vessel
"""
import math

import kspy.utils
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("NextBodyRendezvous")
vessel = connection.space_center.active_vessel
target = connection.space_center.target_vessel

# kspy.programs.ExecuteNextManeuver(connection, vessel)
#
# vessel.control.activate_next_stage()  # OPTIONAL

warpTo = vessel.orbit.next_orbit.time_to_periapsis + vessel.orbit.time_to_soi_change + connection.space_center.ut - 300

connection.space_center.warp_to(warpTo)

kspy.maneuvers.circularizeAtPeriapsis(connection, vessel)
kspy.programs.ExecuteNextManeuver(connection, vessel)

#kspy.programs.RendezvousWithTarget(connection, vessel)
