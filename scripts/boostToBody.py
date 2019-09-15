"""
Run this script to launch into orbit and then initiate a rendezvous with a target vessel
"""
import math
import time

import kspy.utils
import kspy.node as node
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("LaunchToRendezvous")
vessel = connection.space_center.active_vessel

maneuverNode = vessel.control.nodes[0]

doManeuver = node.ExecuteManeuver(connection, vessel, maneuverNode, tuneTime=10)

currentBody = vessel.orbit.body

while not doManeuver() and not vessel.orbit.next_orbit:
    time.sleep(0.01)

vessel.control.sas = True
vessel.control.throttle = 0.0
vessel.auto_pilot.disengage()

# remove the node!
maneuverNode.remove()

connection.space_center.warp_to(connection.space_center.ut + vessel.orbit.time_to_soi_change)

kspy.maneuvers.circularizeAtPeriapsis(connection, vessel)
kspy.programs.ExecuteNextManeuver(connection, vessel)

