from __future__ import absolute_import, print_function, division

import math

import kspy.programs as programs
import kspy.maneuvers as maneuvers

import krpc

connection = krpc.connect("LaunchToRendezvous")
vessel = connection.space_center.active_vessel
target = connection.space_center.target_body or connection.space_center.target_vessel

inc = math.degrees(target.orbit.inclination)
lan = target.orbit.longitude_of_ascending_node

#alt = target.orbit.apoapsis_altitude * .66
alt = 1000000

connection.space_center.quicksave()
#
programs.Launch(connection, vessel, altitude=alt, longitudeOfAscendingNode=lan, targetInclination=inc)
#programs.Launch(connection, vessel, altitude=500000)
#
#
# # once we're in space, re-circularize, just in cases
connection.space_center.quicksave()
#
node = maneuvers.circularizeAtApoapsis(connection, vessel)
programs.ExecuteNextManeuver()

connection.space_center.quicksave()
# # then trigger the rendezvous
programs.RendevousWithTarget(connection, vessel)
#

# node = maneuvers.circularizeAtPeriapsis(connection, vessel)
# programs.ExecuteNextManeuver(connection, vessel, node)
#
# programs.RendevousWithTarget(connection, vessel)
