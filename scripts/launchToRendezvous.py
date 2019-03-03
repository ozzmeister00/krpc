"""
Run this script to launch into orbit and then initiate a rendezvous with a target vessel
"""
import math

import kspy.utils
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("LaunchToRendezvous")
vessel = connection.space_center.active_vessel
target = connection.space_center.target_vessel

targetInclination = math.degrees(target.orbit.inclination)
targetLAN = target.orbit.longitude_of_ascending_node

# launch to a conventiently lower orbit that's at least out of the atmosphere
targetAltitude = max(vessel.orbit.body.atmosphere_height, target.orbit.apoapsis_altitude * .66)

# launch
kspy.programs.Launch(connection, vessel,
                     altitude=targetAltitude,
                     targetInclination=targetInclination,
                     longitudeOfAscendingNode=targetLAN)

# fine-tune our orbit to reduce eccentricity
node = kspy.maneuvers.circularizeAtPeriapsis(connection, vessel)
kspy.programs.ExecuteNextManeuver(connection, vessel, node)

# then initiate our rendezvous maneuver
kspy.programs.RendezvousWithTarget(connection, vessel)
