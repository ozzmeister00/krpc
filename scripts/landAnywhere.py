"""
Run this script to land a craft anywhere on the orbited body under power
NOTE: Not tested on atmospheric bodies.
"""
import math

import kspy.utils
import kspy.programs
import kspy.maneuvers

connection = kspy.utils.defaultConnection("LandAnywhere")
vessel = connection.space_center.active_vessel

# trigger the landing program
kspy.programs.LandAnywhere(connection, vessel)

# quicksave once it's done
connection.space_center.quick_save()
