from __future__ import absolute_import, print_function, division

import sys
import time

import krpc
from sandbox.launcherCallable import Ascend
from sandbox.utils import AutoStage, Fairing, hasAborted
from sandbox.gui_testing import Display

from sandbox.maneuvers import changePeriapsis, ExecuteManeuver


def main():
    connection = krpc.connect("Launcher")
    vessel = connection.space_center.active_vessel
    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    ascend = Ascend(connection, vessel, 500000)
    staging = AutoStage(vessel)
    fairing = Fairing(connection, vessel)

    display = Display(connection, vessel, program=ascend)

    for i in range(3, 0, -1):
        display.addMessage('{}...'.format(i))
        display()
        time.sleep(1)

    display.addMessage('Launch!')

    vessel.control.activate_next_stage()

    while not ascend() and not hasAborted(vessel):
        display()
        staging()
        #fairing()
        time.sleep(0.05)

    if hasAborted(vessel):
        display.addMessage('Good luck!')
        sys.exit(1)

    vessel.control.throttle = 0.0

    time.sleep(1)

    display.addMessage('Circularizing')
    node = changePeriapsis(vessel, ut(), vessel.orbit.apoapsis_altitude)
    doManeuver = ExecuteManeuver(connection, vessel, node=node, tuneTime=5, leadTime=60)

    display.changeProgram(doManeuver)

    while not doManeuver() and not hasAborted(vessel):
        display()
        staging()
        time.sleep(0.05)

    node.remove()

    display.addMessage('Welcome to space!')
    display.changeProgram(None)


if __name__ == '__main__':
    main()
