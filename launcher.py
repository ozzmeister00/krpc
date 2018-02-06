import time
import sys

import krpc

from launcherCallable import Ascend

from maneuvers import changePeriapsis, ExecuteManeuver
from utils import AutoStage, hasAborted

def main():
    connection = krpc.connect("Launcher")
    vessel = connection.space_center.active_vessel
    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    ascend = Ascend(connection, vessel, 200000)
    staging = AutoStage(vessel)

    for i in range(3, -1, -1):
        print(i, '...')
        time.sleep(1)

    print('Launch')

    vessel.control.activate_next_stage()

    while not ascend() and not hasAborted(vessel):
        staging()
        time.sleep(0.1)

    if hasAborted(vessel):
        print('Good luck!')
        sys.exit(1)

    vessel.control.throttle = 0.0

    time.sleep(1)

    print('Circularizing')
    node = changePeriapsis(vessel, ut(), vessel.orbit.apoapsis_altitude)
    doManeuver = ExecuteManeuver(connection, vessel, node, tuneTime = 5, leadTime=60)

    while not doManeuver() and not hasAborted(vessel):
        staging()
        time.sleep(0.1)

    node.remove()

    print('Welcome to space!')

if __name__ == '__main__':
    main()



