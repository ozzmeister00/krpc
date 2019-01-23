import krpc
import time

from sandbox.maneuvers import ExecuteManeuver
from sandbox.gui_testing import Display
from sandbox.utils import AutoStage

connection = krpc.connect("Launcher")
vessel = connection.space_center.active_vessel


def main():
    doManeuver = ExecuteManeuver(connection, vessel, tuneTime=20)
    display = Display(connection, vessel, program=doManeuver)
    autostage = AutoStage(vessel)

    display.addMessage("Executing next maneuver")

    display()

    while not doManeuver():
        #autostage()
        display()
        time.sleep(0.05)

    display.addMessage("Maneuver Complete")

    display()

    vessel.control.sas = True
    vessel.control.throttle = 0.0

if __name__ == '__main__':
    main()