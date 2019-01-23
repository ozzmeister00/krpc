import krpc
import time

from sandbox.maneuvers import ExecuteManeuver
from sandbox.gui_testing import Display
from sandbox.utils import AutoStage

connection = krpc.connect("Launcher")
vessel = connection.space_center.active_vessel


def main():


if __name__ == '__main__':
    main()