from __future__ import absolute_import, print_function, division

import math
import time

from .maths import Vector3
from . import maths
from . import drawing


def getSetPoints(offset, proceed, speedLimit):
    """

    :param offset: the offset vector from the docking port
    :param proceed: if we're supposed to proceed toward the docking port
    :param speedLimit: how much to limit the speed of the vessel

    :returns: The computed set points for docking, which is the offset distances clamped to the speedLimit
    """
    tvUp = max(min(offset.up, speedLimit), -speedLimit)
    tvRight = -1 * (max(min(offset.right, speedLimit), -speedLimit))

    if proceed:
        tvForward = -.2
    else:
        tvForward = max(min((10 - offset.forward), speedLimit), -speedLimit)

    return Vector3(tvRight, tvForward, tvUp)


def proceedCheck(offset):
    """
    A simple check to make sure it's safe to move forward
    toward our docking port

    :param offset: the offset vector from the docking target

    :returns: true if we're lined up and ready to move forward to dock.
    """
    return (offset.up < .1 and
            offset.right < .1 and
            math.fabs(10 - offset.forward) < .1)


def orbitalTraverseToPoint(connection, vessel, targetPoint, targetReferenceFrame):
    """
    using RCS, move the input vessel toward the input point in the input reference frame

    :param connection:
    :param vessel:
    :param targetPoint:
    :param targetReferenceFrame:
    :return:
    """
    pass


def rBarDocking(connection, vessel, target):
    pass
    # RBar line is described by
    tRF = target.orbital_reference_frame
    start = target.position(tRF)

    radialIn = maths.vectorMultiply(target.flight(tRF).radial, -1)
    prograde = target.flight(tRF).prograde

    # find our waypoints
    rBarWaypoint = maths.getPointAwayFrom(start, radialIn, 50.0)
    vBarWaypoint1 = maths.getPointAwayFrom(start, prograde, 50.0)
    vBarWaypoint2 = maths.getPointAwayFrom(start, prograde, 20.0)
    vBarWaypoint3 = maths.getPointAwayFrom(start, prograde, 50.0)

    # also get target docking port to use for vbar waypoint positions
    while True:

        # draw our target waypoints
        connection.drawing.clear()
        drawing.draw_cube(connection, rBarWaypoint, 5, target.orbital_reference_frame)
        # connection.drawing.add_line(start, vBarWaypoint1, target.orbital_reference_frame)
        # connection.drawing.add_line(start, vBarWaypoint2, target.orbital_reference_frame)
        # connection.drawing.add_line(start, vBarWaypoint3, target.orbital_reference_frame)

        time.sleep(0.1)
