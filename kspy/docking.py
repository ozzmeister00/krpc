from __future__ import absolute_import, print_function, division

import math
import time

from .maths import Vector3
from . import maths
from . import drawing
from .pid import PID


def getSetPoints(offset, speedLimit):
    """

    :param offset: the offset vector from our target position
    :param speedLimit: how much to limit the speed of the vessel

    :returns: The computed set points for traversal, which is the offset distances clamped to the speedLimit
    """
    tvUp = max(min(offset.up, speedLimit), -speedLimit)
    tvRight = -1 * (max(min(offset.right, speedLimit), -speedLimit))
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


def getOffsetBetweenPoints(fromPoint, toPoint):
    """

    :param fromPoint:
    :param toPoint:
    :return:
    """
    t = maths.vectorMultiply(toPoint, -1)
    return maths.vectorAdd(fromPoint, t)


def getDirectionBetweenPoints(vessel, toPoint, referenceFrame):
    """

    :param vessel:
    :param toPoint:
    :return:
    """
    ourPosition = vessel.position(referenceFrame)
    offset = getOffsetBetweenPoints(ourPosition, toPoint)
    return maths.normalizeVector(offset)


def orbitalTraverseToPoint(connection, vessel, targetPoint, targetReferenceFrame):
    """
    using RCS, move the input vessel toward the input point in the input reference frame

    :param connection:
    :param vessel:
    :param targetPoint:
    :param targetReferenceFrame:
    :return:
    """
    def getReferenceFrame(connection, vessel, targetPoint, targetReferenceFrame):
        return connection.space_center.ReferenceFrame.create_relative(targetReferenceFrame, position=targetPoint, rotation=vessel.rotation(targetReferenceFrame))


    ap = vessel.auto_pilot

    # Setup Auto Pilot
    ap.reference_frame = getReferenceFrame(connection, vessel, targetPoint, targetReferenceFrame)
    ap.target_direction = getDirectionBetweenPoints(vessel, targetPoint, targetReferenceFrame)
    ap.target_roll = 90
    ap.engage()

    arrived = False

    # create PIDs
    upPID = PID(.1, .01, .01)
    rightPID = PID(.1, .01, .01)
    forwardPID = PID(.1, .01, .01)

    while not arrived:
        # draw a cube so we know where we're supposed to be going
        connection.drawing.clear()

        # always keep us pointed at our target
        rf = getReferenceFrame(connection, vessel, targetPoint, targetReferenceFrame)
        ap.reference_frame = targetReferenceFrame
        ap.target_direction = getDirectionBetweenPoints(vessel, targetPoint, targetReferenceFrame)

        offset = getOffsetBetweenPoints(vessel.position(rf), targetPoint)
        offset = Vector3._make(offset)
        velocity = Vector3._make(vessel.flight(rf).velocity)
        setpoints = getSetPoints(offset, 1.0)

        print(offset)

        upPID.setpoint(setpoints.up)  # set PID setpoints
        rightPID.setpoint(setpoints.right)
        forwardPID.setpoint(setpoints.forward)

        upMag = upPID.update(velocity.up) * 20.0  # steer vessel
        rightMag = rightPID.update(velocity.right) * 20.0
        forwardMag = forwardPID.update(velocity.forward) * 40.0

        upDir = (0,0,1)
        rightDir = (1,0,0)
        fowardDir = (0,1,0)
        connection.drawing.add_direction(fowardDir, rf, length=forwardMag)
        connection.drawing.add_direction(rightDir, rf, length=rightMag)
        connection.drawing.add_direction(upDir, rf, length=upMag)

        # right-control still isn't working quite right. Maybe verify that "right" actually means right.
        # it looks like when printing the vectors that right and up creep up but forward decreases.
        # we ideally want to travel a straight line relative to the target vessel
        # which necessarily means we'll be traversing up and right to counteract orbital motion

        vessel.control.up = upPID.update(velocity.up)  # steer vessel
        vessel.control.right = -rightPID.update(velocity.right)
        vessel.control.forward = forwardPID.update(velocity.forward)

        drawing.draw_cube(connection, targetPoint, 5, targetReferenceFrame)

        if maths.magnitude(offset) < 1.0:
            return

        time.sleep(0.01)


def rBarDocking(connection, vessel, target):
    pass
    # RBar line is described by
    tRF = target.orbital_reference_frame
    start = target.position(tRF)

    radialIn = maths.vectorMultiply(target.flight(tRF).radial, -1)
    prograde = target.flight(tRF).prograde
    target.auto_pilot.reference_frame = tRF
    target.auto_pilot.target_direction = prograde
    target.auto_pilot.target_roll = 0.0

    # find our waypoints
    rBarWaypoint = maths.getPointAwayFrom(start, radialIn, 50.0)
    vBarWaypoint1 = maths.getPointAwayFrom(start, prograde, 50.0)
    vBarWaypoint2 = maths.getPointAwayFrom(start, prograde, 20.0)
    vBarWaypoint3 = maths.getPointAwayFrom(start, prograde, 50.0)

    orbitalTraverseToPoint(connection, vessel, rBarWaypoint, tRF)