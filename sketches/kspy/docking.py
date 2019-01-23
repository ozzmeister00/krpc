from __future__ import absolute_import, print_function, division

import math
import time

from .pid import PID
from .maths import Vector3, getOffsets, getVelocities


def getSetpoints(offset, proceed, speed_limit):
    '''
    returns the computed set points -
    set points are actually just the offset distances clamped to the
    speed_limit variable!   This way we slow down as we get closer to the right
    heading.
    '''
    tvup = max(min(offset.up, speed_limit), -speed_limit)
    tvright = -1 * (max(min(offset.right, speed_limit), -speed_limit))
    if proceed:
        tvforward = -.2
    else:
        tvforward = max(min((10 - offset.forward), speed_limit), -speed_limit)
    return Vector3(tvright, tvforward, tvup)


def proceedCheck(offset):
    '''
    returns true if we're lined up and ready to move forward to dock.
    '''
    return (offset.up < .1 and
            offset.right < .1 and
            math.fabs(10 - offset.forward) < .1)


def dock(conn, speed_limit=1.0):
    # Setup KRPC
    sc = conn.space_center
    v = sc.active_vessel
    t = sc.target_docking_port
    ap = v.auto_pilot
    rf = v.orbit.body.reference_frame

    # Setup Auto Pilot
    ap.reference_frame = rf
    ap.target_direction = tuple(x * -1 for x in t.direction(rf))
    ap.engage()

    # create PIDs
    upPID = PID(.75, .25, 1)
    rightPID = PID(.75, .25, 1)
    forwardPID = PID(.75, .2, .5)

    proceed = False
    # 'proceed' is a flag that signals that we're lined up and ready to dock.
    # Otherwise the program will try to line up 10m from the docking port.

    # LineUp and then dock  - in the same loop with 'proceed' controlling whether
    # we're headed for the 10m safe point, or headed forward to dock.
    while True:
        offset = getOffsets(v, t)  # grab data and compute setpoints
        velocity = getVelocities(v, t)
        if proceedCheck(offset):  # Check whether we're lined up and ready to dock
            proceed = True
        setpoints = getSetpoints(offset, proceed, speed_limit)

        upPID.setpoint(setpoints.up)  # set PID setpoints
        rightPID.setpoint(setpoints.right)
        forwardPID.setpoint(setpoints.forward)

        v.control.up = -upPID.update(velocity.up)  # steer vessel
        v.control.right = -rightPID.update(velocity.right)
        v.control.forward = -forwardPID.update(velocity.forward)

        time.sleep(.05)
