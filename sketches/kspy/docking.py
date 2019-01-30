from __future__ import absolute_import, print_function, division

import math

from .maths import Vector3


# art whaley
def getSetpoints(offset, proceed, speed_limit):
    """
    returns the computed set points -
    set points are actually just the offset distances clamped to the
    speed_limit variable!   This way we slow down as we get closer to the right
    heading.
    """
    tvup = max(min(offset.up, speed_limit), -speed_limit)
    tvright = -1 * (max(min(offset.right, speed_limit), -speed_limit))
    if proceed:
        tvforward = -.2
    else:
        tvforward = max(min((10 - offset.forward), speed_limit), -speed_limit)
    return Vector3(tvright, tvforward, tvup)


# art whaley
def proceedCheck(offset):
    """
    returns true if we're lined up and ready to move forward to dock.
    """
    return (offset.up < .1 and
            offset.right < .1 and
            math.fabs(10 - offset.forward) < .1)
