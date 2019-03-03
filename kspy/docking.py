from __future__ import absolute_import, print_function, division

import math

from .maths import Vector3


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
