"""
Contains functionality to help us execute maneuver nodes
"""
from __future__ import print_function, absolute_import, division

import math

from . import utils


def calculateBurnTime(vessel, node):
    """
    Given a vessel and a maneuver node, calculate for how long the vessel will need to fire
    at full throttle to execute the given node

    :param vessel: the vessel to check
    :param node: the maneuver node to calculate

    :return: time, in seconds, the burn will last
    """
    if not vessel.available_thrust:
        return -1

    F = vessel.available_thrust
    Isp = vessel.specific_impulse * vessel.orbit.body.gravitational_parameter
    m0 = vessel.mass

    m1 = m0 / math.exp(node.delta_v/Isp)
    flowRate = F / Isp
    burnTime = (m0 - m1) / flowRate
    return burnTime


def smoothThrottle(vessel, deltaV, t):
    """
    returns a smooth throttle value based on the remaining deltaV of our burn

    :param vessel: the vessel that's burning
    :param deltaV: remaining delta v
    :param t: total burn time

    :return: throttle value to set to smoothly complete the node
    """
    ISP = vessel.specific_impulse * utils.gHere(vessel.orbit.body, vessel)
    m0 = vessel.mass
    m1 = m0 / math.exp(deltaV/ISP)
    F = ((m0 - m1) / t) * ISP
    return F / max(0.001, vessel.available_thrust)


class ExecuteManeuver(utils.Program):
    """
    Program object to execute a maneuver node smoothly and safely
    """
    def __init__(self, connection, vessel, node, tuneTime=2, leadTime=60):
        """

        :param connection: The connection we're working with
        :param vessel: the vessel that's executing the node
        :param node: the node we're executing
        :param tuneTime: how many seconds from the end of the burn should we start tuning throttle down
        :param leadTime: how long before we have to start burning should we exit time warp
                        Leave this value pretty high if your vessel can't orient quickly
        """
        super(ExecuteManeuver, self).__init__('Maneuver')
        self.connection = connection
        self.vessel = vessel
        self.tuneTime = tuneTime

        # stash off some helpers  and static values for easy access
        self.node = node
        self.nodeUT = node.ut
        self.leadTime = leadTime
        self.totalBurnTime = calculateBurnTime(vessel, node)

        # set up helpful streams
        self.remainingBurn = connection.add_stream(node.remaining_burn_vector, node.reference_frame)
        self.ut = connection.add_stream(getattr, connection.space_center, 'ut')

        self.remainingBurnTime = self.totalBurnTime

        # get the autopilot pointing toward the maneuver node
        self.ap = vessel.auto_pilot
        self.ap.reference_frame = node.reference_frame
        self.ap.target_direction = (0, 1, 0)
        self.ap.target_roll = float("nan")
        self.ap.engage()

    def __call__(self):

        # if we're missing the maneuver node (because the user deleted it, bail out
        if not self.node:
            self.vessel.control.throttle = 0.0
            return True

        # when do we start the burn
        burnUT = self.nodeUT - (self.totalBurnTime / 2.)

        # keep warping until this value is 0
        if self.ut() - burnUT - self.leadTime:
            self.connection.space_center.warp_to(burnUT - self.leadTime)

        # if it's before when we should start burning, just pass
        if self.ut() < burnUT:
            return False

        self.remainingBurnTime = calculateBurnTime(self.vessel, self.node)

        # for safety, let autostaging takeover here
        if self.remainingBurnTime == -1:
            return False

        # if we're close enough, vail out
        if self.remainingBurn()[1] <= 0.1:
            self.vessel.control.throttle = 0.0
            self.node = None
            return True

        # let smooth throttle figure how how much to burn
        self.vessel.control.throttle = max(0.005, smoothThrottle(self.vessel, self.remainingBurn()[1], self.tuneTime))

        return False

    def displayValues(self):
        return [self.prettyName]
