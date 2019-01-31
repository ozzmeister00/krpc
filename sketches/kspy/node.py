"""
Contains functionality to help us execute maneuver nodes
"""
from __future__ import print_function, absolute_import, division

import math

from . import utils


def calculateBurnTime(vessel, node):
    if not vessel.available_thrust:
        return -1

    F = vessel.available_thrust
    Isp = vessel.specific_impulse * vessel.orbit.body.gravitational_parameter
    m0 = vessel.mass

    m1 = m0 / math.exp(node.delta_v/Isp) # math range error
    flowRate = F / Isp
    burnTime = (m0 - m1) / flowRate
    return burnTime


def smoothThrottle(vessel, deltaV, t):
    """
    returns a smooth throttle value based on the remaining deltaV of our burn

    :param vessel:
    :param deltaV:
    :param t: total burn time(?)
    :return:
    """
    ISP = vessel.specific_impulse * utils.gHere(vessel.orbit.body, vessel)
    m0 = vessel.mass
    m1 = m0 / math.exp(deltaV/ISP)
    F = ((m0 - m1) / t) * ISP
    return F / vessel.available_thrust


class ExecuteManeuver(utils.Program):
    def __init__(self, conn, vessel, node, tuneTime=2, leadTime=60):
        super(ExecuteManeuver, self).__init__('Maneuver')
        self.conn = conn
        self.vessel = vessel

        self.node = node
        self.leadTime = leadTime
        self.remainingBurn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
        self.nodeUT = node.ut
        self.totalBurnTime = calculateBurnTime(vessel, node)
        self.remainingBurnTime = self.totalBurnTime
        self.ut = conn.add_stream(getattr, conn.space_center, 'ut')
        self.tuneTime = 2

        self.ap = vessel.auto_pilot
        self.ap.reference_frame = node.reference_frame
        self.ap.target_direction = (0, 1, 0)
        self.ap.target_roll = float("nan")
        self.ap.engage()

        self.mode = ''

    def __call__(self):
        if not self.node:
            self.messages.append('Node deleted')
            return True

        burnUT = self.nodeUT - (self.totalBurnTime / 2.)

        if self.ut() - burnUT - self.leadTime:
            self.mode = 'Warping'
            self.conn.space_center.warp_to(burnUT - self.leadTime)

        if self.ut() < burnUT:
            return False

        self.remainingBurnTime = calculateBurnTime(self.vessel, self.node)

        # for safety, let autostaging takeover here
        if self.remainingBurnTime == -1:
            return False

        if self.remainingBurn()[1] <= 1.0 or utils.hasAborted(self.vessel):
            self.mode = 'Done'
            self.vessel.control.throttle = 0.0
            #self.node.remove()
            self.node = None
            return True

        # if self.remainingBurnTime > self.tuneTime:
        #     self.mode = 'Firing'
        #     self.vessel.control.throttle = 1
        # else:
        #     self.mode = 'Tuning'
        self.vessel.control.throttle = max(0.005, smoothThrottle(self.vessel, self.remainingBurn()[1], self.tuneTime))

        return False

    def displayValues(self):
        return [self.prettyName,
                'Mode : ' + self.mode,
                'TlBrn: ' + str(round(self.totalBurnTime, 1)),
                'RmBrn: ' + str(round(self.remainingBurnTime, 1))]