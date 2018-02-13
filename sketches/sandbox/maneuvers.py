import math

import sandbox.utils as utils


def changePeriapsis(vessel, ut, targetAltitude):
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.body.equatorial_radius + targetAltitude
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    deltaV = v2 - v1

    node = vessel.control.add_node(ut + vessel.orbit.time_to_apoapsis, prograde=deltaV)

    return node

def changeApoapsis(vessel, ut, targetAltitude):
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.body.equatorial_radius + targetAltitude
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    deltaV = v2 - v1

    node = vessel.control.add_node(ut + vessel.orbit.time_to_periapsis, prograde=deltaV)

    return node


def calculateBurnTime(vessel, node):
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * vessel.orbit.body.gravitational_parameter
    m0 = vessel.mass

    m1 = m0 / math.exp(node.delta_v/Isp) # math range error
    flowRate = F / Isp
    burnTime = (m0 - m1) / flowRate
    return burnTime


def smoothThrottle(vessel, deltaV, t):
    ISP = vessel.specific_impulse * utils.gHere()
    m0 = vessel.mass
    m1 = m0 / math.exp(deltaV/ISP)
    F = ((m0 - m1) / t) * ISP
    return F / vessel.available_thrust


class ExecuteManeuver(utils.Program):
    def __init__(self, conn, vessel, node, tuneTime=2, leadTime=5):
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

    def __call__(self):
        if not self.node:
            return True

        burnUT = self.nodeUT - (self.totalBurnTime / 2.)

        if self.ut() - burnUT - self.leadTime:
            # TODO this is a blocking call
            self.conn.space_center.warp_to(burnUT - self.leadTime)

        if self.ut() < burnUT:
            return False

        self.remainingBurnTime = calculateBurnTime(self.vessel, self.node)

        if self.remainingBurnTime > self.tuneTime:
            self.vessel.control.throttle = 1
            return False
        # Tuning isn't quite working right. A little too aggressive
        elif self.remainingBurn()[1] > 0:
            self.vessel.control.throttle = max(0.005, smoothThrottle(self.vessel, self.remainingBurn()[1], self.tuneTime))
            return False
        else:
            self.vessel.control.throttle = 0.0
            self.node = None
            return True

    def displayValues(self):
        return [self.prettyName] # todo