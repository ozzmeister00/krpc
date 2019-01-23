import math

import sandbox.utils as utils


def changePeriapsis(vessel, ut, targetAltitude):
    mu = vessel.orbit.body.gravitational_parameter

    # where we're starting
    r1 = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    v1 = math.sqrt(mu*((2./r1)-(1./a1)))

    # where we're going
    r2 = r1
    a2 = (r1 + targetAltitude + vessel.orbit.body.equatorial_radius) / 2 # why?
    v2 = math.sqrt(mu*((2./r2)-(1./a2)))

    deltaV = v2 - v1

    node = vessel.control.add_node(ut + vessel.orbit.time_to_apoapsis, prograde=deltaV)

    return node

def changeApoapsis(vessel, ut, targetAltitude):
    mu = vessel.orbit.body.gravitational_parameter

    # where we're starting
    r1 = vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    v1 = math.sqrt(mu * ((2. / r1) - (1. / a1)))

    # where we're going
    r2 = r1
    a2 = (r1 + targetAltitude + vessel.orbit.body.equatorial_radius) / 2
    v2 = math.sqrt(mu * ((2. / r2) - (1. / a2)))

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
    ISP = vessel.specific_impulse * utils.gHere(vessel.orbit.body, vessel)
    m0 = vessel.mass
    m1 = m0 / math.exp(deltaV/ISP)
    F = ((m0 - m1) / t) * ISP
    return F / vessel.available_thrust


class ExecuteManeuver(utils.Program):
    def __init__(self, conn, vessel, node=None, tuneTime=2, leadTime=60):
        super(ExecuteManeuver, self).__init__('Maneuver')
        self.conn = conn
        self.vessel = vessel

        if not node:
            node = vessel.control.nodes[0]

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

        if self.remainingBurn()[1] <= 1.0 or utils.hasAborted(self.vessel):
            self.mode = 'Done'
            self.vessel.control.throttle = 0.0
            self.node.remove()
            self.node = None
            return True

        if self.remainingBurnTime > self.tuneTime:
            self.mode = 'Firing'
            self.vessel.control.throttle = 1
        else:
            self.mode = 'Tuning'
            self.vessel.control.throttle = max(0.005,
                                               smoothThrottle(self.vessel, self.remainingBurn()[1], self.tuneTime))

        return False

    def displayValues(self):
        return [self.prettyName,
                'Mode : ' + self.mode,
                'TlBrn: ' + str(round(self.totalBurnTime, 1)),
                'RmBrn: ' + str(round(self.remainingBurnTime, 1))]