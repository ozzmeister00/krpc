"""
Contains functionality to help us execute maneuver nodes
"""

def thrust_controller(vessel, deltaV):
    '''
    This function is somewhat arbitrary in it's working - there's not a 'rule'
    that I've found on how to feather out throttle towards the end of a burn
    but given that the chances of overshooting go up with TWR (since we fly
    in a world with discrete physics frames!) it makes sense to relate the
    throttle to the TWR for this purpose.
    '''
    TWR = vessel.max_thrust / vessel.mass
    if deltaV < TWR / 3:
        return .05
    elif deltaV < TWR / 2:
        return .1
    elif deltaV < TWR:
        return .25
    else:
        return 1.0


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