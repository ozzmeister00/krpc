from __future__ import absolute_import, print_function, division


# art whaley
def thrust_controller(vessel, deltaV):
    """
    This function is somewhat arbitrary in it's working - there's not a 'rule'
    that I've found on how to feather out throttle towards the end of a burn
    but given that the chances of overshooting go up with TWR (since we fly
    in a world with discrete physics frames!) it makes sense to relate the
    throttle to the TWR for this purpose.
    """
    TWR = vessel.max_thrust / vessel.mass
    if deltaV < TWR / 3:
        return .05
    elif deltaV < TWR / 2:
        return .1
    elif deltaV < TWR:
        return .25
    else:
        return 1.0
    

class MaxQController(object):
    def __init__(self, conn, vessel, maxQ):
        self.vessel = vessel
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)
        self.maxQ = maxQ
        self.high = maxQ * 1.1
        self.low = maxQ * 0.9
        self.q = conn.add_stream(getattr, self.flight, 'dynamic_pressure')

    def __call__(self):
        if self.q() < self.low:
            self.vessel.control.throttle = 1.0
        elif self.q() > self.high:
            self.vessel.control.throttle = 0.0
        else:
            self.vessel.control.throttle = (self.high - self.q()) / (self.high - self.low)
