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
