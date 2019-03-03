from __future__ import absolute_import, print_function, division


class MaxQController(object):
    """
    Calculator to help us figure out what our throttle should be
    to keep the vessel just shy of maxQ
    """

    def __init__(self, connection, vessel, maxQ):
        """
        :param connection: connection to use
        :param vessel: vessel to control
        :param maxQ: maximum aerodynamic force allowed on the vessel
        """
        self.vessel = vessel
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)
        self.maxQ = maxQ
        self.high = maxQ * 1.1
        self.low = maxQ * 0.9
        self.q = connection.add_stream(getattr, self.flight, 'dynamic_pressure')

    def __call__(self):
        # if dynamic press is low, full throttle
        if self.q() < self.low:
            self.vessel.control.throttle = 1.0

        # if we've somehow gone beyond our upper limit, kill the throttle
        elif self.q() > self.high:
            self.vessel.control.throttle = 0.0

        # otherwise tune the throttle proportional to aerodynamic pressure within our upper and lower bounds
        else:
            self.vessel.control.throttle = (self.high - self.q()) / (self.high - self.low)
