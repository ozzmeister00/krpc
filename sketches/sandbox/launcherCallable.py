import sandbox.utils as utils
import sandbox.throttle as throttle

class Ascend(utils.Program):
    def __init__(self, conn, vessel, targetAltitude):
        super(Ascend, self).__init__('Ascend')

        self.conn = conn
        self.vessel = vessel
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)
        self.turnStartAltitude = 250
        self.turnEndAltitude = vessel.orbit.body.atmosphere_depth * 0.75
        self.targetAltitude = targetAltitude
        self.maxQ = 30000

        self.ut = conn.add_stream(getattr, conn.space_center, 'ut')
        self.altitude = conn.add_stream(getattr, self.flight, 'mean_altitude')
        self.apoapsis = conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        self.periapsis = conn.add_stream(getattr, self.vessel.orbit, 'periapsis_altitude')
        self.eccentricity = conn.add_stream(getattr, self.vessel.orbit, 'eccentricity')

        self.throttle = throttle.MaxQController(conn, vessel, maxQ=self.maxQ)
        self.autoPilot = vessel.auto_pilot

        self.vessel.control.sas = False
        self.vessel.control.rcs = False
        self.vessel.control.throttle = 1.0
        self.autoPilot.reference_frame = vessel.surface_reference_frame
        self.autoPilot.target_pitch_and_heading(90, 90)
        self.autoPilot.target_roll = float("nan")
        self.autoPilot.engage()

    def __call__(self):
        # TODO conditional calls make it hard to send messages to the message queue
        if self.altitude() < self.turnStartAltitude:
            self.autoPilot.target_pitch_and_heading(90, 90)
        elif self.turnStartAltitude < self.altitude() and self.altitude() < self.turnEndAltitude:
            frac = utils.normalizeToRange(self.altitude(), self.turnStartAltitude, self.turnEndAltitude)
            self.autoPilot.target_pitch_and_heading(90 * (1-frac), 90)
        else:
            self.autoPilot.target_pitch_and_heading(0, 90)

        if self.apoapsis() > self.targetAltitude:
            self.vessel.control.throttle = 0.0
            self.autoPilot.disengage()
            return True
        else:
            self.throttle()
            return False

    def displayValues(self):
        return [self.prettyName]