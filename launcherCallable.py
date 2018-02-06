import utils
import throttle

class Ascend(object):
    def __init__(self, conn, vessel, targetAltitude):
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