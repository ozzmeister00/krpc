from __future__ import absolute_import, print_function, division

import time
import krpc

from sandbox.utils import Program, hasAborted, clamp, rpyToDirection
from sandbox.gui_testing import Display

# TODO add action groups to change target altitude
# TODO add display values

class Hover(Program):
    def __init__(self, connection, vessel, targetAlt=12):
        super(Hover, self).__init__('Hover')

        self.connection = connection
        self.vessel = vessel
        self.flight = self.vessel.flight(self.vessel.orbit.body.reference_frame)
        self.control = self.vessel.control

        self.targetAlt = targetAlt
        self.descentRate = 0.1
        self.landingAlt = 10

        # TODO: I think these values are a little too responsive
        self.horizSpeedMax = 10.0
        self.horizDeflectionMax = 45.0
        self.horizSpeedTolerance = 0.1

        self.vessel.control.sas = False
        self.vessel.control.rcs = False
        self.vessel.control.throttle = 0.0
        self.vessel.control.gear = False
        self.vessel.control.legs = True
        self.vessel.control.wheels = False

        self.vessel.auto_pilot.reference_frame = self.vessel.orbit.body.reference_frame
        self.vessel.auto_pilot.target_direction = (0, 0, 1)
        self.vessel.auto_pilot.target_roll = float('nan')
        self.vessel.auto_pilot.engage()

        self.midPitch = 0.0
        self.midYaw = 0.0

    def __call__(self):
        self.connection.drawing.clear()

        if hasAborted(self.vessel):
            self.control.gear = True
            self.targetAlt -= 1.0
            time.sleep(1)
            if self.vessel.situation == self.vessel.situation.landed:
                self.control.abort = False
                return False

        if self.control.brakes:

            # Kill horizontal velocity
            up, back, right = self.flight.velocity

            pitchOffset = (right / self.horizSpeedMax) * self.horizDeflectionMax
            yawOffset = (back / self.horizSpeedMax) * self.horizDeflectionMax

            # clamp to range
            pitchOffset = clamp(pitchOffset, -self.horizDeflectionMax, self.horizDeflectionMax)
            yawOffset = clamp(yawOffset, -self.horizDeflectionMax, self.horizDeflectionMax)

            pitch = self.midPitch - pitchOffset
            yaw = self.midYaw - yawOffset

            direction = rpyToDirection(pitch, yaw)

            self.vessel.auto_pilot.target_direction = direction
            self.vessel.auto_pilot.target_roll = float("nan")

            # turn off the brakes if we're broken enough
            if abs(self.flight.horizontal_speed) <= self.horizSpeedTolerance:
                self.control.brakes = False

        else:
            self.vessel.auto_pilot.target_direction = (0, 0, 1)

        self.connection.drawing.add_direction(self.vessel.direction(self.vessel.surface_reference_frame),
                                              self.vessel.surface_reference_frame)
        self.connection.drawing.add_direction(self.vessel.auto_pilot.target_direction,
                                              self.vessel.surface_reference_frame)

        alt_error = self.targetAlt - self.flight.surface_altitude

        a = self.vessel.orbit.body.surface_gravity - self.flight.vertical_speed + alt_error

        # Compute throttle setting using newton's law F=ma
        F = self.vessel.mass * a

        if not self.vessel.available_thrust:
            return False

        self.control.throttle = F / self.vessel.available_thrust
        return True

    def displayValues(self):
        return [self.prettyName]


def main():
    connection = krpc.connect("Hover")
    vessel = connection.space_center.active_vessel
    hover = Hover(connection, vessel)
    display = Display(connection, vessel, program=hover)

    if not vessel.available_thrust:
        vessel.control.activate_next_stage()

    while hover():
        display()
        time.sleep(0.01)

    vessel.control.throttle = 0.0
    vessel.control.sas = True


if __name__ == '__main__':
    main()
