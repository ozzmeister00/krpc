import krpc
from math import cos, sin, radians, degrees, atan2, asin
import time

connection = krpc.connect("Hover")
vessel = connection.space_center.active_vessel
flight = vessel.flight(vessel.orbit.body.reference_frame)

g = vessel.orbit.body.surface_gravity

horizontalDeflectionMax = 45.0
horizontalSpeedTolerance = 0.1
horizontalSpeedMax = 10.0

connection.drawing.add_direction((1,0,0), vessel.orbit.body.reference_frame, length=20)

def hasAborted(vessel):
    return vessel.control.abort


def main():
    targetAlt = 15.0

    midPitch = 0.0
    midYaw = 0.0

    UP = (1, 0, 0)

    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_roll = 0.0
    vessel.auto_pilot.auto_tune = False
    # (1, 0, 0) <- UP
    # (0, 0, 1) <- RIGHT
    # (0, 1, 0) <- BACK

    pitchOffset = 0.0
    yawOffset = 0.0

    if vessel.situation.pre_launch:
        vessel.control.throttle = 0.0
        vessel.control.activate_next_stage()

    while True:
        connection.drawing.clear()

        if hasAborted(vessel):
            targetAlt -= 1.0
            time.sleep(1)
            if vessel.situation == vessel.situation.landed:
                break

        if vessel.control.brakes:

            # Kill horizontal velocity
            up, back, right = flight.velocity

            pitchOffset = (right / horizontalSpeedMax) * horizontalDeflectionMax
            yawOffset = (back / horizontalSpeedMax) * horizontalDeflectionMax

            # clamp to range
            pitchOffset = clamp(pitchOffset, -horizontalDeflectionMax, horizontalDeflectionMax)
            yawOffset = clamp(yawOffset, -horizontalDeflectionMax, horizontalDeflectionMax)

            pitch = midPitch - pitchOffset
            yaw = midYaw - yawOffset

            direction = rpyToDirection(pitch, yaw)

            vessel.auto_pilot.target_direction = direction
            vessel.auto_pilot.target_roll = float("nan")

            # turn off the brakes if we're broken enough
            if abs(flight.horizontal_speed) <= horizontalSpeedTolerance:
                vessel.control.brakes = False

        else:
            vessel.auto_pilot.target_direction = UP

        connection.drawing.add_direction(vessel.direction(vessel.surface_reference_frame), vessel.surface_reference_frame)
        connection.drawing.add_direction(vessel.auto_pilot.target_direction,vessel.surface_reference_frame)

        alt_error = targetAlt - flight.surface_altitude

        a = g - flight.vertical_speed + alt_error

        # Compute throttle setting using newton's law F=ma
        F = vessel.mass * a
        vessel.control.throttle = F / vessel.available_thrust

        time.sleep(0.01)

    vessel.auto_pilot.disengage()
    vessel.control.abort = False
    vessel.control.throttle = 0.0
    vessel.control.sas = True

def rpyToDirection(pitch, yaw):
    x = cos(radians(yaw)) * cos(radians(pitch))
    y = sin(radians(yaw)) * cos(radians(pitch))
    z = sin(radians(pitch))

    return x, y, z

def directionToRPY(direction):
    x, y, z = direction
    pitch = degrees(asin(z))
    yaw = degrees(atan2(y, x))

    return pitch, yaw

def clamp(v, minV, maxV):
    return max(minV, min(v, maxV))

if __name__ == '__main__':
    main()
