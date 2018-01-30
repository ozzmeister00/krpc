import krpc
from math import cos, sin
import time

#from krpc.utils import PID, hasAborted

connection = krpc.connect("Hover")
vessel = connection.space_center.active_vessel
flight = vessel.flight(vessel.orbit.body.reference_frame)

g = vessel.orbit.body.surface_gravity

horizontalDeflectionMax = 45.0
horizontalSpeedTolerance = 0.1
horizontalSpeedMax = 15.0

connection.drawing.add_direction((1,0,0), vessel.orbit.body.reference_frame, length=20)

def hasAborted(vessel):
    return vessel.control.abort


def main():
    targetAlt = 15.0

    up = 1.0
    midPitch = 0.0
    midYaw = 0.0

    #( 1, 0, 0) <- UP
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

        # Kill horizontal velocity
        x, y, z = flight.velocity


        if abs(flight.horizontal_speed) > 1.0:

            pitchOffset = (y / 25) * horizontalDeflectionMax
            yawOffset = (x / 25) * horizontalDeflectionMax

            pitch = midPitch + pitchOffset
            yaw = midYaw + yawOffset

            # https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector

            x = cos(yaw) * cos(pitch)
            y = sin(yaw) * cos(pitch)
            z = sin(pitch)

            direction = (x, y, z)


            #vessel.control.auto_pilot.target_direction = direction

            connection.drawing.add_direction(direction, vessel.surface_reference_frame)

        else:
            connection.drawing.add_direction((1, 0, 0), vessel.surface_reference_frame)


        alt_error = targetAlt - flight.surface_altitude

        a = g - flight.vertical_speed + alt_error

        # Compute throttle setting using newton's law F=ma
        F = vessel.mass * a
        vessel.control.throttle = F / vessel.available_thrust

        time.sleep(0.01)

    vessel.control.abort = False
    vessel.control.throttle = 0.0
    vessel.control.sas = True


if __name__ == '__main__':
    main()
