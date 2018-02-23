import time

import krpc

from sandbox.gui_testing import Display
from sandbox.utils import Program
from sandbox.maneuvers import changePeriapsis, changeApoapsis, ExecuteManeuver

def main():
    connection = krpc.connect("Landing")
    vessel = connection.space_center.active_vessel
    flight = vessel.flight(vessel.orbit.body.reference_frame)
    display = Display(connection, vessel)
    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    display()
    display.addMessage("Lowering to stable orbit")
    display()

    surface_altitude = connection.add_stream(getattr, flight, 'surface_altitude')

    g = vessel.orbit.body.gravitational_parameter

    radius = vessel.orbit.body.equatorial_radius

    # TODO check our situation before trying to do all this
    # get ourselves into a 30km x 30km parking orbit
    lowerPeriapsisNode = changePeriapsis(vessel, ut(), 30000)
    lowerPeriapsis = ExecuteManeuver(connection, vessel, node=lowerPeriapsisNode)
    display.changeProgram(lowerPeriapsis)
    while not lowerPeriapsis():
        display()
        time.sleep(0.01)

    lowerApoapsisNode = changeApoapsis(vessel, ut(), 30000)
    lowerApoapsis = ExecuteManeuver(connection, vessel, node=lowerApoapsisNode)
    display.changeProgram(lowerApoapsis)
    while not lowerApoapsis():
        display()
        time.sleep(0.01)

    # deorbit to to PE = -(0.5 * body.radius) # TODO pick where the deorbit burn happens?
    deorbitPeriapsisHeight = radius * -0.5
    deorbitPeriapsisNode = changePeriapsis(vessel, ut() + 300, deorbitPeriapsisHeight)
    deorbit = ExecuteManeuver(connection, vessel, node=deorbitPeriapsisNode)
    while not deorbit():
        display()
        time.sleep(0.01)

    # hoverslam
    surfaceFlight = vessel.flight(vessel.surface_reference_frame)
    vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
    vessel.auto_pilot.target_direction = surfaceFlight.retrograde
    vessel.auto_pilot.target_roll = float("nan")
    vessel.auto_pilot.engage()

    landed = False

    display.addMessage("Engaging hoverslam")
    display()
    vessel.auto_pilot.wait()

    display.addMessage("Aligned for burn")

    # TODO programize this so it displays messages correctly
    while not landed:

        vessel.auto_pilot.target_direction = surfaceFlight.retrograde

        # TODO impact time is working, but stop distnce isn't?
        maxDecel = (vessel.available_thrust / vessel.mass) - g
        stopDist = (flight.vertical_speed ** 2) / (2 * maxDecel)
        idealThrottle = stopDist / surface_altitude()
        impactTime = surface_altitude() / abs(flight.vertical_speed)

        if impactTime < 3.0:
            vessel.control.gear = True

        if surface_altitude() < stopDist:
            vessel.control.throttle = idealThrottle

        if abs(flight.vertical_speed) < 0.5:
            display.addMessage("vertical speed is less than 0.5")
            vessel.control.throttle = 0.0
            vessel.control.sas = True
            landed = True

        display()
        time.sleep(0.1)

    vessel.auto_pilot.disengage()

    display.addMessage("Done!")
    display()
    input("ping?")


if __name__ == '__main__':
    main()