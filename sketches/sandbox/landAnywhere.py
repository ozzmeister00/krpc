import time

import krpc

from sandbox.gui_testing import Display
from sandbox.utils import Program
from sandbox.maneuvers import changePeriapsis, changeApoapsis, ExecuteManeuver
from sandbox.hover_featured import Hover

OUTLIST = ['time,stopDist,altitude,idealThrottle,verticalSpeed,']

class Descend(Program):
    def __init__(self, vessel, connection):
        super(Descend, self).__init__("Descend")

        self.vessel = vessel

        self.auto_pilot = vessel.auto_pilot
        self.control = vessel.control
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)

        self.surface_altitude = connection.add_stream(getattr, self.flight, 'surface_altitude')

        self.maxDecel = 0
        self.stopDist = 0
        self.idealThrottle = 0
        self.impactTime = 0

        self.slamming = False

        self.auto_pilot.reference_frame = self.vessel.orbit.body.reference_frame
        self.auto_pilot.target_direction = self.flight.retrograde
        self.auto_pilot.target_roll = float("nan")
        self.auto_pilot.engage()

    def __call__(self):
        self.maxDecel = (self.vessel.available_thrust / self.vessel.mass) - self.vessel.orbit.body.surface_gravity
        self.stopDist = (self.flight.vertical_speed ** 2) / (2 * self.maxDecel)
        self.idealThrottle = self.stopDist / (self.surface_altitude() + 10)
        self.impactTime = self.surface_altitude() / abs(self.flight.vertical_speed)

        self.auto_pilot.target_direction = self.flight.retrograde

        if self.surface_altitude() + 10 < self.stopDist:
            self.slamming = True
            self.control.throttle = self.idealThrottle #<- This value isn't quite aggressive enough
        elif self.slamming:
            self.control.throttle = (self.idealThrottle + 1) / 2
        else:
            self.control.throttle = 0.0


        if self.flight.vertical_speed > -0.1:
            self.messages.appendleft("Vertical speed below threshold")
            self.messages.appendleft("Stopped at " + str(self.surface_altitude()))
            self.control.throttle = 0.0
            self.control.sas = True
            self.auto_pilot.disengage()
            return False

        return True

    def displayValues(self):
        return [self.prettyName,
                'mxDcl: ' + str(round(self.maxDecel, 2)),
                'stpDs: ' + str(round(self.stopDist, 2)),
                'thrtl: ' + str(round(self.idealThrottle, 2)),
                'ttimp: ' + str(round(self.impactTime, 2))
                ]

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

    # # TODO check our situation before trying to do all this
    # # get ourselves into a 30km x 30km parking orbit
    # lowerPeriapsisNode = changePeriapsis(vessel, ut(), 30000)
    # lowerPeriapsis = ExecuteManeuver(connection, vessel, node=lowerPeriapsisNode)
    # display.changeProgram(lowerPeriapsis)
    # while not lowerPeriapsis():
    #     display()
    #     time.sleep(0.01)
    #
    # lowerApoapsisNode = changeApoapsis(vessel, ut(), 30000)
    # lowerApoapsis = ExecuteManeuver(connection, vessel, node=lowerApoapsisNode)
    # display.changeProgram(lowerApoapsis)
    # while not lowerApoapsis():
    #     display()
    #     time.sleep(0.01)

    # deorbit to to PE = -(0.5 * body.radius) # TODO pick where the deorbit burn happens?
    # deorbitPeriapsisHeight = radius * -0.5
    # deorbitPeriapsisNode = changePeriapsis(vessel, ut() - 300, deorbitPeriapsisHeight)
    # deorbit = ExecuteManeuver(connection, vessel, node=deorbitPeriapsisNode)
    # display.changeProgram(deorbit)
    # display.addMessage("Deorbit burn")
    # while not deorbit():
    #     display()
    #     time.sleep(0.01)

    # TODO is warping to descend?

    # hoverslam
    descend = Descend(vessel, connection)
    display.changeProgram(descend)
    while descend():
        display()
        time.sleep(0.01)

    display.addMessage("moving to hover")
    display()

    vessel.control.gear = True

    # hover = Hover(connection, vessel, targetAlt=flight.surface_altitude)
    # display.changeProgram(hover)
    # while hover():
    #     display()
    #     time.sleep(0.01)
    #
    # vessel.control.throttle = 0.0
    # vessel.control.sas = True

    display.addMessage("landed")
    display()

    input("?")




if __name__ == '__main__':
    main()