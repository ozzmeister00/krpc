import krpc
import math
import time

connection = krpc.connect("Launcher")
vessel = connection.space_center.active_vessel

ut = connection.add_stream(getattr, connection.space_center, 'ut')
altitude = connection.add_stream(getattr, vessel.flight(), 'mean_altitude')

seekAlt = 15
descentRate = 0.1
landingAlt = 10
horizSpeedMax = 15
horizDeflectionMax = 45
horizSpeedTolerance = 0.1

# lock shipLatLng to SHIP:GEOPOSITION.
# lock surfaceElevation to shipLatLng:TERRAINHEIGHT.
# lock betterALTRADAR to max(0.1, ALTITUDE - surfaceElevation).

def Fg_here():
    return 10.0

class Hover(object):
    def __init__(self, vessel):
        self.seekAlt = 15
        self.descentRate = 0.1
        self.landingAlt = 10
        self.vessel = vessel


        self.vessel.control.sas = False
        self.vessel.control.rcs = False
        self.vessel.control.throttle = 0.0
        self.vessel.control.gear = False
        self.vessel.control.legs = True
        self.vessel.control.wheels = False

        self.midThrottle = Fg_here()/self.availableThrust()

        self.pitchOffset = 0.0
        self.yawOffset = 0.0

        self.xOffset = 0.0
        self.yOffset = 0.0

        self.xPid = None
        self.yPid = None

        self.midPitch = 0.0
        self.midYaw = UP:Yaw

        self.targetPitch = self.midPitch + self.xOffset + self.pitchOffset
        self.targetYaw = self.midYaw + self.yOffset - self.yawOffset

        self.runMode = 1

    def stabilizer(self):
        self.pitchOffset = 0.0
        self.yawOffset = 0.0

    def killHorizontal(self):
        self.xOffset = 0.0
        self.yOffset = 0.0

    def goDown(self):
        self.seekAlt -= 1

    def goUp(self):
        self.seekAlt += 1

    def land(self):
        pass


def preflight():

    # make sure the vessel is ready to roll
    vessel.control.sas = True
    vessel.control.rcs = False
    vessel.control.throttle = 0.0
    vessel.control.gear = False
    vessel.control.legs = True
    vessel.control.wheels = False

    vessel.auto_pilot.engage()

    return 1


def userHasAborted():
    return vessel.control.abort

def programHasFinished(runmode):
    return runmode == sorted(runModes.keys())[-1] + 1

runModes = {0:preflight}

def main():
    runmode = 0

    while runmode >= 0:
        # if the current runmode has met its criteria, move on

        runmode = runModes[runmode]()

        # bail out!
        if userHasAborted():
            runmode = -1

        # aaaaand we're done
        if programHasFinished(runmode):
            runmode = -1

        time.sleep(.1)

    vessel.auto_pilot.disengage()


if __name__ == '__main__':
    main()