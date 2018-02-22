import time
from collections import deque
from math import cos, sin, radians, degrees, atan2, asin

def gHere(body, vessel):
    return body.surface_gravity * (body.mass / ((vessel.flight(body.reference_frame).mean_altitude + body.equatorial_radius)**2))

def fgHere(body, vessel):
    return vessel.mass * gHere(body, vessel)

def hasAborted(vessel):
    return vessel.control.abort

def normalizeToRange(v, a, b):
    return (v - a) / (b - a)

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

class AutoStage(object):
    def __init__(self, vessel):
        self.vessel = vessel

    def __call__(self):
        stage = self.vessel.control.current_stage
        parts = self.vessel.parts.in_decouple_stage(stage-1)

        for part in parts:
            engine = part.engine
            if engine and engine.active and engine.has_fuel:
                return

        self.vessel.control.activate_next_stage()


class Fairing(object):
    def __init__(self, connection, vessel, deployAtms=0.001):
        flight = vessel.flight(vessel.orbit.body.reference_frame)
        self.atms = connection.add_stream(getattr, flight, 'atmosphere_density')
        self.deployAtms = deployAtms

        self.fairings = [part for part in vessel.parts.with_tag('deployFairing')]
        self.deployed = False

    def __call__(self):
        if self.atms() <= self.deployAtms and not self.deployed:
            for fairing in self.fairings:
                try:
                    fairing.fairing.jettison()
                except:
                    pass
            self.deployed = True


def clamp(v, minV, maxV):
    return max(minV, min(v, maxV))

class PID(object):
    def __init__(self, kP=1, kI=0, kD=0, dt=1, cMin=0.0, cMax=1.0):
        self.seekP = 0.0

        self.kP = kP
        self.kI = kI
        self.kD = kD

        self.cMin = cMin
        self.cMax = cMax

        self.ti = 0.0
        self.oldTime = time.time()

        self.lastPosition = 0.0

    def __call__(self, seekV, currV):
        error = seekV - currV

        dT = time.time() - self.oldTime
        P = self.kP
        I = self.kI * dT
        D = self.kD / dT

        self.ti += I * error
        self.ti = clamp(self.ti, self.cMin, self.cMax)
        dInput = currV - self.lastPosition
        output = P * error + self.ti - D * dInput
        output = clamp(self.ti, self.cMin, self.cMax)

        self.lastPosition = currV

        return output

class Program(object):
    def __init__(self, prettyName):
        self.prettyName = prettyName
        self.messages = deque()

    def __call__(self):
        raise NotImplementedError("Sublcass of Program has not been set up correctly. Implement a __call__ method.")

    def displayValues(self):
        raise NotImplementedError("Sublcass of Program has not been set up correctly. Implement a displayValues method.")