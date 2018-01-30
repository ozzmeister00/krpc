import math
import time

def gHere(body, vessel):
    return body.surface_gravity * (body.mass / ((vessel.flight(body.reference_frame).altitude + body.radius)**2))

def fgHere(body, vessel):
    return vessel.mass * gHere(body, vessel)

def hasAborted(vessel):
    return vessel.control.abort

class PID(object):
    def __init__(self, kP, kI, kD, cMin=0.0, cMax=1.0):
        self.seekP = 0.0

        self.kP = kP
        self.kI = kI
        self.kD = kD

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.cMin = cMin
        self.cMax = cMax

        self.t = time.time()
        self.oldI = 0.0

    def __call__(self, seekV, currV):
        P = seekV - currV
        newInput = self.oldI

        dT = time.time() - self.t
        self.D = (P - self.P) / dT
        onlyPD = (self.kP * P) + (self.kD * self.D)
        if (self.I > 0.0 or onlyPD > self.cMin) and \
                (self.I < 0 or onlyPD > self.cMax):
            self.I = self.I + (P * dT)
        newInput = onlyPD + (self.kI * self.I)

        newInput = max(self.cMin, min(self.cMax, newInput))

        self.P = P
        self.oldI = newInput

        return newInput