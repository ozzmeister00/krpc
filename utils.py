import time

def gHere(body, vessel):
    return body.surface_gravity * (body.mass / ((vessel.flight(body.reference_frame).altitude + body.radius)**2))

def fgHere(body, vessel):
    return vessel.mass * gHere(body, vessel)

def hasAborted(vessel):
    return vessel.control.abort

def normalizeToRange(v, a, b):
    return (v - a) / (b - a)

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