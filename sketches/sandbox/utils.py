import time
from collections import deque
from math import cos, sin, radians, degrees, atan2, asin, pi

def gHere(body, vessel):
    return body.surface_gravity * (
    body.mass / ((vessel.flight(body.reference_frame).mean_altitude + body.equatorial_radius) ** 2))


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
        parts = self.vessel.parts.in_decouple_stage(stage - 1)

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

        self.fairings = []

        try:
            self.fairings = [part for part in vessel.parts.with_tag('deployFairing')]
        except:
            pass

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
        raise NotImplementedError(
            "Sublcass of Program has not been set up correctly. Implement a displayValues method.")

def getLandingReferenceFrame(connection, body, vessel, landing_longitude, landing_latitude):

    ReferenceFrame = connection.space_center.ReferenceFrame

    # Define landing site
    landing_latitude = -(0+(5.0/60)+(48.38/60/60)) # Top of the VAB
    landing_longitude = -(74+(37.0/60)+(12.2/60/60))
    landing_altitude = 2000

    # Determine landing site reference frame (orientation: x=zenith, y=north, z=east)
    landing_position = body.surface_position(landing_latitude, landing_longitude, body.reference_frame)
    q_long = (0, sin(-landing_longitude * 0.5 * pi / 180), 0, cos(-landing_longitude * 0.5 * pi / 180))
    q_lat = (0, 0, sin(landing_latitude * 0.5 * pi / 180), cos(landing_latitude * 0.5 * pi / 180))
    landing_reference_frame = ReferenceFrame.create_relative(
                                ReferenceFrame.create_relative(
                                  ReferenceFrame.create_relative(
                                    body.reference_frame,
                                    landing_position,
                                    q_long),
                                (0,0,0),
                                q_lat),
                                (landing_altitude, 0, 0)
                                )

    # Up, North, East
    vessel.velocity(landing_reference_frame)
    return landing_reference_frame