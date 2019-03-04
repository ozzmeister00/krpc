"""
A collection of utilities for kspy
"""

from __future__ import print_function, absolute_import, division

import collections
import math
import time

import krpc

from . import maths


def defaultConnection(connectionName):
    """
    modify the default connection here, useful for  fire-and-forgetting to set the connection
    when running from the console

    #TODO it'd be helpful if this was a contextmanager so we could
    with defaultConnection("thing"):
        stuff

    :param connectionName: name of the connection that will appear in KSP
    :return: the connection object
    """
    return krpc.connect(connectionName)


def gHere(body, vessel):
    """
    Get the gravitational parameter for the input vessel orbiting the given body

    :param body: the body the vessel is orbiting
    :param vessel: the vessel to get the force of gravity on

    :return: the gravity parameter at the vessel's current altitude
    """
    return body.surface_gravity * (body.mass / ((vessel.flight(body.reference_frame).mean_altitude +
                                                 body.equatorial_radius) ** 2))


def fgHere(body, vessel):
    """
    Get the force of gravity on the input vessel orbiting the input body

    :param body: the body the vessel is orbiting
    :param vessel: the vessel to check

    :return: the force of gravity on the input vessel at its current altitude
    """
    return vessel.mass * gHere(body, vessel)


def hasAborted(vessel):
    """
    Test if the input vessel has triggered its abort actiongroup

    :param vessel: krpc.Vessel
    :return: Boolean if the vessel has set its abort actiongroup
    """
    return vessel.control.abort


def rpyToDirection(pitch, yaw):
    """
    Converts the input pitch and yaw values to a XYZ direction value

    :param pitch: pitch in degrees
    :param yaw: yaw in degrees
    :return: [X, Y, Z] direction, normalized
    """
    x = math.cos(math.radians(yaw)) * math.cos(math.radians(pitch))
    y = math.sin(math.radians(yaw)) * math.cos(math.radians(pitch))
    z = math.sin(math.radians(pitch))

    return x, y, z


def directionToRPY(direction):
    """
    Converts the input [X,Y,Z] direction value to Pitch and Yaw (in degrees)

    :param direction: 3-length list as [X, Y, Z]
    :return: pitch, yaw in degrees
    """
    x, y, z = direction
    pitch = math.degrees(math.asin(z))
    yaw = math.degrees(math.atan2(y, x))

    return pitch, yaw


class AutoStage(object):
    """
    A class to check and handle the need for autostaging and will return true
    if the vessel needs to stage based on fuel and engine values
    """
    def __init__(self, vessel):
        self.vessel = vessel

    def __call__(self):
        stage = self.vessel.control.current_stage
        parts = self.vessel.parts.in_decouple_stage(stage - 1)

        for part in parts:
            engine = part.engine
            if engine and engine.active and engine.has_fuel:
                return

        print("Staging")
        self.vessel.control.activate_next_stage()


class Fairing(object):
    """
    A class to check and handle the need to deploy fairings, but the fairings need to be
    tagged "deployFairing" in KSP
    """
    def __init__(self, connection, vessel, deployAtms=0.0001):
        """

        :param connection: krpc Connection object to check
        :param vessel: vessel to check for fairings on
        :param deployAtms: Minimum atmosphereic density threshold at which to deploy fairings
        """
        flight = vessel.flight(vessel.orbit.body.reference_frame)
        self.atms = connection.add_stream(getattr, flight, 'atmosphere_density')
        self.deployAtms = deployAtms

        self.fairings = []

        try:
            self.fairings = [part for part in vessel.parts.with_tag('deployFairing')]
        # if, for whatever reason, something goes wrong trying to find fairings, don't worry.
        except:
            pass

        self.deployed = False

    def __call__(self):
        if self.atms() <= self.deployAtms and not self.deployed:
            for fairing in self.fairings:
                try:
                    print("fairings")
                    fairing.fairing.jettison()
                # if, for whatever reason, something goes wrong, just ignore it.
                except:
                    pass
            self.deployed = True


class Abort(object):
    """
    A class to check and handle user-initiated abort sequence, when called
    will return True or False based on the abort condition of the vessel
    """
    def __init__(self, vessel):
        self.vessel = vessel

    def __call__(self):
        if self.vessel.control.abort:
            # disengage the throttle
            self.vessel.control.throttle = 0.0

            # activate the abort action group
            self.vessel.control.setAbort(True)

            return True

        return False


class PID(object):
    """
    A rudimentary PID controller class (unused in favor of Art Whaley's)
    """
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
        self.ti = maths.clamp(self.ti, self.cMin, self.cMax)
        dInput = currV - self.lastPosition
        output = P * error + self.ti - D * dInput
        output = maths.clamp(self.ti, self.cMin, self.cMax)

        self.lastPosition = currV

        return output


class Program(object):
    """
    The base class for all looping programs that we'll run to accomplish a given task
    Provides some additional functionality that we'll use later for displays
    """
    def __init__(self, prettyName):
        """

        :param prettyName: Name of the program, for eventual use in display
        """
        self.prettyName = prettyName
        self.messages = collections.deque()

    def __call__(self):
        raise NotImplementedError("Sublcass of Program has not been set up correctly. Implement a __call__ method.")

    def displayValues(self):
        raise NotImplementedError(
            "Sublcass of Program has not been set up correctly. Implement a displayValues method.")
