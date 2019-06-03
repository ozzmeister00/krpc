"""
Contains all the helper functions and programs for launching a craft
"""
from __future__ import print_function, absolute_import, division

import math

from . import utils
from . import throttle
from . import maths


def calculateTimeToLaunch(connection, vessel, targetInclination, longitudeOfAscendingNode):
    """
    Given the input inclination and longitude of ascending node, figure out how long
    we have to wait until we can launch as we launch into the plane defined by the inclination
    and Longitude of Ascending Node

    :param connection: krpc.Connection to operate upon
    :param vessel: vessel to test for
    :param targetInclination: the inclination of our target orbit, in degrees
    :param longitudeOfAscendingNode: the longitude of the ascending node we want to launch into, in radians

    :return: the universal time at which we should launch the vessel to meet our given orbit conditions
    """
    # figure out the point on the other side, in case it's closer
    longitudeOfDescendingNode = (longitudeOfAscendingNode + math.pi) % (2 * math.pi)

    # figure out where our ship is relative to the orbiting body's rotation angle
    currentSolarLongitude = math.radians(vessel.flight().longitude) + vessel.orbit.body.rotation_angle

    # figure out how far it is to each launch window
    distanceToLAN = longitudeOfAscendingNode - currentSolarLongitude
    distanceToLDN = longitudeOfDescendingNode - currentSolarLongitude

    distToNode = distanceToLAN
    targetNode = longitudeOfAscendingNode

    # we're closer in time to the LDN
    if distanceToLAN < 0:
        targetInclination *= -1
        distToNode = distanceToLDN
        targetNode = longitudeOfDescendingNode

    timeToNode = distToNode / vessel.orbit.body.rotational_speed

    timeOfNode = timeToNode + connection.space_center.ut

    return timeOfNode, targetInclination


class LaunchAzimuthCalculator(object):
    """
    A calculator class to figure out where our vessel should be pointing to reach an orbit at the
    target altitude with the target inclination
    """
    def __init__(self, targetAltitude, targetInclination, connection=None, vessel=None):
        """
        Calculate the launch azimuth (heading from launch site) to achieve an orbit
        at the given altitude and inclination

        :param targetAltitude: How high we want the apoapsis of our launch orbit
        :param targetInclination: how inclined we want our orbit when we reach that target altitude
        :param connection: the krpc.Connection to use for calculation. Will default to defaultConnection
        :param vessel: the vessel to operate upon, by default will use the active vessel on the connection
        """
        if not connection:
            connection = utils.defaultConnection("CalculateLaunchAzimuth")
        if not vessel:
            vessel = connection.space_center.active_vessel

        self.connection = connection
        self.vessel = vessel

        # sanity check
        if targetAltitude <= 0:
            raise ValueError("Orbital altitude can't be below sea level")

        # figure out where we're starting
        self.launchLatitude = vessel.flight().latitude

        # Determines whether we're trying to launch from the ascending or descending node
        self.ascending = True
        if targetInclination < 0:
            self.ascending = False
            # We'll make it positive for now and convert to southerly heading later
            targetInclination = abs(targetInclination)

        # Orbital inclination can't be less than launch latitude or greater than 180 - launch latitude
        if abs(self.launchLatitude) > targetInclination:
            targetInclination = abs(self.launchLatitude)
            print("Inclination impossible from current latitude, setting for lowest possible inclination.")

        if 180 - abs(self.launchLatitude) < targetInclination:
            targetInclination = 180 - abs(self.launchLatitude)
            print("Inclination impossible from current latitude, setting for highest possible inclination.")

        # One-time calculations
        self.equatorialVel = (2 * math.pi * vessel.orbit.body.equatorial_radius) / vessel.orbit.body.rotational_period

        self.targetOrbVel = math.sqrt(vessel.orbit.body.gravitational_parameter /
                                      (vessel.orbit.body.equatorial_radius + targetAltitude))

        # stash off our targets
        self.targetInclination = targetInclination
        self.targetAltitude = targetAltitude

    def __call__(self):
        inertialAzimuth = math.asin(max(min(math.cos(math.radians(self.targetInclination)) /
                                            math.cos(math.radians(self.vessel.flight().latitude)), 1), -1))

        VXRot = (self.targetOrbVel * math.sin(inertialAzimuth)) - \
                (self.equatorialVel * math.cos(math.radians(self.launchLatitude)))
        VYRot = self.targetOrbVel * math.cos(inertialAzimuth)

        # This clamps the result to values between 0 and 360.
        Azimuth = (math.degrees(math.atan2(VXRot, VYRot)) + 360) % 360

        if not self.ascending:
            if Azimuth <= 90:
                Azimuth = 180 - Azimuth
            elif Azimuth >= 270:
                Azimuth = 540 - Azimuth

        return Azimuth


class Ascend(utils.Program):
    """
    Program object to launch a vessel into orbit with the given parameters
    """
    def __init__(self, connection, vessel, targetAltitude, targetInclination=0.0):
        """

        :param connection: The connection to operate upon
        :param vessel: the vessel to launch
        :param targetAltitude: how high we want the apoapsis of our launch to be
        :param targetInclination: how much we want our orbit inclined
        """
        super(Ascend, self).__init__('Ascend')

        self.connection = connection
        self.vessel = vessel
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)
        self.turnStartAltitude = 250  # TODO allow users to tune this

        # End the turn within the target's atmosphere (if one exists),
        # otherwise end the turn at 25% of the target altitude
        self.turnEndAltitude = vessel.orbit.body.atmosphere_depth * 0.75 if vessel.orbit.body.atmosphere_depth > 1 else targetAltitude * .25
        #self.turnEndAltitude = targetAltitude * .5
        self.targetAltitude = targetAltitude

        # TODO allow users to tune this
        self.maxQ = 30000


        self.targetInclination = targetInclination

        # set up the launch azimuth calculator
        self.lazCalc = LaunchAzimuthCalculator(targetAltitude, targetInclination, connection, vessel)

        # set up some helpful data streams from the connection
        self.ut = connection.add_stream(getattr, connection.space_center, 'ut')
        self.altitude = connection.add_stream(getattr, self.flight, 'mean_altitude')
        self.apoapsis = connection.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        self.periapsis = connection.add_stream(getattr, self.vessel.orbit, 'periapsis_altitude')
        self.eccentricity = connection.add_stream(getattr, self.vessel.orbit, 'eccentricity')

        # set up our throttle controller so we don't experience too much force and waste fuel
        self.throttle = throttle.MaxQController(connection, vessel, maxQ=self.maxQ)

        # set the initial state of the vessel
        self.vessel.control.sas = False
        self.vessel.control.rcs = False
        self.vessel.control.throttle = 1.0

        # set up the autopilot
        self.autoPilot = vessel.auto_pilot
        self.autoPilot.reference_frame = vessel.surface_reference_frame
        self.autoPilot.target_pitch_and_heading(90, 90)  # start out pointing straight up
        self.autoPilot.target_roll = float("nan")
        self.autoPilot.engage()

    def __call__(self):
        # point straight up until we get to our turn start altitude
        if self.altitude() < self.turnStartAltitude:
            self.autoPilot.target_pitch_and_heading(90, 90)

        # if we're between turn start and turn end, lerp our pitch between 90 and 0
        elif self.turnStartAltitude < self.altitude() and self.altitude() < self.turnEndAltitude:
            frac = maths.normalizeToRange(self.altitude(), self.turnStartAltitude, self.turnEndAltitude)
            self.autoPilot.target_pitch_and_heading(90 * (1-frac), self.lazCalc())

        # if we're done with our gravity turn, stay parallel to the body's surface
        else:
            self.autoPilot.target_pitch_and_heading(0, self.lazCalc())

        if self.apoapsis() > self.targetAltitude:
            self.vessel.control.throttle = 0.0
            self.autoPilot.disengage()
            return True
        else:
            self.throttle()
            return False

    def displayValues(self):
        return [self.prettyName]
