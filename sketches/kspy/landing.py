"""
Stores all the functions we need to land a craft
"""
from __future__ import print_function, absolute_import, division

import math
import numpy as np
import time

from . import utils
from . import maths


def getLandingReferenceFrame(landingLongitude,
                             landingLatitude,
                             landingAltitude=None,
                             connection=None,
                             vessel=None,
                             body=None):
    """
    Constructs a reference frame object based on the vessel, body, and landing lat/long we're aiming for

    Check https://www.reddit.com/r/Kos/comments/45amle/landing_at_target/ for implementation details.

    :param landingLongitude: the longitude of our landing target
    :param landingLatitude: the latitude of our landing target
    :param landingAltitude: If none, will use the terrain altitude at the given lat/long

    :param connection: The connection to operate upon
    :param vessel: the vessel to check for
    :param body: the body we're orbiting

    :return: the constructed reference frame
    """

    # init default values
    if not connection:
        connection = utils.defaultConnection("getLandingReferenceFrame")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not body:
        body = vessel.orbit.body
    if not landingAltitude:
        landingAltitude = body.surface_height(landingLatitude, landingLongitude)

    # sneak the namespace in to make creation later easier
    ReferenceFrame = connection.space_center.ReferenceFrame

    # Determine landing site reference frame (orientation: x=zenith, y=north, z=east)
    landing_position = body.surface_position(landingLatitude, landingLongitude, body.reference_frame)
    q_long = (0, math.sin(-landingLongitude * 0.5 * math.pi / 180), 0, math.cos(-landingLongitude * 0.5 * math.pi / 180))
    q_lat = (0, 0, math.sin(landingLatitude * 0.5 * math.pi / 180), math.cos(landingLatitude * 0.5 * math.pi / 180))
    landing_reference_frame = ReferenceFrame.create_relative(
                                ReferenceFrame.create_relative(
                                  ReferenceFrame.create_relative(
                                    body.reference_frame,
                                    landing_position,
                                    q_long),
                                    (0, 0, 0),
                                    q_lat),
                                    (landingAltitude, 0, 0)
                                )

    # Up, North, East
    vessel.velocity(landing_reference_frame)
    return landing_reference_frame


def coordsDownBearing(lat, lon, bearing, distance, body):
    """
    Provided by Art Whaley

    Takes a latitude, longitude and bearing in degrees, and a
    distance in meters over a given body.  Returns a tuple
    (latitude, longitude) of the point you've calculated.

    :param lat: the latitude we're at
    :param lon: the longitude we're at
    :param bearing: the given heading
    :param distance: how far away we want to check from the input point
    :param body: the body we're checking for

    :return: (latitude, longitude) of the point x distance away down the bearing from the input lat/long
    """
    bearing = math.radians(bearing)
    R = body.equatorial_radius
    lat = math.radians(lat)
    lon = math.radians(lon)

    lat2 = math.asin(math.sin(lat) * math.cos(distance / R) + math.cos(lat) *
                     math.sin(distance / R) * math.cos(bearing))

    lon2 = lon + math.atan2(math.sin(bearing) * math.sin(distance / R) * math.cos(lat),
                            math.cos(distance / R) - math.sin(lat) * math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return lat2, lon2


def checkTerrain(lat1, lon1, lat2, lon2, body):
    """
    Returns an estimate of the highest terrain altitude between
    two latitude / longitude points.

    :param lat1: latitude of the first point, in degrees
    :param lon1: longitude of the first point, in degrees
    :param lat2: latitude of the second point, in degrees
    :param lon2: longitude of the second point, in degrees
    :param body: the body upon which to check

    :return: an estimate of the highest terrain altitude from COM between those two points
    """
    lat = lat1
    lon = lon1
    highestLat = lat
    highestLon = lon
    highestAlt = body.surface_height(lat, lon)
    latstep = (lat2 - lat1) / 20
    lonstep = (lon2 - lon1) / 20

    # test 20 points between where we started and where we're going
    for x in range(20):
        testAlt = body.surface_height(lat, lon)
        if testAlt > highestAlt:
            highestLat = lat
            highestLon = lon
            highestAlt = testAlt
        lat = lat + latstep
        lon = lon + lonstep

    return highestAlt


class SuicideBurnCalculator(object):
    """
    Calculator object used to determine the correct throttle value
    for the input vessel to perform a suicide burn so that when surface_altitude = 0, velocity = 0
    """
    def __init__(self, connection, vessel, altitude):
        """

        :param connection: krpc Connection object to work with
        :param vessel: vessel to work with
        :param altitude: highest terrain point under the path of the vessel
        """
        self.connection = connection
        self.vessel = vessel
        self.spaceCenter = connection.space_center  # stash off the space center to work with it later

        # initialize our variables
        self.burnTime = np.inf  # when to burn (in Universal Time))
        self.burnDuration = np.inf  # for how long to but
        self.groundTrack = np.inf  # height over the ground
        self.effectiveDeceleration = np.inf  # the effective ability for this vessel to decelerate
        self.radius = np.inf  # the radius of the body
        self.timeToImpact = np.inf  # how long until we crash, given our current course and speed
        self.timeToBurn = np.inf

        self.altitude = altitude

        self.desiredThrottle = 0.95

        self.referenceFrame = self.spaceCenter.ReferenceFrame.create_hybrid(
            position=vessel.orbit.body.reference_frame,
            rotation=vessel.surface_reference_frame
        )

    def __call__(self):
        # grab the body's reference frame
        referenceFrame = self.vessel.orbit.body.reference_frame

        vesselVelocity = self.vessel.velocity(self.referenceFrame)
        horizontalVelocity = (0, 0, 1)
        self.angleFromHorizontal = maths.angleBetween(vesselVelocity, horizontalVelocity)
        sine = math.sin(self.angleFromHorizontal)

        g = self.vessel.orbit.body.surface_gravity
        thrust = (self.vessel.max_thrust / self.vessel.mass)

        # is it because of this line?
        self.effectiveDeceleration = .5 * (-2 * g * sine + math.sqrt((2 * g * sine) *
                                                                     (2 * g * sine) + 4 *
                                                                     (thrust * thrust - g * g)))

        self.decelerationTime = self.vessel.flight(self.referenceFrame).speed / self.effectiveDeceleration
        # TODO: I'm not sure this is getting calculated right, as we end up cutting off so high
        # TODO: and any time we spend below about 95% throttle we might be using too much fuel

        # the height of the highest point over our ground path from the body's COM
        radius = self.vessel.orbit.body.equatorial_radius + self.altitude
        TA = self.vessel.orbit.true_anomaly_at_radius(radius) * -1
        self.timeToImpact = self.vessel.orbit.ut_at_true_anomaly(TA) - self.spaceCenter.ut
        self.timeToBurn = self.timeToImpact - self.decelerationTime

        impactTime = self.vessel.orbit.ut_at_true_anomaly(TA)
        burnTime = impactTime - self.decelerationTime / 2
        self.groundTrack = ((burnTime - self.spaceCenter.ut) * self.vessel.flight(self.referenceFrame).speed) + \
                           (.5 * self.vessel.flight(self.referenceFrame).speed * self.decelerationTime)

        self.desiredThrottle = self.decelerationTime / self.timeToImpact


class Descend(utils.Program):
    """
    Program object to handle landing a vessel on a suborbital path over a body
    """
    def __init__(self, connection, vessel):
        """
        :param connection: The krpc.Connection to operate upon
        :param vessel: The vessel to try and land
        """
        super(Descend, self).__init__("Descend")

        self.vessel = vessel
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)

        self.sbc = SuicideBurnCalculator(connection, vessel, 5000)
        self.sbc()  # init call of the SBC

        # find the highest point over our current ground path and use that to calculate our suicide burn, for safety
        touchdown = coordsDownBearing(self.flight.latitude, self.flight.longitude, (180 + self.flight.heading),
                                      self.sbc.groundTrack, self.vessel.orbit.body)

        self.sbc.altitude = checkTerrain(self.flight.latitude, self.flight.longitude, touchdown[0], touchdown[1],
                                         self.vessel.orbit.body)
        self.sbc()

        self.burning = False

        # point surface velocity retrograde the entire time
        self.vessel.auto_pilot.reference_frame = self.vessel.surface_velocity_reference_frame
        self.vessel.auto_pilot.target_direction = (0, -1, 0)
        self.vessel.auto_pilot.target_roll = float("nan")
        self.vessel.auto_pilot.engage()

    def __call__(self):

        # update the max safe altitude as our burn progresses
        touchdown = coordsDownBearing(self.flight.latitude, self.flight.longitude, (180 + self.flight.heading),
                                      self.sbc.groundTrack, self.vessel.orbit.body)

        self.sbc.altitude = checkTerrain(self.flight.latitude, self.flight.longitude, touchdown[0], touchdown[1],
                                         self.vessel.orbit.body)

        self.sbc()  # call the SBC to update itself

        # one-time call to check if it's time to burn
        if self.sbc.timeToBurn <= 0.0:
            self.burning = True

        # throttle should be proportional to how long we have to burn verses how long until we hit the ground
        if self.burning:
            self.vessel.control.throttle = self.sbc.desiredThrottle

        # once we're slow enough, move to the next mode
        if self.flight.speed < 10.0:
            return False

        return True

    def displayValues(self):
        return [self.prettyName]


class SoftTouchdown(utils.Program):
    """
    Program object to take us from the suicide burn/descend program (which seems to leave us a little high)
    all the way down to softly touch down on the surface of the body we're orbiting
    """
    def __init__(self, vessel):
        """
        :param vessel: the vessel to guide to a soft touchdown
        """
        super(SoftTouchdown, self).__init__("SoftLanding")

        self.vessel = vessel
        self.autoPilot = vessel.auto_pilot
        self.control = vessel.control
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)

        # we still want to point surface velocity retrograde because at this point that shound be straight up
        self.autoPilot.reference_frame = self.vessel.surface_velocity_reference_frame
        self.autoPilot.target_direction = (0, -1, 0)
        self.autoPilot.target_roll = float("nan")
        self.autoPilot.engage()

    def __call__(self):
        # our maximum descent speed should be 10% of the distance to the ground
        safeDescent = self.flight.surface_altitude / -10
        safeDescent = max(safeDescent, -15)  # but also clamp it, in case we're too far up or moving too quickly

        # the midpoint of our safe descent should be a hovering throttle
        a = self.vessel.orbit.body.surface_gravity - self.flight.vertical_speed
        F = self.vessel.mass * a

        midPoint = (F / self.vessel.available_thrust)

        # rudimentary proportional controller
        e = safeDescent - self.flight.vertical_speed
        p0 = midPoint
        ekp = .25 * e
        self.control.throttle = ekp + p0

        # if we're on the ground, or if we're moving really slowly, otherwise we go in a wonky direction really quickly
        if self.vessel.situation == self.vessel.situation.landed or abs(self.flight.vertical_speed) < 0.1:
            return False

        return True

    def displayValues(self):
        return [self.prettyName]


class Hover(utils.Program):
    """
    A program object to allow the user to nicely hover and navigate around at a defined altitude
    """
    def __init__(self, connection, vessel, targetAlt=12):
        """

        :param connection: the krpc.Connection to work with
        :param vessel: the vessel to hover
        :param targetAlt: our initial target hovering altitude (from COM to terrain)
        """
        super(Hover, self).__init__('Hover')

        self.connection = connection
        self.vessel = vessel
        self.flight = self.vessel.flight(self.vessel.orbit.body.reference_frame)
        self.control = self.vessel.control

        self.targetAlt = targetAlt
        self.descentRate = 0.1
        self.landingAlt = 10

        # TODO: I think these values are a little too responsive
        # set our hovering control parameters
        self.horizSpeedMax = 10.0
        self.horizDeflectionMax = 45.0
        self.horizSpeedTolerance = 0.1

        # set our initial state
        self.vessel.control.sas = False
        self.vessel.control.rcs = False
        self.vessel.control.throttle = 0.0
        self.vessel.control.gear = False
        self.vessel.control.legs = True
        self.vessel.control.wheels = False

        self.vessel.auto_pilot.reference_frame = self.vessel.orbit.body.reference_frame
        self.vessel.auto_pilot.target_direction = (0, 0, 1)
        self.vessel.auto_pilot.target_roll = float('nan')
        self.vessel.auto_pilot.engage()

        self.midPitch = 0.0
        self.midYaw = 0.0

    def __call__(self):

        # update our debug drawings
        self.connection.drawing.clear()

        # allow the user to hit abort and softly and slowly land the craft
        if utils.hasAborted(self.vessel):
            self.control.gear = True
            self.targetAlt -= 1.0
            time.sleep(1)
            if self.vessel.situation == self.vessel.situation.landed:
                self.control.abort = False
                return False

        # hitting the brakes means killing horizontal velocity
        if self.control.brakes:
            up, back, right = self.flight.velocity

            pitchOffset = (right / self.horizSpeedMax) * self.horizDeflectionMax
            yawOffset = (back / self.horizSpeedMax) * self.horizDeflectionMax

            # clamp to range
            pitchOffset = maths.clamp(pitchOffset, -self.horizDeflectionMax, self.horizDeflectionMax)
            yawOffset = maths.clamp(yawOffset, -self.horizDeflectionMax, self.horizDeflectionMax)

            pitch = self.midPitch - pitchOffset
            yaw = self.midYaw - yawOffset

            direction = maths.rpyToDirection(pitch, yaw)

            self.vessel.auto_pilot.target_direction = direction
            self.vessel.auto_pilot.target_roll = float("nan")

            # turn off the brakes if we're broken enough
            if abs(self.flight.horizontal_speed) <= self.horizSpeedTolerance:
                self.control.brakes = False

        else:
            self.vessel.auto_pilot.target_direction = (0, 0, 1)

        # add some debug drawings to help diagnose any potential over-correction issues
        self.connection.drawing.add_direction(self.vessel.direction(self.vessel.surface_reference_frame),
                                              self.vessel.surface_reference_frame)
        self.connection.drawing.add_direction(self.vessel.auto_pilot.target_direction,
                                              self.vessel.surface_reference_frame)

        # the different between where we want to be and where we are
        altError = self.targetAlt - self.flight.surface_altitude

        # acceleration
        a = self.vessel.orbit.body.surface_gravity - self.flight.vertical_speed + altError

        # Compute throttle setting using newton's law F=ma
        F = self.vessel.mass * a

        # if we've run out of fuel, bail out
        if not self.vessel.available_thrust:
            return False

        # set the vessel's throttle to what it should be
        self.control.throttle = F / self.vessel.available_thrust
        return True

    def displayValues(self):
        return [self.prettyName]
