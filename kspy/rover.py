"""
A collection of functions to handle surface traversal operations
"""

from __future__ import absolute_import, print_function, division

import time
import math

from .pid import PID

from . import maths
from . import utils


def headingForLatLon(target, location):
    """
    For a given target and current location, return the compass heading to that target

    :param target: latlon object for where we want to go
    :param location: latlon object for where we are
    :return:
    """
    lat1 = math.radians(location.lat)
    lat2 = math.radians(target.lat)

    diffLong = math.radians(target.lon - location.lon)

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
                                           * math.cos(lat2) * math.cos(diffLong))

    initialBearing = math.atan2(x, y)

    initialBearing = math.degrees(initialBearing)
    comparssBearing = (initialBearing + 360) % 360

    return comparssBearing


def distanceOverSurface(target, location, body):
    """
    Roughly approximate the distance over the input surface between two input locations

    :param target: latlon object for where we want to go
    :param location: latlon object for where we are
    :param body: the body on which to test those two positions
    :return:
    """
    R = body.equatorial_radius

    dLat = math.radians(target.lat - location.lat)
    dLon = math.radians(target.lon - location.lon)
    lat1 = math.radians(location.lat)
    lat2 = math.radians(target.lat)

    a = math.sin(dLat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))

    return R * c


def courseCorrection(currentHeading, targetBearing):
    """
    Given our current heading, correct it to make sure we're pointed the right way

    :param currentHeading:
    :param targetBearing:
    :return:
    """

    unadjusted = targetBearing - currentHeading
    if unadjusted < -180:
        return unadjusted + 360
    if unadjusted > 180:
        return unadjusted - 360
    return unadjusted


class RoverGo(utils.Program):
    """
    Program object to drive a rover to the specified waypoint. Uses waypoints as a convenience to
    figure out where to go.   Attempts to bring rover to a complete stop and quicksave at regular
    intervals.
    """
    def __init__(self, connection, vessel, waypoint, speed=10.0, savetime=300):
        """
        :param connection: connection to use
        :param vessel: vessel to control
        :param waypoint: waypoint to rove toward
        :param speed: how fast we can go (in m/s)
        :param savetime: how often (in seconds) we should save
        """

        super(RoverGo, self).__init__("RoverGo")
        self.connection = connection
        self.vessel = vessel
        self.waypoint = waypoint
        self.speed = speed
        self.savetime = savetime

        self.autosave = RoverAutoSave(connection, vessel, savetime, vessel.parts.all)
        self.recharge = RoverRecharge(connection, vessel)

        self.groundTelem = vessel.flight(vessel.orbit.body.reference_frame)
        self.surfTelem = vessel.flight(vessel.surface_reference_frame)
        self.target = maths.latlon(waypoint.latitude, waypoint.longitude)

        self.there_yet = False

        # Setup the PID controllers for steering and throttle. The steering
        # setpoint is locked to 0 since we'll be feeding in an error number to
        # the update function.
        self.steering = PID(.01, .01, .001)
        self.throttle = PID(.5, .01, .001)
        self.steering.setpoint(0)
        self.throttle.setpoint(speed)

        self.speed = speed

    def __call__(self):

        self.autosave()  # make sure we're saved up properly

        self.recharge()  # make sure we're charged up

        location = maths.latlon(self.groundTelem.latitude, self.groundTelem.longitude)

        targetHeading = headingForLatLon(self.target, location)
        courseCorrect = courseCorrection(self.surfTelem.heading, targetHeading)
        steerCorrect = self.steering.update(courseCorrect)
        self.vessel.control.wheel_steering = steerCorrect

        # Throttle control  -  tries to maintain the given speed!
        self.vessel.control.brakes = False
        throttleSetting = self.throttle.update(self.groundTelem.speed)
        self.vessel.control.wheel_throttle = throttleSetting

        # Check if we're close and end the program
        if distanceOverSurface(self.target, location, self.vessel.orbit.body) < 50:
            print("Done")
            return True

        return False


class RoverAutoSave(utils.Program):
    """
    A program object to handle autosaving while we're roving, and will do some error checking
    if the vessel it's operating on happens to lose some parts
    """
    def __init__(self, connection, vessel, saveTime, partsList):
        """

        :param connection: connection to use
        :param vessel: vessel to control
        :param saveTime: how often to save
        :param partsList: the list of parts on our current vessel
        """
        super(RoverAutoSave, self).__init__("RoverAutoSave")
        self.saveTime = saveTime
        self.connection = connection
        self.vessel = vessel

        self.surfTelem = vessel.flight(vessel.surface_reference_frame)
        self.groundTelem = vessel.flight(vessel.orbit.body.reference_frame)

        self.lastSave = time.time()

        self.partsList = partsList

    def __call__(self):
        # if save time is zero, just bail out
        if self.saveTime == 0:
            print("bail")
            return

        # if it's time to save
        if time.time() - self.lastSave > self.saveTime:
            print("Saving")
            # wait until it's safe to save
            if self.safeToSave():
                # Stop the rover then save
                self.vessel.control.throttle = 0.0
                self.vessel.control.brakes = True

                # we're slowing down
                while self.surfTelem.speed > 0.01:
                    time.sleep(0.1)
                time.sleep(.1)

                self.connection.space_center.quicksave()

                self.vessel.control.brakes = False

            self.lastSave = time.time()

    def safeToSave(self):
        """
        Checks if it's safe for us to save, but will bail out if the rover is rolling, or broken

        :return: if it's safe for us to try and save
        """
        if self.groundTelem.speed < .1:  # We might be stuck!
            return False
        if self.surfTelem.pitch > 25 or self.surfTelem.roll > 25:  # We might have rolled!
            return False
        if len(self.partsList) is not len(self.vessel.parts.all):  # We might have lost something?
            return False

        return True  # all good!


class RoverRecharge(utils.Program):
    """
    Subprogram for the RoverGo to use to recharge its batteries
    """
    def __init__(self, connection, vessel):
        """
        :param connection: connection we're using
        :param vessel: vessel we're controlling
        """
        super(RoverRecharge, self).__init__("RoverRecharge")
        self.connection = connection
        self.vessel = vessel
        self.telemetry = vessel.flight(vessel.orbit.body.reference_frame)
        self.maxEC = vessel.resources.max('ElectricCharge')

    def __call__(self):
        EC = self.vessel.resources.amount('ElectricCharge')

        # if we're at less than 5% of our max charge, stop the rover and wait until we're charged up
        if EC / self.maxEC < .05:
            print("Charging")
            self.vessel.control.wheel_throttle = 0
            self.vessel.control.brakes = True

            print("slowing down")
            while self.telemetry.speed > 0.01:
                time.sleep(0.1)

            self.vessel.control.solar_panels = True

            print("charging")
            while EC / self.maxEC < .85:  # less than 85% charge
                EC = self.vessel.resources.amount('ElectricCharge')
                time.sleep(0.1)

            # for safety, retract the solar panels so they don't break
            self.vessel.control.solar_panels = False
            self.vessel.control.brakes = False
