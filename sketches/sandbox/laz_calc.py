import krpc

import math


class LaunchAzimuthCalculator(object):
    def __init__(self, targetAltitude, targetInclination, connection=None, vessel=None):
        """
        Calculate the launch azimuth (heading from launch site) to achieve an orbit
        at the given altitude and inclination

        :param targetAltitude:
        :param targetInclination:
        """
        if not connection:
            self.connection = krpc.connect("CalculateLaunchAzimuth")
        if not vessel:
            self.vessel = connection.space_center.active_vessel

        # //Orbital altitude can't be less than sea level
        if targetAltitude <= 0:
            raise ValueError("Orbital altitude can't be below sea level")

        launchLatitude = self.vessel.flight().latitude

        # //Determines whether we're trying to launch from the ascending or descending node
        self.ascending = True
        if targetInclination < 0:
            self.ascending = False
            # //We'll make it positive for now and convert to southerly heading later
            targetInclination = abs(targetInclination)

        # TODO clean this up
        # //Orbital inclination can't be less than launch latitude or greater than 180 - launch latitude
        if abs(launchLatitude) > targetInclination:
            targetInclination = abs(launchLatitude)
            print("Inclination impossible from current latitude, setting for lowest possible inclination.")

        if 180 - abs(launchLatitude) < targetInclination:
            targetInclination = 180 - abs(launchLatitude)
            print("Inclination impossible from current latitude, setting for highest possible inclination.")

        # //Does all the one time calculations and stores them in a list to help reduce the overhead or continuously updating
        self.equatorialVel = (2 * math.pi * vessel.orbit.body.equatorial_radius) / vessel.orbit.body.rotation_period
        self.targetOrbVel = math.sqrt(vessel.orbit.body.mu / (vessel.orbit.body.equatorial_radius + targetAltitude))
        self.targetInclination = targetInclination
        self.targetAltitude = targetAltitude
        self.launchLatitude = launchLatitude

    # data:ADD(desiredInc).       //[0]
    # data:ADD(launchLatitude).   //[1]
    # data:ADD(equatorialVel).    //[2]
    # data:ADD(targetOrbVel).     //[3]
    # data:ADD(launchNode).       //[4]
    def __call__(self):
        inertialAzimuth = math.asin(max(min(math.cos(self.targetInclination) / math.cos(self.vessel.flight().latitude), 1), -1))
        VXRot = self.targetOrbVel * math.sin(inertialAzimuth) - self.equatorialVel * math.cos(self.launchLatitude)
        VYRot = self.targetOrbVel * math.cos(inertialAzimuth)

        # // This clamps the result to values between 0 and 360.
        Azimuth = (math.atan2(VXRot, VYRot) + 360) % 360

        # //Returns northerly azimuth if launching from the ascending node
        if self.ascending:
            return Azimuth

        # //Returns southerly azimuth if launching from the descending node
        else:
            if Azimuth <= 90:
                return 180 - Azimuth

            elif Azimuth >= 270:
                return 540 - Azimuth
