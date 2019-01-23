"""
Stores all the functions we need to land a craft
"""

import math


def getLandingReferenceFrame(connection, body, vessel, landing_longitude, landing_latitude):
    """
    Constructs a reference frame object based on the vessel, body, and landing lat/long we're aiming for
    :param connection:
    :param body:
    :param vessel:
    :param landing_longitude:
    :param landing_latitude:
    :return:
    """
    ReferenceFrame = connection.space_center.ReferenceFrame

    # Define landing site
    landing_latitude = -(0+(5.0/60)+(48.38/60/60)) # Top of the VAB
    landing_longitude = -(74+(37.0/60)+(12.2/60/60))
    landing_altitude = 2000

    # Determine landing site reference frame (orientation: x=zenith, y=north, z=east)
    landing_position = body.surface_position(landing_latitude, landing_longitude, body.reference_frame)
    q_long = (0, math.sin(-landing_longitude * 0.5 * math.pi / 180), 0, math.cos(-landing_longitude * 0.5 * math.pi / 180))
    q_lat = (0, 0, math.sin(landing_latitude * 0.5 * math.pi / 180), math.cos(landing_latitude * 0.5 * math.pi / 180))
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


def unit_vector(vector):
    """ Returns the unit vector of the vector provided.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def coords_down_bearing(lat, lon, bearing, distance, body):
    '''
    Takes a latitude, longitude and bearing in degrees, and a
    distance in meters over a given body.  Returns a tuple
    (latitude, longitude) of the point you've calculated.
    '''

    bearing = math.radians(bearing)
    R = body.equatorial_radius
    lat = math.radians(lat)
    lon = math.radians(lon)

    a = math.sin(lat)
    b = math.cos(distance / R)

    lat2 = math.asin(
        math.sin(lat) * math.cos(distance / R) + math.cos(lat) * math.sin(distance / R) * math.cos(bearing))

    lon2 = lon + math.atan2(math.sin(bearing) * math.sin(distance / R) * math.cos(lat),
                            math.cos(distance / R) - math.sin(lat) * math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    return (lat2, lon2)


def check_terrain(lat1, lon1, lat2, lon2, body):
    '''
    Returns an estimate of the highest terrain altitude betwen
            two latitude / longitude points.
    '''
    lat = lat1
    lon = lon1
    highest_lat = lat
    highest_lon = lon
    highest_alt = body.surface_height(lat, lon)
    latstep = (lat2 - lat1) / 20
    lonstep = (lon2 - lon1) / 20

    for x in range(20):
        test_alt = body.surface_height(lat, lon)
        if test_alt > highest_alt:
            highest_lat = lat
            highest_lon = lon
            highest_alt = test_alt
        lat = lat + latstep
        lon = lon + lonstep
    return highest_alt


class SuicideBurnCalculator():
    def __init__(self, conn, v, alt):
        self.conn = conn
        self.v = v
        self.sc = conn.space_center
        self.burn_time = np.inf
        self.burn_duration = np.inf
        self.ground_track = np.inf
        self.effective_decel = np.inf
        self.radius = np.inf
        self.time_to_impact = np.inf
        self.time_to_burn = np.inf
        self.desired_throttle = 0.95

        self.alt = alt

        self.rf = self.sc.ReferenceFrame.create_hybrid(
            position=v.orbit.body.reference_frame,
            rotation=v.surface_reference_frame
        )

    def __call__(self):
        rf = self.v.orbit.body.reference_frame
        v1 = self.v.velocity(self.rf)
        v2 = (0, 0, 1)
        self.angle_from_horizontal = angle_between(v1, v2)
        sine = math.sin(self.angle_from_horizontal)

        g = self.v.orbit.body.surface_gravity
        T = (self.v.max_thrust / self.v.mass)
        self.effective_decel = .5 * (-2 * g * sine + math.sqrt(
            (2 * g * sine) * (2 * g * sine) + 4 * (T * T - g * g)))  # is it because of this line?
        self.decel_time = self.v.flight(
            self.rf).speed / self.effective_decel  # TODO: I'm not sure this is getting calculated right, as we end up cutting off so high
        # TODO: and any time we spend below about 95% throttle we might be using too much fuel

        # no idea how this math works
        radius = self.v.orbit.body.equatorial_radius + self.alt
        TA = self.v.orbit.true_anomaly_at_radius(radius)
        TA = -1 * TA
        self.time_to_impact = self.v.orbit.ut_at_true_anomaly(TA) - self.sc.ut
        self.time_to_burn = self.time_to_impact - self.decel_time

        impact_time = self.v.orbit.ut_at_true_anomaly(TA)
        burn_time = impact_time - self.decel_time / 2
        self.ground_track = ((burn_time - self.sc.ut) * self.v.flight(self.rf).speed) + (
                    .5 * self.v.flight(self.rf).speed * self.decel_time)

        self.desired_throttle = self.decel_time / self.time_to_impact


class Descend(Program):
    def __init__(self, vessel, connection):
        super(Descend, self).__init__("Descend")

        self.vessel = vessel
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)

        self.sbc = SuicideBurnCalculator(connection, vessel, 5000)
        self.sbc()

        # find the highest point over our current ground path and use that to calculate our suicide burn, for safety
        touchdown = coords_down_bearing(self.flight.latitude, self.flight.longitude, (180 + self.flight.heading),
                                        self.sbc.ground_track, self.vessel.orbit.body)
        self.sbc.alt = check_terrain(self.flight.latitude, self.flight.longitude, touchdown[0], touchdown[1],
                                     self.vessel.orbit.body)
        self.sbc()

        self.burning = False

        self.vessel.auto_pilot.reference_frame = self.vessel.surface_velocity_reference_frame
        self.vessel.auto_pilot.target_direction = (0, -1, 0)
        self.vessel.auto_pilot.target_roll = float("nan")
        self.vessel.auto_pilot.engage()

    def __call__(self):

        # update the max safe altitude as our burn progresses
        touchdown = coords_down_bearing(self.flight.latitude, self.flight.longitude, (180 + self.flight.heading),
                                        self.sbc.ground_track, self.vessel.orbit.body)
        self.sbc.alt = check_terrain(self.flight.latitude, self.flight.longitude, touchdown[0], touchdown[1],
                                     self.vessel.orbit.body)

        self.sbc()

        # one-time call to check if it's time to burn
        if self.sbc.time_to_burn <= 0.0:
            self.burning = True

        # throttle should be proportional to how long we have to burn verses how long until we hit the ground
        if self.burning:
            self.vessel.control.throttle = self.sbc.desired_throttle

        # once we're slow enough, move to the next mode
        if self.flight.speed < 10.0:
            return False

        return True

    def displayValues(self):
        return [self.prettyName,
                'mxDcl: ' + str(round(self.sbc.effective_decel, 2)),
                'brnDr: ' + str(round(self.sbc.decel_time, 2)),
                'ttimp: ' + str(round(self.sbc.time_to_impact, 2)),
                'alt  : ' + str(round(self.sbc.alt, 2)),
                'grTrk: ' + str(round(self.sbc.ground_track, 2)),
                'dsrTh: ' + str(round(self.sbc.desired_throttle, 2))
                ]


class SoftTouchdown(Program):
    def __init__(self, vessel, connection):
        super(SoftTouchdown, self).__init__("SoftLanding")

        self.vessel = vessel
        self.auto_pilot = vessel.auto_pilot
        self.control = vessel.control
        self.flight = vessel.flight(vessel.orbit.body.reference_frame)

        self.auto_pilot.reference_frame = self.vessel.surface_velocity_reference_frame
        self.auto_pilot.target_direction = (0, -1, 0)
        self.auto_pilot.target_roll = float("nan")
        self.auto_pilot.engage()

    def __call__(self):
        # our maximum descent speed should be 10% of the distance to the ground
        safe_descent = self.flight.surface_altitude / -10
        safe_descent = max(safe_descent, -15)

        # the midpoint of our safe descent should be a hovering throttle
        a = self.vessel.orbit.body.surface_gravity - self.flight.vertical_speed
        F = self.vessel.mass * a

        midPoint = (F / self.vessel.available_thrust)

        # rudimentary proportional controller
        e = safe_descent - self.flight.vertical_speed
        p0 = midPoint
        ekp = .25 * e
        self.control.throttle = ekp + p0

        # if we're on the ground, or if we're moving really slowly, otherwise we go in a wonky direction really quickly
        if self.vessel.situation == self.vessel.situation.landed or abs(self.flight.vertical_speed) < 0.1:
            return False

        return True

    def displayValues(self):
        return [self.prettyName,
                'thrtl: ' + str(round(self.control.throttle, 2)),
                'alt  : ' + str(round(self.flight.surface_altitude)),
                'vrtsp: ' + str(round(self.flight.vertical_speed))
                ]