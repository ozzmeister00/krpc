import krpc
import math

connection = krpc.connect("Launcher")
vessel = connection.space_center.active_vessel

def total_area(orbit):
    a = orbit.semi_major_axis
    b = orbit.semi_minor_axis
    return math.pi * a * b

def area_since_periapsis(orbit, E):
    a = orbit.semi_major_axis
    b = orbit.semi_minor_axis
    return 0.5 * a * b * E

def mean_anomaly_from_true_anomaly(orbit, theta):
    e = orbit.eccentricity
    adj = e + math.cos(theta)
    hyp = 1 + e*math.cos(theta)
    return math.acos(adj / hyp)

def area_between_mean_anomalies(orbit, E0, E1):
    A0 = area_since_periapsis(orbit, E0)
    A1 = area_since_periapsis(orbit, E1)
    return A1 - A0

def time_to_ascending_node(orbit):
    A0 = total_area(orbit)

    E = orbit.mean_anomaly
    Ap = area_since_periapsis(orbit, E)

    theta = orbit.argument_of_periapsis - math.pi
    En = mean_anomaly_from_true_anomaly(orbit, theta)
    En += math.pi
    An = area_since_periapsis(orbit, En)

    frac = (An-Ap)/A0
    return orbit.period * frac

def change_inclination(conn, vessel, new_inclination):
    orbit = vessel.orbit
    i = new_inclination - orbit.inclination
    v = orbit.speed
    normal = v*math.sin(i)
    prograde = v*math.cos(i) - v
    ut = conn.space_center.ut + time_to_ascending_node(orbit)
    node = vessel.control.add_node(ut, normal=normal, prograde=prograde)
    return node

def Main():
    change_inclination(connection, vessel, 0.0)

if __name__ == '__main__':
    Main()