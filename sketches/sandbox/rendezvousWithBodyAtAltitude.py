import kspy.maneuvers
import math
import krpc
from kspy.utils import defaultConnection

def clamp_2pi(x):
    '''
    clamp radians to a single revolution
    '''
    return x % (math.pi * 2)

def get_phase_angle(vessel, target):
    '''
    returns the relative phase angle for a hohmann transfer
    '''
    vo = vessel.orbit
    to = target.orbit
    h = (vo.semi_major_axis + to.semi_major_axis) / 2  # SMA of transfer orbit
    # calculate the percentage of the target orbit that goes by during the half period of transfer orbit
    p = 1 / (2 * math.sqrt(math.pow(to.semi_major_axis, 3) / math.pow(h, 3)))
    # convert that to an angle in radians
    a = (2 * math.pi) - ((2 * math.pi) * p)
    print("Transfer Phase Angle is {}.".format(a))
    return a

def orbital_progress(vessel, ut):
    '''
    returns the orbital progress in radians, referenced to the planet's origin
    of longitude.
    '''
    lan = vessel.orbit.longitude_of_ascending_node
    arg_p = vessel.orbit.argument_of_periapsis
    ma_ut = vessel.orbit.mean_anomaly_at_ut(ut)
    return clamp_2pi(lan + arg_p + ma_ut)

def time_transfer(vessel, target, ut, phase_angle):
    """
    Performs an iterative search for the next time vessel and target
    have the given relative phase_angle after ut
    """
    while True:
        v_pos = orbital_progress(vessel, ut)
        t_pos = orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .01:
            break
        ut += 10
    ut -= 10

    # fine unbound search
    while True:
        v_pos = orbital_progress(vessel, ut)
        t_pos = orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .001:
            break
        ut += 1

    return ut


def hohmannTransfer(connection=None, vessel=None, target=None, orbitAlt=0):
    '''
    Create a maneuver node for a hohmann transfer from vessel's orbit to target
    orbit at the appropriate time of closest approach

    :param connection:
    :param vessel:
    :param target:

    :return: the node created for the hohmann transfer
    '''
    if not connection:
        connection = defaultConnection("hohmannTransfer")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not target:
        # grab whatever target is currently set
        target = connection.space_center.target_vessel or connection.space_center.target_body

    phase_angle = get_phase_angle(vessel, target)

    print("phase angle is {}".format(phase_angle))
    transfer_time = time_transfer(vessel, target, connection.space_center.ut, phase_angle)

    return kspy.maneuvers.changeApoapsis(target.orbit.radius_at(transfer_time) - vessel.orbit.body.equatorial_radius, connection, vessel, atTime=transfer_time)

def main():
    node = hohmannTransfer(orbitAlt = 0)

if __name__ == '__main__':
    main()