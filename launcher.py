import krpc
import math
import time

connection = krpc.connection("Launcher")
vessel = connection.space_center.active_vessel

# TODO: json files for launch profiles
TURN_START_ALTITUDE = 250
TURN_END_ALTITUDE = 45000
TARGET_APOAPSIS = 100000
TARGET_INCLINATION = 0
OUT_OF_ATMOSPHERE = 75000


burnTime = 0

# set up telemetry streams
ut = connection.add_stream(getattr, connection.space_center, 'ut')
altitude = connection.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = connection.add_stream(getattr, vessel.flight(), 'apoapsis_altitude')
# TODO: get resources per stage?
stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)

turn_angle = 0

cirularizeNode = None

def calculcateCircularizationBurn():
    global cirularizeNode

    print('Calculating ciruclarization burn')
    mu = vessel.orbit.body.gravitational_paremeter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1.-a1)))
    v2 = math.sqrt(mu*((2./r)-(1.-a2)))
    deltaV = v2 - v1

    cirularizeNode = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=deltaV)

    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(deltaV/Isp)
    flowRate = F / Isp
    burnTime = (m0 - m1) / flowRate

    return burnTime


def preflight():

    # warn the user if they're trying to run a launch script in an post-launched state
    if vessel.situation != krpc.Vessel.VesselSituation.pre_launch:
        raise Exception("This vessel is not ready for launch")

    # make sure the vessel is ready to roll
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1.0
    vessel.control.gear = False
    vessel.control.legs = False
    vessel.control.wheels = False

    # countdowns are fun
    for i in range(3):
        print('T-{}'.format(i))
        time.sleep(1)

    return False

def liftoff():

    # keep pointed up
    vessel.auto_pilot.target_pitch_and_heading(90, 90)

    # if we're beyond our turn start altitude, start the turn
    if altitude() > TURN_START_ALTITUDE:
        return True

    return False

def gravityTurn():

    if altitude() > TURN_START_ALTITUDE and altitude() < TURN_END_ALTITUDE:

        # TODO: this looks suspiciously like a lerp
        frac = ((altitude() - TURN_START_ALTITUDE) / (TURN_END_ALTITUDE - TURN_START_ALTITUDE))

        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

    if altitude() > TURN_END_ALTITUDE:
        return True

    return False

def tuneApoapsis():
    global burnTime

    vessel.auto_pilot.target_pitch_and_heading(0, 90)

    if apoapsis() > TARGET_APOAPSIS:
        if altitude() < OUT_OF_ATMOSPHERE:
            vessel.control.throttle = 0.0625
            return False

        burnTime = calculcateCircularizationBurn()

        return True

    vessel.control.throttle = 0.250
    return False

def coastToApoapsis():
    global cirularizeNode

    vessel.control.throttle = 0.0
    if altitude() < OUT_OF_ATMOSPHERE:
        print('Orientating ship for circularization burn')
        vessel.auto_pilot.reference_frame = cirularizeNode.reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)
        vessel.auto_pilot.wait()

    return False

def circularize():
    global cirularizeNode
    global burnTime

    print('Waiting until circularization burn')
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burnTime/2.)
    lead_time = 5
    connection.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    print('Ready to execute burn')
    time_to_apoapsis = connection.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burnTime/2.) > 0:
        pass
    print('Executing burn')
    vessel.control.throttle = 1.0
    time.sleep(burnTime - 0.1)
    print('Fine tuning')
    vessel.control.throttle = 0.05
    remaining_burn = connection.add_stream(cirularizeNode.remaining_burn_vector, cirularizeNode.reference_frame)
    while remaining_burn()[1] > 0:
        pass
    vessel.control.throttle = 0.0
    cirularizeNode.remove()

    return True

def deployment():
    vessel.control.antennas = True
    vessel.control.solar_panels = True
    vessel.control.throttle = 0.0

    return True

runModes = {0:preflight,
            1:liftoff,
            2:gravityTurn,
            3:tuneApoapsis,
            4:coastToApoapsis,
            5:circularize}

def needsStaging():
    # TODO figure out if we need to stage or not, dynamically
    return False

def userHasAborted():
    return vessel.control.abort

def programHasFinished(runmode):
    return runmode == sorted(runModes.keys())[-1]

def main():
    runmode = 0

    while runmode >= 0:
        # if the current runmode has met its criteria, move on
        if runModes[runmode]():
            runmode += 1

        # see if we're out of fuel in our current stage, and stage
        if needsStaging():
            vessel.control.activate_next_stage()

        # bail out!
        if userHasAborted():
            runmode = -1

        # aaaaand we're done
        if programHasFinished(runmode):
            deployment()
            runmode = -1


if __name__ == '__main__':
    main()