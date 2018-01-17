import krpc
import math
import time

connection = krpc.connect("Launcher")
vessel = connection.space_center.active_vessel

# TODO: json files for launch profiles
TURN_START_ALTITUDE = 500
TURN_END_ALTITUDE = 45000
TARGET_APOAPSIS = 100000 # TODO would be helpful to modify the turn end altitude based on how high we're trying to go
TARGET_INCLINATION = 0 # TODO launch into correct inclination and argument of periapsis
OUT_OF_ATMOSPHERE = 75000
DISCARD_LAUNCH_STAGE = True # TODO this might not always work and should be streamlined
# TODO further, the launch controller should start pushing the nose inward to better utilize fuel
# TODO thrust should be controlled to maximize maxq potential

burnTime = 0

# set up telemetry streams
ut = connection.add_stream(getattr, connection.space_center, 'ut')
altitude = connection.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = connection.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

turn_angle = 0

cirularizeNode = None

def calculcateCircularizationBurn():
    global cirularizeNode

    print('Calculating ciruclarization burn')
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    deltaV = v2 - v1

    print('mu ', mu)
    print('r ', r)
    print('a1 ', a1)
    print('a2 ', a2)
    print('v1 ', v1)
    print('v2 ', v2)
    print('deltaV ', deltaV)
    print('timeToApoapsis ', vessel.orbit.time_to_apoapsis)

    cirularizeNode = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=deltaV)

    F = vessel.available_thrust
    Isp = vessel.specific_impulse * vessel.orbit.body.gravitational_parameter
    m0 = vessel.mass

    print('F ', F)
    print('specific impulse ', vessel.specific_impulse)
    print('Isp ', Isp)
    print('m0 ', m0)

    m1 = m0 / math.exp(deltaV/Isp) # math range error
    flowRate = F / Isp
    burnTime = (m0 - m1) / flowRate

    return burnTime


def preflight():

    # warn the user if they're trying to run a launch script in an post-launched state
    if vessel.situation != vessel.situation.pre_launch:
        raise Exception("This vessel is not ready for launch")

    # make sure the vessel is ready to roll
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1.0
    vessel.control.gear = False
    vessel.control.legs = False
    vessel.control.wheels = False

    vessel.auto_pilot.engage()

    # countdowns are fun
    for i in list(reversed(range(4))):
        print('T-{}'.format(i))
        time.sleep(1)

    vessel.control.activate_next_stage()

    return True

def liftoff():

    # keep pointed up
    vessel.auto_pilot.target_pitch_and_heading(90, 90)

    # if we're beyond our turn start altitude, start the turn
    if altitude() > TURN_START_ALTITUDE:
        return True

    return False

def gravityTurn():
    global turn_angle

    if altitude() < TURN_END_ALTITUDE:

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
    global DISCARD_LAUNCH_STAGE

    vessel.auto_pilot.target_pitch_and_heading(0, 90)

    if apoapsis() > TARGET_APOAPSIS:
        if altitude() < OUT_OF_ATMOSPHERE:
            if apoapsis() >= TARGET_APOAPSIS:
                vessel.control.throttle = 0.0
            else:
                vessel.control.throttle = 0.0625

            return False

        if DISCARD_LAUNCH_STAGE:
            vessel.control.throttle = 0.0

            time.sleep(0.5)

            vessel.control.activate_next_stage()

            DISCARD_LAUNCH_STAGE = False

        burnTime = calculcateCircularizationBurn()

        return True

    vessel.control.throttle = 0.250
    return False

def coastToApoapsis():
    global cirularizeNode

    vessel.control.throttle = 0.0
    if altitude() > OUT_OF_ATMOSPHERE:
        print("Orienting ship to circularization node")
        vessel.auto_pilot.reference_frame = cirularizeNode.reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)
        vessel.auto_pilot.wait()

    print('Waiting until circularization burn')
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burnTime/2.)
    lead_time = 5
    connection.space_center.warp_to(burn_ut - lead_time)

    return True

def circularizeMine():
    global cirularizeNode
    global burnTime

    initialDirection = cirularizeNode.burn_vector(reference_frame=vessel.orbital_reference_frame)
    targetApoapsis = apoapsis()

    vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame

    # hold in the initial direction
    vessel.auto_pilot.target_direction = initialDirection

    # Execute burn
    print('Ready to execute burn')
    time_to_apoapsis = connection.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burnTime/2.) > 0:
        pass

    print('Executing burn')
    vessel.control.throttle = 1.0
    while vessel.orbit.periapsis_altitude < targetApoapsis * 0.9:
        pass

    print('Fine tuning')
    vessel.control.throttle = 0.05
    while vessel.orbit.periapsis_altitude < targetApoapsis:
        pass

    vessel.control.throttle = 0.0
    cirularizeNode.remove()

    return True

# TODO make this work better in the runmode loop so we can handle aborts
def circularize():
    global cirularizeNode
    global burnTime

    remaining_burn = connection.add_stream(cirularizeNode.remaining_burn_vector, cirularizeNode.reference_frame)
    fullBurn = remaining_burn()[1]

    # Execute burn
    print('Ready to execute burn')
    time_to_apoapsis = connection.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burnTime/2.) > 0:
        pass

    print('Executing burn')
    vessel.control.throttle = 1.0
    while (remaining_burn()[1] / fullBurn) > 0.025:
        pass

    holdDirection = cirularizeNode.burn_vector(reference_frame=vessel.orbital_reference_frame)
    vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame

    # hold our direction to fine-tune it
    vessel.auto_pilot.target_direction = holdDirection

    # fine tune to a very small spot
    print("Fine tuning")
    vessel.control.throttle = 0.05
    while remaining_burn()[1] > 2.0:
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

# get decouple stages with liquid fuel or solid fuel
launchStage = vessel.control.current_stage
decoupleStages = []
for stage in range(launchStage):
    resources = vessel.resources_in_decouple_stage(stage=stage, cumulative=False)
    if resources.names:
        if resources.has_resource("SolidFuel") or resources.has_resource("LiquidFuel"):
            decoupleStages.append(stage)

decoupleStages = list(reversed(decoupleStages))

def getDecoupleStage(curr):
    for i in decoupleStages:
        if curr > i:
            return i

    return curr

def needsStaging():
    # TODO: get resources per stage?
    #stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
    currentStage = vessel.control.current_stage
    currentDecoupleStage = getDecoupleStage(currentStage)

    stageResources = vessel.resources_in_decouple_stage(stage=currentDecoupleStage, cumulative=False)

    # if we have solid fuel and we're out of it, we need staging
    if stageResources.has_resource("SolidFuel"):
        maxSolid =  stageResources.max("SolidFuel")
        currentSolid = stageResources.amount("SolidFuel")
        if currentSolid <= 0.0:
            print("Triggering staging because we had but are out of solid fuel")
            return True

    elif stageResources.has_resource("LiquidFuel"):
        maxLiquid =  stageResources.max("LiquidFuel")
        currentLiquid = stageResources.amount("LiquidFuel")
        if currentLiquid <= 0.0:
            print("Triggering staging because we're out of liquid fuel")
            return True

    else:
        return False

    return False

def userHasAborted():
    return vessel.control.abort

def programHasFinished(runmode):
    return runmode == sorted(runModes.keys())[-1] + 1

def main():
    runmode = 0

    while runmode >= 0:
        # if the current runmode has met its criteria, move on
        if runModes[runmode]():
            print("Advancing runmode to ", runmode+1)
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

        time.sleep(.5)

    vessel.auto_pilot.disengage()


if __name__ == '__main__':
    main()