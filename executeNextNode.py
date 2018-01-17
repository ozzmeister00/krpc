import krpc
import math
import time

connection = krpc.connect("Launcher")
vessel = connection.space_center.active_vessel



# set up telemetry streams
ut = connection.add_stream(getattr, connection.space_center, 'ut')
altitude = connection.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = connection.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
# find the next node
nextNode = vessel.control.nodes[0]

remaining_burn = connection.add_stream(nextNode.remaining_burn_vector, nextNode.reference_frame)

fullBurn = remaining_burn()[1]

F = vessel.available_thrust
Isp = vessel.specific_impulse * vessel.orbit.body.gravitational_parameter
m0 = vessel.mass
m1 = m0 / math.exp(nextNode.delta_v/Isp) # math range error
flowRate = F / Isp
burnTime = (m0 - m1) / flowRate

def preflight():

    # todo make sure we're running the right program for our situation

    # make sure the vessel is ready to roll
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 0.0
    vessel.control.gear = False
    vessel.control.legs = False
    vessel.control.wheels = False

    vessel.auto_pilot.engage()

    return True

def orientToManeuver():
    global nextNode

    vessel.control.throttle = 0.0
    vessel.auto_pilot.reference_frame = nextNode.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    vessel.auto_pilot.wait()

    print('Waiting until circularization burn')
    burn_ut = ut() + nextNode.time_to - (burnTime/2.) # TODO
    lead_time = 5
    connection.space_center.warp_to(burn_ut - lead_time)

    return True

def coastToManeuver():
    # Execute burn
    print('Ready to execute burn')
    while nextNode.time_to - (burnTime/2.) > 0:
        pass

    return True

def burn():
    global nextNode

    vessel.control.throttle = 1.0
    if (remaining_burn()[1] / fullBurn) > 0.1:
        return False

    return True

def fineTune():
    holdDirection = nextNode.burn_vector(reference_frame=vessel.orbital_reference_frame)
    vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
    # hold our direction to fine-tune it
    vessel.auto_pilot.target_direction = holdDirection

    # fine tune to a very small spot
    print("Fine tuning")
    vessel.control.throttle = 0.05
    if remaining_burn()[1] > 2.0:
        return False

    return True


def cleanup():
    vessel.control.throttle = 0.0
    nextNode.remove()
    return True

runModes = {0:preflight,
            1:orientToManeuver,
            2:coastToManeuver,
            3:burn,
            4:fineTune,
            5:cleanup
            }

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
            runmode = -1

        time.sleep(.1)

    vessel.auto_pilot.disengage()


if __name__ == '__main__':
    main()