import krpc
import time

connection = krpc.connect("Staging Test")
vessel = connection.space_center.active_vessel

# get decouple stages with liquid fuel or solid fuel
launchStage = vessel.control.current_stage
decoupleStages = []
for stage in range(launchStage):
    resources = vessel.resources_in_decouple_stage(stage=stage, cumulative=False)
    if resources.names:
        if resources.has_resource("SolidFuel") or resources.has_resource("LiquidFuel"):
            decoupleStages.append(stage)

decoupleStages = list(reversed(decoupleStages))
print(decoupleStages)

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

    print(currentStage, currentDecoupleStage)

    stageResources = vessel.resources_in_decouple_stage(stage=currentDecoupleStage, cumulative=False)

    # if we have solid fuel and we're out of it, we need staging
    if stageResources.has_resource("SolidFuel"):
        print("Stage has solid fuel")
        maxSolid =  stageResources.max("SolidFuel")
        currentSolid = stageResources.amount("SolidFuel")
        if currentSolid / maxSolid <= 0.05:
            print("Triggering staging because we had but are out of solid fuel")
            return True

    elif stageResources.has_resource("LiquidFuel"):
        print("Stage has liquid fuel")
        maxLiquid =  stageResources.max("LiquidFuel")
        currentLiquid = stageResources.amount("LiquidFuel")
        if currentLiquid <= 0.0 or maxLiquid / currentLiquid <= 0.05:
            print("Triggering staging because we're out of liquid fuel")
            return True

    else:
        return False

    return False

def main():
    vessel.control.activate_next_stage()

    while True:
        if needsStaging():
            print("Need staging")
            time.sleep(0.5)
            vessel.control.activate_next_stage()
            print("Staged")

        time.sleep(0.1)

if __name__ == '__main__':
    main()