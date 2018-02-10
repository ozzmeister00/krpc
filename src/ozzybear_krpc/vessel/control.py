"""
control.py

Functions for controlling a vehicle, at this time probably mostly rockets.
This will probably get broken up in the future, so try to group things in what
ever way seems sensible to you, man.

TODO:
    *   determine whether a Resources object updates, or if it is static upon
        creation. This might impact how I handle things like
        get_stage_resources (if it's static, probably return a dict, but if it
        is dynamic, might want to return the object so that it can be used for
        updates).

"""

import ozzybear_krpc.telemetry
from ozzybear_krpc import const

import time

INTERRUPT_STOP = 'stop'
INTERRUPT_REMOVE = 'remove'
INTERRUPT_CONTINUE = 'continue'


def get_current_stage(vessel):

    return vessel.control.current_stage


def get_stage_resources_native(vessel, stage=None, cumulative=False):
    if stage is None:
        stage = get_current_stage(vessel)
        # TODO go to logging
        print("current stage: {0}".format(stage))

    resources = vessel.resources_in_decouple_stage(stage=stage, cumulative=cumulative)

    return resources


def get_stage_resource_amounts(
    vessel, stage=None,
    cumulative=False, res_filter=frozenset()):

    resources = get_stage_resources_native(vessel, stage=stage, cumulative=cumulative)
    # TODO cleanup on aisle too dense
    resource_dict = dict((name, resources.amount(name)) for name in resources.names if name not in res_filter)

    return resource_dict


def prep_rocket_launch(vessel):
    vessel.auto_pilot.target_pitch_and_heading(90, 90)
    vessel.auto_pilot.engage()
    vessel.control.throttle = 1


def launch_rocket(conn, vessel, heading, turn_altitude=10000):
    vessel.control.activate_next_stage()


def _stage_ready(resources_obj, autostage_resources, mode=const.AND):
    if mode not in [const.AND, const.OR]:
        raise ValueError('mode must be one of [{0}], {1} was provided'.format(', '.join([const.AND, const.OR]), mode))
    maxes = dict((res, resources_obj.max(res))for res in autostage_resources)

    for resource in autostage_resources:
        current = resources_obj.amount(resource)
        if current <= 0 or current / maxes[resource] <= 0.05:
            # TODO logging
            print('out of {0}'.format(resource))
            if mode == const.OR:
                return True
        elif mode == const.AND:
            print("in mode AND and found still have resource {0}".format(resource))
            return False

    if mode == const.AND:
        print("returning True since reached end with AND")
        return True
    else:
        print("implicit OR mode we didn't hit any empties")
        return False


def _get_autostage_stages(vessel, autostage_resources=const.RESOURCES_FUEL, stop_if_skipping=True):
    # I /think/ this needs the +1, as the dox say it corresponds to the
    # in-game UI, but I haven't checked yet
    count_stages = get_current_stage(vessel)

    print("count stages: {0}".format(count_stages))

    stages = []

    for stage_i in reversed(range(count_stages)):
        stage_resources = vessel.resources_in_decouple_stage(stage=stage_i, cumulative=False)
        print(stage_i, set(stage_resources.names), autostage_resources)
        if set(stage_resources.names).intersection(autostage_resources):
            stages.append(stage_i)
        elif stop_if_skipping:
            stages.append(None)
    # TODO logging
    print('stages: {0}'.format(', '.join([str(i) for i in stages])))
    return stages



def get_gravity_turn_interrupt(conn, def_altitude=10000):
    def gravity_turn_interrupt(vessel, altitude=def_altitude):
        # TODO
        pass


def autostage(
        vessel, stages=None,
        stop_if_skipping=True,
        interrupts=None, noisy=True,
        autostage_resources=const.RESOURCES_FUEL):

    if stages is None:
        stages = _get_autostage_stages(vessel, autostage_resources=autostage_resources, stop_if_skipping=stop_if_skipping)

    interrupts = set(interrupts or [])

    for stage in stages:
        if stage is None:
            raise NonEngineStage()
        while True:
            resources = vessel.resources_in_decouple_stage(stage)
            if _stage_ready(resources, autostage_resources  , mode=const.AND):
                print("firing stage {0}".format(stage))
                time.sleep(0.5)
                vessel.control.activate_next_stage()
                print("stage {0} fired.".format(stage))
                break
            if noisy:
                for resource in autostage_resources:
                    pass
                    #if resources.amount(resource):
                        #print("{0} level: {1}".format(resource, resources.amount(resource)))
            if interrupts is not None:
                for interrupt in set(interrupts):
                    try:
                        interrupt(vessel)
                    except Interrupt as exc:
                        if exc.remove:
                            interrupts.remove(interrupt)
                        if exc.stop:
                            raise exc
                        if exc.skip_stage:
                            break
            time.sleep(0.1)

    print('no more stages!')


class NonEngineStage(Exception):
    pass


class Interrupt(Exception):
    def __init__(self, msg=None, remove=False, skip_stage=False, stop=False):
        self.remove = remove
        self.skip_stage = skip_stage
        self.stop = stop

        if msg is None:
            msg = 'Interrupt'

        super(Interrupt, self).__init__(msg)




