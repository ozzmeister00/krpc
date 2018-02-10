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

import time

INTERRUPT_STOP = 'stop'
INTERRUPT_REMOVE = 'remove'
INTERRUPT_CONTINUE = 'continue'


def get_current_stage(vessel):
    return vessel.control.current_stage + 1


def get_stage_resources_native(vessel, stage=None, cumulative=False):
    if stage is None:
        stage = get_current_stage(vessel)

    resources = vessel.resources_in_decouple_stage(stage=stage, cumulative=cumulative)

    return resources


def get_stage_resource_amounts(
    vessel, stage=None,
    cumulative=False, res_filter=frozenset()):

    resources = get_stage_resources_native(vessel, stage=stage, cumulative=cumulative)
    # TODO cleanup on aisle too dense
    resource_dict = dict((name, resources.amount(name) for name in resources.names if name not in res_filter))

    return resource_dict


def prep_rocket_launch(vessel):
    vessel.auto_pilot.target_pitch_and_heading(90, 90)
    vessel.auto_pilot.engage()
    vessel.control.throttle = 1


def launch_rocket(conn, vessel, heading, turn_altitude=10000):
    vessel.control.activate_next_stage()


def _stage_ready(resources_obj, autostage_resources, mode=const.AND):

    maxes = dict((res, resources_obj.max(res))for res in autostage_resources)

    for resource in autostage_resources:
        current = resources_obj.amount(resource)
        if current <= 0 or current / maxes[resource]:
            if mode == const.OR:
                return True
        elif mode == const.AND:
            return False

    if mode == const.AND:
        return True
    else:
        return False


def _get_autostage_stages(vessel):
    # I /think/ this needs the +1, as the dox say it corresponds to the
    # in-game UI, but I haven't checked yet
    count_stages = get_current_stage(vessel)

    print("count stages: {0}".format(count_stages))

    stages = []

    for stage_i in range(count_stages):
        stage_resources = vessel.resources_in_decouple_stage(stage=stage_i, cumulative=False)

        if set(resources.names).intersection(autostage_resources):
            stages.append(stage)
        elif stop_if_skipping:
            stages.append(None)

    return stages





def get_gravity_turn_interrupt(conn, def_altitude=10000):
    def gravity_turn_interrupt(vessel, altitude=def_altitude):



def autostage(
        vessel, stages=None,
        stop_if_skipping=True,
        interrupts=None,
        autostage_resources=const.RESOURCES_FUEL):

    if stages is None:
        stages = _get_autostage_stages(vessel)

    interrupts = set(interrupts)

    for stage in stages:
        if stage is None:
            raise NonEngineStage()
        while True:
            if _stage_ready(vessel.resources_in_decouple_stage(stage), stages):
                print("firing stage {0}".format(stage))
                time.sleep(0.5)
                vessel.control.activate_next_stage()
                print("stage {0} fired.".format(stage))
                break
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
            times.sleep(0.1)

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




