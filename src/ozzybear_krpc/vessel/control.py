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


def get_current_stage(vessel):
    return vessel.control.current_stage + 1


def get_stage_resources_native(vessel, stage=None, cumulative=False):
    if stage is None:
        stage = get_current_stage(vessel)

    resources = vessel.resources_in_decouple_stage(stage=stage, cumulative=cumulative)

    return resources


def get_stage_resource_amounts(vessel, stage=None, cumulative=False, res_filter=frozenset()):

    resources = get_stage_resources_native(vessel, stage=stage, cumulative=cumulative)
    # TODO cleanup on aisle too dense
    resource_dict = dict((name, resources.amount(name) for name in resources.names if name not in res_filter))

    return resource_dict


def _stage_ready(resources_obj, autostage_resources, mode=const.AND):
    checks = []

    maxes = dict((res, resources_obj.max(res))for res in autostage_resources)

    for resource in autostage_resources:
        current = resources_obj.amount(resource)
        if current <= 0 or current / maxes[resource]:
            if mode == const.OR:
                return True
            checks.append(True)


    if mode == const.AND:



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


def autostage(conn, vessel, stages=None, stop_if_skipping=True, autostage_resources=const.RESOURCES_FUEL):

    if stages is None:
        stages = _get_autostage_stages(vessel)

    for stage in stages:
        while True:





