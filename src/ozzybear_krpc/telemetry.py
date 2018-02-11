"""
telemetry.py

Class and functions for reading data from the server
"""

from ozzybear_krpc import const


class StreamManager(object):
    _instance = None

    def __new__(cls, *args, **kwargs):

        if cls._instance is None:
            cls._instance = object.__new__(cls, *args, **kwargs)

        return cls._instance

    def __init__(self, conn):
        self._conn = conn
        self._streams = {}

        super(StreamManager, self).__init__()

    def _add_stream(self, *args):
        return self._conn.add_stream(*args)

    def add_apoapsis_altitude(self, vessel):
        key = 'vessel[{0}]'.format(vessel._object_id)
        stream = self._conn.add_stream()

    def add_accelleration_(self, vessel, reference_frame):
        key = 'vessel[{0}]'.format(vessel._object_id)
        stream = self._add_stream(getattr, vessel.flight)

    def add_vessel_altitude(self, vessel):
        key = vessel
        stream = self._add_stream(getattr, vessel.flight, const.MEAN_ALTITUDE)

    def get_stage_resource_stream(vessel, stage, resource, cumulative=False):
        key = '{0}'

        resource_obj = vessel.resources_in_decouple_stage(stage=stage, cumulative=cumulative)
        stream = self._conn.add_stream(resource_obj, resource)


