"""
This should go away soon, but just making sure I have this working correctly.
"""

import ozzybear_krpc.vessel.util

from ozzybear_krpc import util


def hello_world():

    conn = util.get_conn('hello world!')
    vessel = ozzybear_krpc.vessel.util.get_active_vessel(conn)
    print(vessel.name)
