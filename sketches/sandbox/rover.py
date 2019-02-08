######################################################################
### Rover Controller Library and Example
######################################################################
###   Like all of the scripts in my folder here, this file contains
###   functions you might want to include into your own scripts for
###   actual use and a demo in the 'main' function that you can just
###   run to see how it works.
###
###   This file shows how to make a rover navigate to a waypoint.
###   The function has a few advanced features - most interestingly
###   it autosaves at a fixed interval, if the rover is stable and
###   moving and seems to be intact.  This is an effort to fix the
###   frustration of having a rover crash when you're an hour from
###   your last quick save.
######################################################################

import krpc
import time
import math
from collections import namedtuple
from math import atan2, degrees, cos, sqrt, asin, sin, radians

from pid import PID  # imports my PID controller from the PID example file!

latlon = namedtuple('latlon', 'lat lon')  # create a named tuple


##############################################################################
##  Main function -  this exists to demonstrate the use of the library.  If
##  you import this library into your own script, main doesn't get called.
##############################################################################
def main():
    conn = krpc.connect()

    # Create a waypoint to drive to
    #wp1 = conn.space_center.waypoint_manager.add_waypoint(
    #    0.05, -75.0, conn.space_center.active_vessel.orbit.body, "Waypoint1")

    target = conn.space_center.target_vessel
    vessel = conn.space_center.active_vessel
    flight = target.flight()

    latitude = flight.latitude
    longitude = flight.longitude

    wp1 = conn.space_center.waypoint_manager.add_waypoint(latitude, longitude, vessel.orbit.body, "Target")

    # call the rover autopilot
    rover_go(conn, wp1, 2.5, savetime=120)

    # remove the waypoint when the function returns
    wp1.remove()


##############################################################################
##  And here's the function that's actually interesting!
##############################################################################
def rover_go(conn, waypoint, speed=10.0, savetime=300):
    '''
    Function to drive a rover to the specified waypoint.  Must be called with
    an active KRPC connection and a valid waypoint.   Attempts to bring rover
    to a complete stop and quicksave a file called "rover_ap" at a regular
    interval.  Defaults to 5 minutes.   This and rover speed can be specified
    as optional arguments.  A savetime of 0 turns this feature off.
    '''

    ## grab hold of the krpc functions we'll need to drive the rover
    v = conn.space_center.active_vessel
    ground_telem = v.flight(v.orbit.body.reference_frame)
    surf_telem = v.flight(v.surface_reference_frame)
    target = latlon(waypoint.latitude, waypoint.longitude)

    autosave.lastsave = time.time()
    partslist = v.parts.all
    there_yet = False

    ## Setup the PID controllers for steering and throttle.   The steering
    ## setpoint is locked to 0 since we'll be feeding in an error number to
    ## the update function.
    steering = PID(.01, .01, .001)
    throttle = PID(.5, .01, .001)
    steering.setpoint(0)
    throttle.setpoint(speed)

    location = latlon(ground_telem.latitude, ground_telem.longitude)

    # The main loop that drives to the way point
    while not there_yet:

        autosave(conn, savetime, partslist, target, location)  ##call autosave to see if we should save yet
        recharge(conn)

        ##  Steering control - handles selecting a bearing, comparing it to
        ##  current heading and feeding that error in degrees to the PID to
        ##  get the control correction required.
        location = latlon(ground_telem.latitude, ground_telem.longitude)
        target_heading = heading_for_latlon(target, location)
        course_correct = course_correction(surf_telem.heading, target_heading)
        steer_correct = steering.update(course_correct)
        v.control.wheel_steering = steer_correct

        #print(location, target_heading, v.control.wheel_steering)

        # Throttle control  -  tries to maintain the given speed!
        v.control.brakes = False
        throttsetting = throttle.update(ground_telem.speed)
        v.control.wheel_throttle = throttsetting

        # Check if we're there to end the loop
        if distance(target, location, v.orbit.body) < 50:
            print("we're there!")
            there_yet = True

    v.control.brakes = True # turn the brakes on


##############################################################################
###  Autosave function.   Saves if the vessel appears stable and isn't already
###  stopped, pitched greater than 30 degrees, or showing a different part
###  count than before.
##############################################################################

def autosave(conn, savetime, partslist, target, location):
    if savetime == 0:
        print("we literally just saved")
        return
    if time.time() - autosave.lastsave > savetime:
        print("it's time to save!")
        v = conn.space_center.active_vessel
        telem = v.flight(v.orbit.body.reference_frame)
        surf = v.flight(v.surface_reference_frame)

        if safetosave(conn, partslist):
            v.control.throttle = 0.0  ## Stop the rover then save
            v.control.brakes = True
            while surf.speed > 0.01:
                print("slowing down")
                pass
            time.sleep(.1)
            print("saving {}m away".format(distance(target, location, v.orbit.body)))
            conn.space_center.save('rover_ap')
            print("and on we go!")
            v.control.brakes = False
        autosave.lastsave = time.time()


# function called by autosave to determine if it's safe to save the file!
# tries to avoid overwriting a good save with one we take AFTER the rover
# has crashed or flipped or run into a space tree.
def safetosave(conn, partslist):
    v = conn.space_center.active_vessel
    ground_telem = v.flight(v.orbit.body.reference_frame)
    surf_telem = v.flight(v.surface_reference_frame)

    if ground_telem.speed < .1:  # We might be stuck!
        print("stuck")
        return False
    if surf_telem.pitch > 25 or surf_telem.roll > 25:  # We might have rolled!
        print("roll")
        return False
    if len(partslist) is not len(v.parts.all):  # We might have lost something?
        print("broken")
        return False
    print("it's safe to save")
    return True  # all good!


##############################################################################
##  Battery Charging Function - if the batteries are below 5% - stops rover
##  and deploys solar panels until charge is above 85% then resumes travel.
##############################################################################
def recharge(conn):
    vessel = conn.space_center.active_vessel
    telem = vessel.flight(vessel.orbit.body.reference_frame)
    Max_EC = vessel.resources.max('ElectricCharge')
    EC = vessel.resources.amount('ElectricCharge')
    if EC / Max_EC < .05:  # less than 5% charge - Stop the rover
        print("outta gas! try again")
        vessel.control.wheel_throttle = 0
        vessel.control.brakes = True
        while telem.speed > 0.01:
            print("slowing down")
            pass
        vessel.control.solar_panels = True
        while EC / Max_EC < .85:  # less than 85% charge
            Max_EC = vessel.resources.max('ElectricCharge')
            EC = vessel.resources.amount('ElectricCharge')
        vessel.control.solar_panels = False  ##pack up and get moving again
        vessel.control.brakes = False


##############################################################################
##  Navigation Math Functions
##############################################################################
def heading_for_latlon(target, location):
    lat1 = math.radians(location.lat)
    lat2 = math.radians(target.lat)

    diffLong = math.radians(target.lon - location.lon)

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
                                           * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing


def distance(target, location, body):
    R = body.equatorial_radius  # Earth radius in kilometers

    dLat = radians(target.lat - location.lat)
    dLon = radians(target.lon - location.lon)
    lat1 = radians(location.lat)
    lat2 = radians(target.lat)

    a = sin(dLat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dLon / 2) ** 2
    c = 2 * asin(sqrt(a))

    return R * c


def course_correction(myheading, targetbearing):
    unadjusted = targetbearing - myheading
    if unadjusted < -180:
        return unadjusted + 360
    if unadjusted > 180:
        return unadjusted - 360
    return unadjusted


# ----------------------------------------------------------------------------
# Activate main loop, if we are executing THIS file explicitly.
# ----------------------------------------------------------------------------
if __name__ == "__main__":
    print("main")
    main()
