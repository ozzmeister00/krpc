"""
This file should contain all the programs that we can run from the console
"""
from __future__ import print_function, absolute_import, division

import math
import time

import krpc

from . import docking
from . import launch
from . import landing
from . import maneuvers
from . import node
from . import maths
from . import rendezvous
from . import rover
from . import utils
from .pid import PID


def Dock(connection, speedLimit=1.0):
    """
    Art Whaley's docking function, for testing

    :param connection: connection to start with
    :param speedLimit: maximum speed in m/s

    :return: None, this will crash out on success
    """
    # Setup KRPC
    sc = connection.space_center
    v = sc.active_vessel
    t = sc.target_docking_port
    ap = v.auto_pilot
    rf = v.orbit.body.reference_frame

    # Setup Auto Pilot
    ap.reference_frame = rf
    ap.target_direction = tuple(x * -1 for x in t.direction(rf))
    ap.engage()

    # create PIDs
    upPID = PID(.75, .25, 1)
    rightPID = PID(.75, .25, 1)
    forwardPID = PID(.75, .2, .5)

    proceed = False
    # 'proceed' is a flag that signals that we're lined up and ready to dock.
    # Otherwise the program will try to line up 10m from the docking port.

    # LineUp and then dock  - in the same loop with 'proceed' controlling whether
    # we're headed for the 10m safe point, or headed forward to dock.
    while True:
        offset = maths.getOffsets(v, t)  # grab data and compute setpoints
        velocity = maths.getVelocities(v, t)
        if docking.proceedCheck(offset):  # Check whether we're lined up and ready to dock
            proceed = True

        setpoints = docking.getSetPoints(offset, proceed, speedLimit)

        upPID.setpoint(setpoints.up)  # set PID setpoints
        rightPID.setpoint(setpoints.right)
        forwardPID.setpoint(setpoints.forward)

        v.control.up = -upPID.update(velocity.up)  # steer vessel
        v.control.right = -rightPID.update(velocity.right)
        v.control.forward = -forwardPID.update(velocity.forward)

        time.sleep(.05)


def ExecuteNextManeuver(connection=None, vessel=None, maneuverNode=None, autoStage=False):
    """
    Attempts to execute the next maneuver node for the input vessel and connection

    :param connection: the connection to work on, will use defaultConnection if none provided
    :param vessel: The vessel to control
    :param node: The node to execute, will use the vessel's next node by default
    :param autoStage: if the vessel should automatically trigger the next stage

    :return: success of the operation
    """
    if not connection:
        connection = utils.defaultConnection("ExecuteNextManeuver")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not maneuverNode:
        maneuverNode = vessel.control.nodes[0]

    doManeuver = node.ExecuteManeuver(connection, vessel, maneuverNode, tuneTime=10)
    autoStager = utils.AutoStage(vessel)

    # pre-check this
    if autoStage:
        autoStager()

    # maneuver control loop
    while not doManeuver():
        if autoStage:
            autoStager()

        time.sleep(0.05)

    vessel.control.sas = True
    vessel.control.throttle = 0.0
    vessel.auto_pilot.disengage()

    # remove the node!
    maneuverNode.remove()

    return True


def Launch(connection=None,
           vessel=None,
           altitude=250000,
           targetInclination=0.0,
           longitudeOfAscendingNode=None,
           autoStage=True,
           deployFairings=True):
    """
    Launches the input vessel on the input connection to the input altitude, inclination, argument of periapsis/ascending node

    :param connection: conneciton to operate upon
    :param vessel: vessel to operate upon
    :param altitude: target circular altitude
    :param inclination: target inclination
    :param targetInclination: in radians
    :param autoStage: if we should autostage the vessel
    :param longitudeOfAscendingNode: longitude of the ascending node we want to launch into
    :param deployFairings: if we should attempt to deploy fairings on the vessel
    """
    if not connection:
        connection = utils.defaultConnection("Launch")

    if not vessel:
        vessel = connection.space_center.active_vessel

    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    # if we've set a LAN, warp to it and creep up on it
    if longitudeOfAscendingNode:
        warpToTime, targetInclination = launch.calculateTimeToLaunch(connection,
                                                                     vessel,
                                                                     targetInclination,
                                                                     longitudeOfAscendingNode)
        if warpToTime > (ut() + 20):
            print("warping to launch window")
            connection.space_center.warp_to(warpToTime - 20)

        currentSolarLongitude = math.radians(vessel.flight().longitude) + vessel.orbit.body.rotation_angle
        while warpToTime > (ut() - 3):
            print("waiting to get close to the window {} {}".format(currentSolarLongitude, longitudeOfAscendingNode))
            currentSolarLongitude = math.radians(vessel.flight().longitude) + vessel.orbit.body.rotation_angle

            time.sleep(0.01)

    ascend = launch.Ascend(connection, vessel, targetAltitude=altitude, targetInclination=targetInclination)
    aborter = utils.Abort(vessel)
    staging = utils.AutoStage(vessel)
    fairing = utils.Fairing(connection, vessel)

    # count down to 0
    for i in range(3, 0, -1):
        time.sleep(1)

    # trigger the next stage to get us going
    vessel.control.activate_next_stage()

    # launch control loop
    while not ascend() and not aborter():
        if autoStage:
            staging()

        if deployFairings:
            fairing()

        time.sleep(0.1)

    # if we aborted, kill the throttle and bail out
    if aborter():
        vessel.control.throttle = 0.0
        return False

    vessel.control.throttle = 0.0

    # pause briefly before we move to the next bit
    time.sleep(1)

    # plot a circularization burn at apoapsis
    node = maneuvers.circularizeAtApoapsis(connection, vessel)
    ExecuteNextManeuver(connection, vessel, node, autoStage=autoStage)

    # trigger whatever is on action group 1, since I usually set that to be antenna and panels
    vessel.control.set_action_group(1, True)

    # let the calling software know if we aborted
    return aborter()


def Hover(connection=None, vessel=None):
    """
    Puts the vessel into a user-controlled hovering mode

    :param connection: connection to operate upon
    :param vessel: vessel to hover
    """
    if not connection:
        connection = utils.defaultConnection("Hover")
    if not vessel:
        vessel = connection.space_center.active_vessel

    hover = landing.Hover(connection, vessel)

    # if we have no thrust, try staging
    if not vessel.available_thrust:
        vessel.control.activate_next_stage()

    # hover control loop, keep it going until we've been bailed out
    while hover():
        time.sleep(0.01)

    vessel.control.throttle = 0.0
    vessel.control.sas = True


def LandAnywhere(connection=None, vessel=None):
    """
    Attempts to land the vessel on the input connection softly somewhere on its current orbiting body

    Best used around a body without atmosphere

    TODO Handle aborts

    :param connection: connection to operate on
    :param vessel: vessel to land
    """
    if not connection:
        connection = utils.defaultConnection("Landing")
    if not vessel:
        vessel = connection.space_center.active_vessel

    flight = vessel.flight(vessel.orbit.body.reference_frame)
    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    radius = vessel.orbit.body.equatorial_radius

    apoapsis = vessel.orbit.apoapsis_altitude
    periapsis = vessel.orbit.periapsis_altitude

    if periapsis > 31000:
        # get ourselves into a 30km x 30km parking orbit
        lowerPeriapsisNode = maneuvers.changePeriapsis(30000, connection, vessel)
        ExecuteNextManeuver(connection, vessel, maneuverNode=lowerPeriapsisNode)

    if apoapsis > 31000:
        lowerApoapsisNode = maneuvers.changeApoapsis(30000, connection, vessel)
        ExecuteNextManeuver(connection, vessel, maneuverNode=lowerApoapsisNode)

    #run the deorbit burn
    if periapsis > radius * -0.4:
        # deorbit to to PE = -(0.5 * body.radius) # TODO pick where the deorbit burn happens?
        deorbitPeriapsisHeight = radius * -0.5  # TODO this is gonna be broken
        deorbitPeriapsisNode = maneuvers.changePeriapsis(deorbitPeriapsisHeight, connection, vessel, ut()+300)
        ExecuteNextManeuver(connection, vessel, maneuverNode=deorbitPeriapsisNode)

    # now that we've set ourselves up on a suborbital trajectory, hand it over to the soft landing mode
    SoftLanding(connection, vessel)

    return True


def SoftLanding(connection=None, vessel=None):
    """
    Attempts to softly land the input vessel on the input connection
    assuming it is on a suborbital trajectory

    :param connection: connection to operate upon
    :param vessel: vessel to operate upon
    """
    if not connection:
        connection = utils.defaultConnection("Landing")

    if not vessel:
        vessel = connection.space_center.active_vessel

    descend = landing.Descend(connection, vessel)
    while descend():
        time.sleep(0.01)

    vessel.control.gear = True
    softTouchdown = landing.SoftTouchdown(vessel)
    while softTouchdown():
        time.sleep(0.01)

    vessel.control.throttle = 0.0
    vessel.control.sas = True
    vessel.auto_pilot.disengage()

    return True


def RendezvousWithTarget(connection=None, vessel=None, separation=None):
    """
    Given the input connection and vessel, plot and execute a rendezvous with the active target

    Will only plot the maneuver for rendezvous with a body, since that burn will result in lithobraking

    :param connection: Connection to plot for, will use default if none provided
    :param vessel: vessel to plot for, will use active if none provided
    :param separation: how far away from the target do we want to be. If none, will use defaults (400m, or edge of SOI)
    """
    if not connection:
        connection = utils.defaultConnection("Rendevous")
    if not vessel:
        vessel = connection.space_center.active_vessel

    # we'll need to know what kind of thing we're targeting later
    targetVessel = connection.space_center.target_vessel
    targetBody = connection.space_center.target_body

    # then, regardless of what we're targeting, stash it here
    target = targetVessel if targetVessel else targetBody

    # match planes with our target, if necessary
    matchPlanes = maneuvers.matchPlanes(connection, vessel, target)
    if matchPlanes:
        print("matching planes!")
        ExecuteNextManeuver(connection, vessel, matchPlanes)

    print("plotting maneuver")
    # plot the maneuver to meet our target
    hohmannTransfer = maneuvers.hohmannTransfer(connection, vessel, target)

    # if we're targeting a vessel, we can go ahead and try to get closer to it
    if targetVessel:
        ExecuteNextManeuver(connection, vessel, hohmannTransfer)

        # then, plot a maneuver that should roughly match our two vessels' orbits at closest approach
        timeOfClosestApproach = vessel.orbit.time_of_closest_approach(target.orbit)

        altitudeAtClosest = target.orbit.radius_at(timeOfClosestApproach) - target.orbit.body.equatorial_radius

        maneuverNode = maneuvers.changeApoapsis(altitudeAtClosest, connection, vessel, timeOfClosestApproach)

        # circularize at the point and altitude of closest approach so we can give getCloser a fighting chance
        ExecuteNextManeuver(connection, vessel, maneuverNode)

        # wait until closest approach
        connection.space_center.warp_to(vessel.orbit.time_of_closest_approach(target.orbit))

        print("getting closer")

        if not separation:
            separation = 400

        rendezvous.getCloser(connection, vessel, target, closeDistance=separation)

    else:
        # plot our hohmann transfer
        doManeuver = node.ExecuteManeuver(connection, vessel, hohmannTransfer, tuneTime=10)

        # if there's a "next_orbit" that means we're breaking out of our SOI
        # and that's all we need for body rendezvous
        while not doManeuver() and not vessel.orbit.next_orbit:
            time.sleep(0.05)

        # kill the autopilot
        vessel.control.sas = True
        vessel.control.throttle = 0.0
        vessel.auto_pilot.disengage()

        # remove the node now that we're done with it
        hohmannTransfer.remove()

        # warp to SOI change
        connection.space_center.warp_to(connection.space_center.ut + vessel.orbit.time_to_soi_change)

        # circularize at periapsis
        maneuvers.circularizeAtPeriapsis(connection, vessel)
        ExecuteNextManeuver(connection, vessel)

        # if the user provided a target orbit, go for it
        if separation:
            # since we're circular, we don't need to wait until apoapsis to lower our periapsis
            maneuvers.changePeriapsis(separation, connection, vessel, atTime=connection.space_center.ut + 300)
            ExecuteNextManeuver(connection, vessel)

            maneuvers.circularizeAtPeriapsis(connection, vessel)
            ExecuteNextManeuver(connection, vessel)


def DeorbitIntoAtmosphere(connection=None, vessel=None):
    """
    Deorbits the input vessel to ~ 30000m above the surface,
    keeps the vessel oriented to retrograde, and fires
    parachutes when it's safe

    :param connection: connection to operate upon
    :param vessel: vessel to operate upon
    """
    if not connection:
        connection = utils.defaultConnection("Deorbit")
    if not vessel:
        vessel = connection.space_center.active_vessel

    # if our periapsis is above a target
    if vessel.orbit.periapsis_altitude > 31000:
        deorbit = maneuvers.changePeriapsis(30000, connection=connection, vessel=vessel)

        ExecuteNextManeuver(connection, vessel, deorbit)

    vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
    vessel.auto_pilot.engage()

    print("jettisoning everything")
    vessel.control.activate_next_stage()

    while vessel.flight().surface_altitude > 10000:
        vessel.auto_pilot.target_direction = vessel.flight(vessel.surface_reference_frame).retrograde
        time.sleep(0.1)

    # TODO this is a dumb way to deploy fairings
    vessel.control.activate_next_stage()

    vessel.auto_pilot.disengage()


def RoveToTarget(connection=None, vessel=None, target=None, saveInterval=120, maxSpeed=5.0):
    """
    For a vessel that's landed on a planet, auto-rove to the in put target (another vessel)

    :param connection: the connection to use
    :param vessel: the vessel to control
    :param target: the target vessel to rove to
    :param saveInterval: how frequently should we stop and save
    :param maxSpeed: how fast should we go
    """

    if not connection:
        connection = utils.defaultConnection("RoveToTarget")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not target:
        target = connection.space_center.target_vessel

    flight = target.flight()

    latitude = flight.latitude
    longitude = flight.longitude

    print("setting up waypoint")
    # add a waypoint above our target
    wp1 = connection.space_center.waypoint_manager.add_waypoint(latitude, longitude, vessel.orbit.body, "Target")

    roverGo = rover.RoverGo(connection, vessel, wp1, maxSpeed, savetime=saveInterval)

    # call the rover autopilot
    while not roverGo():
        time.sleep(0.01)

    # remove the waypoint when the function returns
    wp1.remove()
