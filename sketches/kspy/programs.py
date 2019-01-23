"""
This file should contain all the programs that we can run from the console
"""

def getConnection():
    connection = krpc.connect()
    # connection = krpc.connect(address='192.168.1.64',
    #                            stream_port=50001,
    #                            rpc_port=50000)

    return connection

def ExecuteNextManeuver():
    doManeuver = ExecuteManeuver(connection, vessel, tuneTime=20)
    autostage = AutoStage(vessel)

    while not doManeuver():
        # autostage()
        time.sleep(0.05)

    vessel.control.sas = True
    vessel.control.throttle = 0.0

def Launch():
    # todo make a conneciton utility so I only ever have to define that in one file
    connection = getConnection()
    vessel = connection.space_center.active_vessel
    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    ascend = Ascend(connection, vessel, 500000)
    staging = AutoStage(vessel)
    fairing = Fairing(connection, vessel)

    for i in range(3, 0, -1):
        time.sleep(1)

    display.addMessage('Launch!')

    vessel.control.activate_next_stage()

    while not ascend() and not hasAborted(vessel):
        display()
        staging()
        fairing()
        time.sleep(0.1)

    if hasAborted(vessel):
        display.addMessage('Good luck!')
        sys.exit(1)

    vessel.control.throttle = 0.0

    time.sleep(1)

    display.addMessage('Circularizing')
    node = changePeriapsis(vessel, ut(), vessel.orbit.apoapsis_altitude)
    doManeuver = ExecuteManeuver(connection, vessel, node=node, tuneTime=5, leadTime=60)

    display.changeProgram(doManeuver)

    while not doManeuver() and not hasAborted(vessel):
        display()
        staging()
        time.sleep(0.05)

    node.remove()

    display.addMessage('Welcome to space!')
    display.changeProgram(None)

def Hover():
    connection = krpc.connect("Hover")
    vessel = connection.space_center.active_vessel
    hover = Hover(connection, vessel)
    display = Display(connection, vessel, program=hover)

    if not vessel.available_thrust:
        vessel.control.activate_next_stage()

    while hover():
        display()
        time.sleep(0.01)

    vessel.control.throttle = 0.0
    vessel.control.sas = True

def LandAnywhere():
    connection = krpc.connect("Landing")
    vessel = connection.space_center.active_vessel
    flight = vessel.flight(vessel.orbit.body.reference_frame)
    display = Display(connection, vessel)
    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    display()

    surface_altitude = connection.add_stream(getattr, flight, 'surface_altitude')

    g = vessel.orbit.body.gravitational_parameter

    radius = vessel.orbit.body.equatorial_radius

    apoapsis = vessel.orbit.apoapsis_altitude
    periapsis = vessel.orbit.periapsis_altitude

    if periapsis > 31000:
        display.addMessage("Lowering Periapsis")
        display()
        # get ourselves into a 30km x 30km parking orbit
        lowerPeriapsisNode = changePeriapsis(vessel, ut(), 30000)
        lowerPeriapsis = ExecuteManeuver(connection, vessel, node=lowerPeriapsisNode)
        display.changeProgram(lowerPeriapsis)
        while not lowerPeriapsis():
            display()
            time.sleep(0.01)

    if apoapsis > 31000:
        display.addMessage("Lowering Apoapsis")
        display()
        lowerApoapsisNode = changeApoapsis(vessel, ut(), 30000)
        lowerApoapsis = ExecuteManeuver(connection, vessel, node=lowerApoapsisNode)
        display.changeProgram(lowerApoapsis)
        while not lowerApoapsis():
            display()
            time.sleep(0.01)

    #run the deorbit
    if periapsis > radius * -0.5:
        # deorbit to to PE = -(0.5 * body.radius) # TODO pick where the deorbit burn happens?
        deorbitPeriapsisHeight = radius * -0.5
        deorbitPeriapsisNode = changePeriapsis(vessel, ut()+300, deorbitPeriapsisHeight)
        deorbit = ExecuteManeuver(connection, vessel, node=deorbitPeriapsisNode)
        display.changeProgram(deorbit)
        display.addMessage("Deorbit burn")
        while not deorbit():
            display()
            time.sleep(0.01)

    # TODO is warping to descend?

    # hoverslam
    descend = Descend(vessel, connection)
    display.changeProgram(descend)
    display.addMessage("Hoverslam")
    display()
    while descend():
        display()
        time.sleep(0.01)

    display.addMessage("moving to soft landing")
    display()

    vessel.control.gear = True
    softTouchdown = SoftTouchdown(vessel, connection)
    display.changeProgram(softTouchdown)
    while softTouchdown():
        display()
        time.sleep(0.1)

    vessel.control.throttle = 0.0
    vessel.control.sas = True
    vessel.auto_pilot.disengage()

    display.addMessage("landed")
    display()


def InsightLanding():
    def vessel():
        return conn.space_center.active_vessel

    conn = krpc.connect("Insight")

    drogues = vessel().parts.with_tag("drogue")
    mains = vessel().parts.with_tag("main")
    heatshield = vessel().parts.with_tag("heatShield")[0]
    fairingSep = vessel().parts.with_tag("fairingSep")
    cruiseSep = vessel().parts.with_tag("cruiseSep")

    #vessel.auto_pilot.reference_frame = vessel.reference_frame
    vessel().auto_pilot.engage()
    # point at sun until 50000 meters
    sun = conn.space_center.bodies['Sun']

    # this doesn't quite point right, but we're getting what we need out of it
    print("waiting until 60000m")
    while vessel().flight().mean_altitude > 60000:
        rf = vessel().reference_frame
        pos = sun.position(rf)
        vessel().auto_pilot.target_direction = pos
        time.sleep(0.05)

    print("decoupling cruise stage")
    # deploy cruise stage
    for part in cruiseSep:
        part.decoupler.decouple()

    print("pointing retrograde")
    # point retrograde
    vessel().auto_pilot.target_direction = vessel().flight().retrograde

    print("waiting until we're out of plasma")
    # wait until heatshield thermal flux is less than 0
    isOutOfPlasma = False
    underAltitude = False
    while not isOutOfPlasma and not underAltitude:
        underAltitude = vessel().flight().mean_altitude < 15000.0
        isOutOfPlasma = heatshield.thermal_convection_flux < 0.0
        time.sleep(0.05)

    next = vessel().flight().mean_altitude - 1000

    print("waiting until a bit more")
    while vessel().flight().surface_altitude > next:
        time.sleep(0.05)

    print("decoupling heat shield")
    heatshield.decoupler.decouple()

    print("deploying drouges")
    for drogue in drogues:
        if drogue.parachute:
            drogue.parachute.deploy()

    print("waiting until 2500")
    while vessel().flight().surface_altitude > 2500.0:
        print(vessel().flight().surface_altitude)
        time.sleep(0.1)

    print("deploying mains")
    for m in mains:
        m.parachute.deploy()

    print("waiting until 1000m")
    while vessel().flight().surface_altitude > 2000.0:
        time.sleep(0.05)

    print("separating fairing stage'")
    for i in fairingSep:
        if i.fairing:
            i.fairing.jettison()
        elif i.decoupler:
            i.decoupler.decouple()
        elif i.engine:
            i.engine.active = True
        else:
            print("What is a {}".format(i.name))

    # fire the engines!
    vessel().control.activate_next_stage()

    softLanding()

def SoftLanding():
    connection = krpc.connect("Landing")
    vessel = connection.space_center.active_vessel

    descend = Descend(vessel, connection)
    print("descending under suicide burn")
    while descend():
        time.sleep(0.01)

    vessel.control.gear = True
    softTouchdown = SoftTouchdown(vessel, connection)
    print("activating soft touchdown")
    while softTouchdown():
        time.sleep(0.01)

    print("touchdown")
    vessel.control.throttle = 0.0
    vessel.control.sas = True
    vessel.auto_pilot.disengage()

def RendevousWithTarget():
    conn = getConnection()
    sc = conn.space_center
    v = conn.space_center.active_vessel
    t = conn.space_center.target_vessel
    # t = conn.space_center.target_body

    match_planes(conn)
    hohmann(conn)
    circularize_at_intercept(conn)
    get_closer(conn)
    print ("Rendezvous Complete.")