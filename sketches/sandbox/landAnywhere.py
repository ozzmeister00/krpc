import krpc

from sandbox.gui_testing import Display
from sandbox.utils import Program

def main():
    connection = krpc.connect("Landing")
    vessel = connection.space_center.active_vessel
    flight = vessel.flight(vessel.orbit.body.reference_frame)
    display = Display(connection, vessel)

    display()
    display.addMessage("Lowering to stable orbit")
    display()

    targetOrbit = 30000
    targetPeriapsis = (vessel.orbit.body.mean_radius / 3) * 2

    targetBurnLng = 50
    landingBurnStartAlt = 4500
    landingLowHeight = 25
    landingMinSpeed = 2.5

    cutoffHeight = 100

    latitude = connection.add_stream(getattr, flight, 'latitude')
    longitude = connection.add_stream(getattr, flight, 'longitude')

    elevation = connection.add_stream(getattr, flight, 'elevation')

    g = vessel.orbit.body.gravitational_parameter

    vessel.auto_pilot.sas = True
    vessel.auto_pilot.sas_mode = vessel.control.sas_mode.retrograde
    vessel.auto_pilot.target_roll = float("nan")
    vessel.auto_pilot.engage()

    '''// locked values
lock shipLatLng to SHIP:GEOPOSITION.
lock surfaceElevation to shipLatLng:TERRAINHEIGHT.
lock betterALTRADAR to max(0.1, ALTITUDE - surfaceElevation).
lock impactTime to betterALTRADAR / -VERTICALSPEED.
set GRAVITY to (constant():G * body:mass) / body:radius ^ 2.
lock TWR to MAX(0.001, MAXTHRUST / (MASS*GRAVITY)).

// initialization
SAS ON.
SET SASMODE TO "STABILITY".
RCS off.
lock throttle to 0.
gear off.
panels on.

clearscreen.

set updateHeading to true.
set targetHeading to RETROGRADE.
set tval to 0.

set ORBITBODY to SHIP:BODY.
set MAXACC to SHIP:MAXTHRUST/SHIP:MASS.

// safety checks
set runmode to 100.
	
if ALTITUDE > 5000 {
	set runmode to 1.
}

// if the orbit is already in a good orbit, skip to deorbiting.
if PERIAPSIS < TARGETORBIT * 1.05 and APOAPSIS < TARGETORBIT * 1.05 {
	set runmode to 5.
}
	
// main loop
until runmode = 0 {
	set updateHeading to true.
	set targetHeading to RETROGRADE.

	// warp to apoapsis
	if runmode = 1 {
		if (SHIP:ALTITUDE > TARGETORBIT) and (ETA:APOAPSIS > 60) {
			if WARP = 0 {
				wait 1.
				set WARP to 3.
			}
		}
		else if ETA:APOAPSIS < 60 {
			set WARP to 0.
			set runmode to 2.
		}
		else {
			print "SHIP IS OUT OF POSITION".
			set runmode to 0.
		}
	}
	
	// burn until periapsis is at target value
	else if runmode = 2 {
		set targetHeading to RETROGRADE.
		if (ETA:APOAPSIS < 5) or (VERTICALSPEED < 0) {
			// tune the throttle based on how close we're getting to the target orbit
			// may be even more useful for executeNextManeuver
			set tval to MAX(.1, PERIAPSIS/TARGETORBIT).
		}
		else {
			set tval to 0.
		}
		
		if PERIAPSIS < TARGETORBIT {
			set tval to 0.
			set runmode to 3.
		}
	}
	
	// time warp to periapsis
	else if runmode = 3 {
		set tval to 0.
		if (SHIP:ALTITUDE > TARGETORBIT) and (ETA:PERIAPSIS > 60) {
			if WARP = 0 {
				wait 1.
				set WARP to 3.
			}
		}
		else if ETA:PERIAPSIS < 60 {
			set WARP to 0.
			set runmode to 4.
		}
	}
	
	// lower the apoasis to the descent orbit
	else if runmode = 4 { 
		set targetHeading to RETROGRADE.
		if ETA:PERIAPSIS < 5 or VERTICALSPEED > 0 {
			set tval to MAX(.1, APOAPSIS/TARGETORBIT).
		}
		else {
			set tval to 0.
		}
		
		if APOAPSIS < MAX(TARGETORBIT, PERIAPSIS * 1.05) {
			set tval to 0.
			set runmode to 5.
		}
	}
	
	// timewarp to deorbit burn
	else if runmode = 5 {
		set tval to 0.
		if (shipLatLNG:LNG < TARGETBURNLNG - 20 or shipLatLng:LNG > TARGETBURNLNG + 1) {
			if WARP = 0 {
				wait 1.
				set WARP to 2.
			}
		}
		else {
			set WARP to 0.
			set runmode to 6.
		}
	}
	
	// deorbit burn
	else if runmode = 6 {
		set targetHeading to RETROGRADE.
		if shipLatLng:LNG > TARGETBURNLNG and shipLatLng:LNG < TARGETBURNLNG + 2 {
			set tval to 0.5.
		}
		if PERIAPSIS < TARGETPERIAPSIS {
			set tval to 0.
			set runmode to 7.
		}
	}
	
	// coast until we're close to the ground
	else if runmode = 7 {
		set targetHeading to velocity:surface * -1. // surface retrograde
		set tval to 0.
	
		if ALTITUDE > 20000 {
			wait 1.
			set WARP to 2.
		}
		else if ALTITUDE < 20000 and WARP > 0 {
			set WARP to 0.
		}
		if verticalSpeed < -1 and betterALTRADAR < LANDINGBURNSTARTALTITUDE {
			set runmode to 8.
		}
	}
	
	// landing burn
	else if runmode = 8 {
		set targetHeading to velocity:surface * -1.
		set landingRadar to min(ALTITUDE, betterALTRADAR).
		
		// hover thrust around until we're close to the ground
		set tval to (1 / TWR) - (verticalSpeed + max(landingMinSpeed, min(LANDINGSLOWHEIGHT, landingRadar^1.08 / 8))) / 3 / TWR.
		
		gear on.
		
		if betterALTRADAR < CUTOFFHEIGHT and ABS(VERTICALSPEED < 1) {
			set tval to 0.
			set targetHeading to UP.
			print "LANDED!".
			run hover.
		}
	}
	
	if updateHeading = true { lock STEERING to targetHeading. }
	else { set SASMODE to "STABILITY". }
	
	lock throttle to tval.
	
	// status prinouts
	print "RUNMODE:         " + runmode + "     " at (5,4).
	print "STAGE:LIQUIDFUEL " + STAGE:LIQUIDFUEL + "    " at (5,5).
	print "ALTRADAR:        " + round(betterALTRADAR) + "     " at (5,6).
	print "APOAPSIS:        " + round(ship:apoapsis) + "     " at (5,7).
	print "PERIAPSIS:       " + round(ship:periapsis) + "     " at (5,8).
	print "============= Landing Data =============    " at (5,9).
	print "THROTTLE:        " + round(tval, 2) + "        " at (5,10).
	print "LONGITUDE:       " + round(shipLatLng:LNG, 3) at (5,11).'''



if __name__ == '__main__':
    main()