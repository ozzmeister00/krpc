import time

from bearlibterminal import terminal

import krpc

templateStr = '''
+- Flight --------+- Program -----------+
| Name:           |                     |
| Ap  :           |                     |
| Pe  :           |                     |
| Ecc :           |                     |
| Inc :           |                     |
| MET :           |                     |
| Sp  :           |                     |
| Alt :           |                     |
| G   :           |                     |
| Pres:           |                     |
| Sit :           |                     |
| Stg :           |                     |
+-----------------+---------------------+
+- Messages ----------------------------+
|                                       |
|                                       |
|                                       |
|                                       |
|                                       |
+---------------------------------------+
+- Actions -----------------------------+
|       |       |       |       |       |
|  Ag1  |  Ag2  |  Ag3  |  Ag4  |  Ag5  |
|       |       |       |       |       |
+---------------------------------------+
|       |       |       |       |       |
|  Ag6  |  Ag7  |  Ag8  |  Ag9  |  Ag0  |
|       |       |       |       |       |
+---------------------------------------+
|       |       |       |       |       |
| Gears | Light | Brake |  SAS  |  RCS  |
|       |       |       |       |       |
+---------------------------------------+
|       |       |       |       |       |
| Wheel | Solar | Fairs | Abort | Cargo |
|       |       |       |       |       |
+---------------------------------------+
|       |       |       |       |       |
| Legs  | Intke | Chute | Comms | Rads  |
|       |       |       |       |       |
+---------------------------------------+
'''.split('\n')

conn = krpc.connect("GUI")
vessel = conn.space_center.active_vessel
flight = vessel.flight(vessel.orbit.body.reference_frame)


terminal.open()

terminal.set('window: size={}x{}'.format(len(templateStr[1]) + 2, len(templateStr) + 2))

for i, line in enumerate(templateStr):
    terminal.printf(0, i, ' {} '.format(line))

terminal.refresh()

y = 2

situations = {vessel.situation.docked:'Docked',
vessel.situation.escaping:'Escaping',
vessel.situation.flying:'Flying',
vessel.situation.landed:'Landed',
vessel.situation.orbiting:'Orbit',
vessel.situation.pre_launch:'PreLaunch',
vessel.situation.splashed:'Splashed',
vessel.situation.sub_orbital:'SubOrbit',
}

while True:
    for i in range(12):
        terminal.printf(9, y+i, '          ')

    terminal.printf(9, y, vessel.name) #| Name:
    terminal.printf(9, y+1, str(round(vessel.orbit.apoapsis_altitude, 0))) #| Ap:
    terminal.printf(9, y+2, str(round(vessel.orbit.periapsis_altitude, 0))) #| Pe:
    terminal.printf(9, y+3, str(round(vessel.orbit.eccentricity, 6))) #| Ecc:
    terminal.printf(9, y+4, str(round(vessel.orbit.inclination, 2))) #| Inc:
    terminal.printf(9, y+5, str(round(vessel.met))) #| MET:
    terminal.printf(9, y+6, str(round(flight.speed, 2))) #| Sp:
    terminal.printf(9, y+7, str(round(flight.mean_altitude, 2))) #| Alt:
    terminal.printf(9, y+8, str(round(flight.g_force, 2))) #| G:
    terminal.printf(9, y+9, str(round(flight.atmosphere_density))) #| Pres:
    terminal.printf(9, y+10, situations[vessel.situation]) #| Sit:
    terminal.printf(9, y+11, str(vessel.control.current_stage)) #| Stg:

    terminal.refresh()

    time.sleep(0.1)


terminal.close()

