from collections import deque

from bearlibterminal import terminal

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


class Display(object):
    def __init__(self, connection, vessel, program=None):
        self.vessel = vessel
        self.connection = connection
        self.program = program
        self.messages = deque(maxlen=5)

        terminal.open()
        terminal.set('window: size={}x{}'.format(len(templateStr[1]) + 2, len(templateStr) + 2))

        for i, line in enumerate(templateStr):
            terminal.printf(0, i, ' {} '.format(line))

        terminal.refresh()

    def changeProgram(self, newProgram=None):
        self.program = newProgram

    def putValues(self, x, y, values):
        for i, value in enumerate(values):
            terminal.printf(x, y + i, str(value))

    def addMessage(self, message):
        print(message)
        self.messages.appendleft(message)

    def __call__(self):
        situations = {self.vessel.situation.docked: 'Docked',
                      self.vessel.situation.escaping: 'Escaping',
                      self.vessel.situation.flying: 'Flying',
                      self.vessel.situation.landed: 'Landed',
                      self.vessel.situation.orbiting: 'Orbit',
                      self.vessel.situation.pre_launch: 'PreLaunch',
                      self.vessel.situation.splashed: 'Splashed',
                      self.vessel.situation.sub_orbital: 'SubOrbit',
                      }

        y = 2  # start height
        x1 = 9  # start width of the vessel info display
        x2 = 21  # start width of the program info display

        # Messages
        x3 = 3
        y2 = 16

        # blank the display to prevent overdraw
        for i in range(12):
            terminal.printf(x1, y + i, ''.ljust(10, ' '))
            terminal.printf(x2, y + i, ''.ljust(20, ' '))

        # blank the messages
        for i in range(5):
            terminal.printf(x3, y2 + i, ''.ljust(38, ' '))

        # print the vessel stats to the screen
        vesselValues = [self.vessel.name[:10],
                        round(self.vessel.orbit.apoapsis_altitude, 0),
                        round(self.vessel.orbit.periapsis_altitude, 0),
                        round(self.vessel.orbit.eccentricity, 6),
                        round(self.vessel.orbit.inclination, 2),
                        round(self.vessel.met),
                        round(self.vessel.flight().speed, 2),
                        round(self.vessel.flight().mean_altitude, 2),
                        round(self.vessel.flight().g_force, 2),
                        round(self.vessel.flight().atmosphere_density),
                        situations[self.vessel.situation],
                        self.vessel.control.current_stage
                        ]

        manualValues = ['Mode : Manual',
                        'RdAlt: ',
                        ' ',
                        'Mass : ',
                        'LqFul: ',
                        'Ox   : ',
                        ' ',
                        ' ',
                        ' ',
                        ' ',
                        ' ',
                        ' '
                        ]

        self.putValues(x1, y, vesselValues)

        if self.program:
            self.putValues(x2, y, self.program.displayValues())

            if self.program.messages:
                self.messages.appendleft(self.program.messages.popleft())

        # if there's no program, print more vessel information
        else:
            self.putValues(x2, y, manualValues)

        self.putValues(x3, y2, self.messages)

        terminal.refresh()
