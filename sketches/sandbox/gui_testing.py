from bearlibterminal import terminal

import krpc

#conn = krpc.connect("GUI")
#vessel = conn.space_center.active_vessel

terminal.open()
terminal.printf(2, 1, "Wello, Horld")
terminal.refresh()
while terminal.read() != terminal.TK_CLOSE:
    pass
terminal.close()