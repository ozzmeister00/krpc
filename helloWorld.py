import krpc

conn = krpc.connect('Hello, World')
vessel = conn.space_center.active_vessel
print(vessel.name)