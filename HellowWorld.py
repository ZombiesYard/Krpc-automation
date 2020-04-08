import krpc

conn = krpc.connect(name='Hello World')

vessel = conn.space_center.active_vessel

print(vessel.name, vessel.type, vessel.situation, vessel.recoverable)
print(vessel.mass)
print(vessel.flight.g_force)




#flight_info = vessel.flight()

#print(flight_info.mean_altitude)
#conn.krpc.get_status()
#print(conn.krpc.get_status())
#print(get_status())

#refframe = vessel.orbit.body.reference_frame
#position = conn.add_stream(vessel.position, refframe)
#while True:
    #print(vessel.position(refframe))
    #print(position())

#help(conn.space_center)

#help(conn.space_center.Vessel)