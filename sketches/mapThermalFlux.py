import krpc
import time

outCSV = [("ut", "alt", "speed", "conduction", "convection", "radiation", "ablator")]

connection = krpc.connect("thermalFlux")
vessel = connection.space_center.active_vessel

heatShield = vessel.parts.with_tag("heatShield")[0]
print(heatShield)

vesselActive = True

try:
    while vesselActive:
        ut = str(connection.space_center.ut)
        alt = str(vessel.flight().mean_altitude)
        speed = str(vessel.flight().speed)
        conduction_flux = str(heatShield.thermal_conduction_flux)
        convection_flux = str(heatShield.thermal_convection_flux)
        radiation_flux = str(heatShield.thermal_radiation_flux)
        ablator = str(heatShield.resources.amount("Ablator"))

        outCSV.append((ut, alt, speed, conduction_flux, convection_flux, radiation_flux, ablator))
        time.sleep(0.1)
except:
    print("we crashed")


with open("C:\\krpc\\mapThermalFluxSteep.csv", "w") as fh:
    outString = '\n'.join([','.join(line) for line in outCSV])
    fh.write(outString)

print("we done")
