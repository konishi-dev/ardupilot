from dronekit import connect,VehicleMode,LocationGlobalRelative
import math,sys,time

#
# Global valuables
#
sin18deg = math.sin(math.radians(18))
cos18deg = math.cos(math.radians(18))
sin54deg = math.sin(math.radians(54))
cos54deg = math.cos(math.radians(54))

radiusEquator = 6378137.0000
wgs84_e2 = 0.00669437999019758

defaultSideLen = 50     # m

#
# Functions
#

# Connect to SITL
def connect2Sitl():
    print("Connecting to SITL...")
    v = connect("tcp:127.0.0.1:5762",wait_ready=True)
    print("Connected !!\n")
    return v

def isArmed(v):
    return v.armed

# Arming
def arming(v):
    sys.stdout.write("Waiting for initialize.")
    while not v.is_armable:
        time.sleep(0.2)
        sys.stdout.write(".")
    print "\nDone !!!\n"

    v.mode = VehicleMode("GUIDED")
    v.armed = True

    sys.stdout.write("Waiting for vehicle to be armed.")
    while not isArmed(vehicle):
        time.sleep(0.2)
        sys.stdout.write(".")

    print "\nArmed !!!\n"

# Load the home location
def setHomeLocation(v):
    sys.stdout.write("Setting Home location.")
    while not v.home_location:
        cmds = v.commands
        cmds.download()
        cmds.wait_ready()
        time.sleep(0.2)
        sys.stdout.write(".")

    print("\nHome location: %s" %v.home_location)

# Calc dist by latitude/longitude
def calcDistByLocation(loc1, loc2):
    #print("Loc1: %s" %loc1)
    #print("Loc2: %s" %loc2)
    diffLat = (loc1.lat - loc2.lat)*(math.pi/180)*radiusEquator*(1-wgs84_e2)/(1-wgs84_e2*(math.sin(math.radians((loc1.lat+loc2.lat)/2)))**2)**(1.5)
    diffLon = (loc1.lon - loc2.lon)*(math.pi/180)*radiusEquator*math.cos(math.radians((loc1.lat+loc2.lat)/2))/math.sqrt(1-wgs84_e2*math.sin(math.radians((loc1.lat+loc2.lat)/2))**2)

    diff = math.sqrt(diffLat ** 2 + diffLon ** 2)
    #print "diff =",diff
    return diff

def calcScale(startLoc, distance):
    unitLoc = LocationGlobalRelative(
            startLoc.lat - 1e-7 * (1 + sin54deg),
            startLoc.lon - 1e-7 * cos54deg,
            startLoc.alt)
    return distance * 1e-7 / calcDistByLocation(startLoc, unitLoc)

def setupWayPoints(v):
    start = v.location.global_relative_frame
    scale = calcScale(start, 50)
    print "scale =", scale
    wp1 = LocationGlobalRelative(
            start.lat - scale * (1 + sin54deg),
            start.lon - scale * cos54deg,
            start.alt - 10)
    wp2 = LocationGlobalRelative(
            start.lat - scale * (1 - sin18deg),
            start.lon + scale * cos18deg,
            start.alt - 5)
    wp3 = LocationGlobalRelative(
            start.lat - scale * (1 - sin18deg),
            start.lon - scale * cos18deg,
            start.alt - 5)
    wp4 = LocationGlobalRelative(
            start.lat - scale * (1 + sin54deg),
            start.lon + scale * cos54deg,
            start.alt - 10)
    wp5 = start

    return (wp1, wp2, wp3, wp4, wp5)

def reached(v, t):
    if calcDistByLocation(v.location.global_relative_frame,t) < 0.6:
        return True

    return False

#
# Start main from here
#
vehicle = connect2Sitl()

print("Flight mode:" + vehicle.mode.name + ", Armed:" +
                ("Armed" if isArmed(vehicle) else "Disarmed"))

setHomeLocation(vehicle)

startLoc = vehicle.home_location
needsTakeOff = (startLoc.alt < 20)
startLoc.alt = 20 if needsTakeOff else startLoc.alt
print("\nStart location: %s" %startLoc)

arming(vehicle)

if needsTakeOff:
    print("Take off !!!")
    vehicle.simple_takeoff(startLoc.alt)

    while True:
        if vehicle.location.global_relative_frame.alt >= startLoc.alt * 0.95:
            print("Reached Starting Location!!")
            print(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt)
            break

wayPoints = setupWayPoints(vehicle)

time.sleep(3)
print("Start !!!")

# Star-shaped flight
i = 0
for wp in wayPoints:
    print("Next location: %s" %wp)

    vehicle.simple_goto(wp, groundspeed=10) 
    while True:
        if reached(vehicle, wp):
            print("Reached target Location")
            break

    print(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt)
    i += 1
    print "target#",i,"finish"
    time.sleep(5)

time.sleep(5)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

while isArmed(vehicle):
    time.sleep(0.5)
    sys.stdout.write(".")

print "\nDisarmed !!!\n"

time.sleep(1)

print("Close vehicle object")
vehicle.close()
