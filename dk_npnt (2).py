from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
from datetime import datetime
import math
import shapely
import xml.etree.ElementTree as ET
from shapely.geometry import Point, Polygon

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=115200, wait_ready=True)

home = vehicle.home_location
if home is None:
  print("[TAKING OTHER HOME]")
  home = vehicle.location.global_frame

def xml_parses(filename):
  tree = ET.parse(filename)
  root = tree.getroot()
  flightParams = [elem.attrib for elem in root.iter('FlightParameters')]
  geoFence = [elem.attrib for elem in root.iter('Coordinate')]
  return flightParams, geoFence

def altitude_check(maxAlt):
  print("[ALT] " +str(vehicle.location.global_relative_frame.alt))
  if vehicle.location.global_relative_frame.alt > maxAlt:
    return True
  else:
    return False

def velocity_check(maxVel):
  print("[VEL] "+str(vehicle.groundspeed))
  if (vehicle.groundspeed > maxVel) :
    return True
  else:
    return False

def LOS(LOSDist):
  loc = vehicle.location.global_frame
  global home
  print(loc)
  print("HOME"+str(home))
  if home is not None:  
    dlat = loc.lat - home.lat
    dlong = loc.lon - home.lon
    dist = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    if dist >= LOSDist:
      return True
    else:
      return False

# Function to arm and then check parameters continuosly
def arm_and_check(altitude_limit=10, polygon=None):

  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  # Check that vehicle has reached takeoff altitude
  while True:

    copterLoc = vehicle.location.global_frame
    print("[LOCATION]"+str(copterLoc))
    copterPoint = Point(copterLoc.lat, copterLoc.lon)
    #perform checks      
    a = altitude_check(altitude_limit)
    v = velocity_check(10) 
    l = LOS(5)
    g = copterPoint.within(polygon)

    if not g:
      vehicle.mode = VehicleMode("RTL")
      print("[MODE RTL] GEOFENCE VIOLATED")

    if a:
      vehicle.mode = VehicleMode("RTL")
      print("[MODE RTL] ACTIVATED ALTITUDE EXCEEDED")

    if v:
      vehicle.mode = VehicleMode("RTL")
      print("[MODE RTL] ACTIVATED VELOCITY EXCEEDED")

    if l:
      vehicle.mode = VehicleMode("RTL")
      print("[MODE RTL] ACTIVATED LOS DISTANCE EXCEEDED")
    time.sleep(1)

flightParams, geoFence = xml_parses("pa.xml")

#TIME CHECK
startTime = flightParams[0]['flightStartTime']
endTime = flightParams[0]['flightEndTime']

now = datetime.now()
dt_string = now.strftime("%Y-%m-%d"+"T"+"%H:%M:%S")

if not (dt_string > startTime and dt_string < endTime):
  print("[UNARMING] Timing not in permitted range")
  time_flag = False
else:
  time_flag = True

#Altitude
alt = float(flightParams[0]['maxAltitude'])

pts = []            
for i in range(len(geoFence)):
  lat = float(geoFence[i]['latitude'])
  lon = float(geoFence[i]['longitude'])
  pts.append((lat,lon))

polygon = Polygon(pts)

if time_flag:
  arm_and_check(altitude_limit=alt, polygon = polygon)

print("RTL complete")

#vehicle.close()
