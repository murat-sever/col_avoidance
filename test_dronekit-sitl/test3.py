from dronekit import connect, VehicleMode , LocationGlobal, LocationGlobalRelative
from numpy import sign
import time
import math
from pymavlink import mavutil

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
        
def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only positions enabled)
        0, 0, 0,
        north, east, down, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
        
arm_and_takeoff(5)







#get detection.relative_position
#get detection.booleen

#booleen=1


#get servo.flag
#get GCS.zone
flag=-1
zone=1
Sx_min=-1
Sx_max=1
Sy_min=-1
Sy_max=1
h_net=2
h_camera=1


def get_location_metres(original_location, dNorth, dEast, dDown):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    dAlt = -dDown

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    newalt = original_location.alt + (dAlt * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon, newalt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon, newalt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    dalt = aLocation2.alt - aLocation1.alt
    return math.sqrt((dlat*dlat) + (dlong*dlong) + (dalt*dalt)) * 1.113195e5

i=0
start_time= time.time()   
while vehicle.mode.name=="GUIDED":
    phi = vehicle.heading
    print('phi=',phi)
    currentLocation = vehicle.location.global_relative_frame
    currentcoordinates= vehicle.location.local_frame
    (x,y,z)=(currentcoordinates.north,currentcoordinates.east,currentcoordinates.down)
    print(currentcoordinates)
    #t=vehicle.time
    #print(time.time()-start_time)
    #relative_position=(10-(time.time()-start_time),10-(time.time()-start_time)) #vehicle.velocity=1??
    relative_position=(-2,-2)
    xI= relative_position[0]
    yI= relative_position[1]
    print(xI,yI)


    booleen=1
   
	

    #get xI et yI
    #get booleen
    #get flag
    if booleen==1:
        print('detected')
        if flag==-1:
            l1=flag*sign(xI)*50
            l2=-flag*sign(yI)*50
            (n,e,d)=(-l1*math.sin(phi*math.pi/180),l1*math.cos(phi*math.pi/180),l2)
            goto_position_target_local_ned(n, e, d)
            #targetLocation = get_location_metres(currentLocation, n, e, d) #down???
            #targetDistance = get_distance_metres(currentLocation, targetLocation)
            #vehicle.simple_goto(targetLocation)
            remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
            print('colision av')           
            print("Distance to target: ", remainingDistance)
            if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
                print("Reached target")
                break;
            time.sleep(2)
    
        if flag==1 and zone==1 and (xI<Sx_min or xI>Sx_max) and (yI<Sy_min or yI>Sy_max):
            l1=flag*sign(xI)*10
            l2=-flag*sign(yI)
	    print('track')
        #    (n,e,d)=(x-l1*math.sin(phi*math.pi/180),y+l1*math.cos(phi*math.pi/180),z+l2+h_net+h_camera/2)
            (n,e,d)=(-l1*math.sin(phi*math.pi/180),l1*math.cos(phi*math.pi/180),l2)
	    print('n=',n)
	    print('e=',e)
	    print('d=',d)

           #send MAVLink msg
            goto_position_target_local_ned(n,e,d)
    
        #else
            # no msg  
    i=i+1  
            
print(x,y,z)        
