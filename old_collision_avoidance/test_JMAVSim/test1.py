from dronekit import connect, Command, VehicleMode , LocationGlobal, LocationGlobalRelative, LocationLocal
from numpy import sign
from pymavlink import mavutil
import time, sys, argparse, math


################################################################################################
# Settings
################################################################################################

connection_string       = '127.0.0.1:14540'
MAV_MODE_AUTO   = 4
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py


# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Init
################################################################################################

# Connect to the Vehicle
print "Connecting"
vehicle = connect(connection_string, wait_ready=True)

def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready

        
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
    return LocationLocal(n,e,d)
        

home_position_set = False

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True






#get detection.relative_position
#get detection.booleen

#booleen=1


#get servo.flag
#get GCS.zone
flag=1
zone=1
Sx_min=-2
Sx_max=2
Sy_min=-2
Sy_max=2
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

def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)





# wait for a home position lock
while not home_position_set:
    print "Waiting for home position..."
    time.sleep(1)

arm_and_takeoff(2)

# Display basic vehicle state
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt

# Change to AUTO mode
#PX4setMode(MAV_MODE_AUTO)
#time.sleep(1)
# Load commands
cmds = vehicle.commands
#cmds.clear()

home = vehicle.location.global_relative_frame


# takeoff to 10 meters
#wp = get_location_offset_meters(home, 0, 0, 10);
#cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
#cmds.add(cmd)


#SET_POSITION_TARGET_LOCAL_NED





start_time= time.time() 
while(1==1):  

    phi = vehicle.heading
    #phi=5
    print('phi=',phi)
    currentLocation = vehicle.location.global_relative_frame
    currentcoordinates= vehicle.location.local_frame
    (x,y,z)=(currentcoordinates.north,currentcoordinates.east,currentcoordinates.down)
    print(currentcoordinates)
    #t=vehicle.time
    print("time=",time.time()-start_time)
    if time.time()-start_time<100:
	relative_position=(4-(time.time()-start_time)*0.3,4-(time.time()-start_time)*0.3) #vehicle.velocity=1??
    else:
	relative_position=(0,0)
    
#relative_position=(-2,-2)
    xI= relative_position[0]
    yI= relative_position[1]
    print(xI,yI)


    booleen=1
   
	

    #get xI et yI
    #get booleen
    #get flag
    if booleen==1:
        print('a drone is detected')
        if flag==-1:
            l1=flag*sign(xI)*10
            l2=-flag*sign(yI)
	    print('col_avoid')
            (n,e,d)=(-l1*math.sin(phi*math.pi/180),l1*math.cos(phi*math.pi/180),l2)
	    print('n=',n)
	    print('e=',e)
	    print('d=',d)
            
        if flag==1 and zone==1 and (xI<Sx_min or xI>Sx_max) and (yI<Sy_min or yI>Sy_max):
            l1=flag*sign(xI)*10
            l2=-flag*sign(yI)
	    print('track')
        #    (n,e,d)=(x-l1*math.sin(phi*math.pi/180),y+l1*math.cos(phi*math.pi/180),z+l2+h_net+h_camera/2)
            (n,e,d)=(-l1*math.sin(phi*math.pi/180),l1*math.cos(phi*math.pi/180),l2)
	    print('n=',n)
	    print('e=',e)
	    print('d=',d)
	if flag==1 and zone==1 and (xI>Sx_min and xI<Sx_max) :
	    l2=-flag*sign(yI)
	    print('track and threshold x')
	    (n,e,d)=(0,0,l2)
	    print('n=',n)
	    print('e=',e)
	    print('d=',d)

	if flag==1 and zone==1 and (yI>Sy_min and yI<Sy_max):
	    l1=flag*sign(xI)*10
	    print('track and threshold y')
	    (n,e,d)=(-l1*math.sin(phi*math.pi/180),l1*math.cos(phi*math.pi/180),0)
	    print('n=',n)
	    print('e=',e)
	    print('d=',d)

        # send MAVLink msg
        goto_position_target_local_ned(n,e,d)
    
	print("Mode:",vehicle.mode.name)
        
