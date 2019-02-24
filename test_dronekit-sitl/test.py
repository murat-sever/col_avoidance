from dronekit import connect, VehicleMode #, LocationLocal
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
#vehicle=connect('tcp:127.0.0.1:5760',wait_ready=True)

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
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
        
arm_and_takeoff(5)

phi = vehicle.heading
print(phi)
#loc_frame= (1,2,3) #NED frame

loc_frame= vehicle.location.local_frame

print(vehicle.location.local_frame)

(x,y,z)=(loc_frame.north,loc_frame.east,loc_frame.down)

#get detection.relative_position
#get detection.booleen
relative_position=(2,2)
booleen=1


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

xI= relative_position[0]
yI= relative_position[1]

if booleen==1:
    if flag==-1:
        l1=flag*sign(xI)
        l2=-flag*sign(yI)
        (x,y,z)=(x-l1*math.sin(phi*math.pi/180),y+l1*math.cos(phi*math.pi/180),z+l2)
        goto_position_target_local_ned(x, y, z)
    
    elif flag==1 and zone==1 and (xI<Sx_min or xI>Sx_max) and (yI<Sy_min or yI>Sy_max):
        l1=flag*sign(xI)
        l2=-flag*sign(yI)
        (x,y,z)=(x-l1*math.sin(phi*math.pi/180),y+l1*math.cos(phi*math.pi/180),z+l2+h_net+h_camera/2)
        #send MAVLink msg
        goto_position_target_local_ned(x, y, z)
        
    #else
        # no msg    
print(x,y,z)        
