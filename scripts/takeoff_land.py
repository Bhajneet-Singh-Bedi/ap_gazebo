from pymavlink import mavutil
import sys
import time


def changeMode(master, mode):
    # Check if mode is available
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)
    mode_id = master.mode_mapping()[mode]

    master.mav.command_long_send(
       master.target_system, master.target_component,
       mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
       0, mode_id, 0, 0, 0, 0, 0)
    master.set_mode(mode_id)
    print(f'Mode change command sent to {mode}, waiting for confirmation...')
    while not master.wait_heartbeat().custom_mode == mode_id:
        time.sleep(1)
    print(f'Mode changed to {mode}')

def armDisarm(master, value):
    master.mav.command_long_send(   
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,value, 0, 0, 0, 0, 0, 0)
    print("Waiting for the vehicle to arm" if value == 1 else "Disarm command sent")
    if value == 1:
        master.motors_armed_wait()
        print("Armed")
    else:
        print("Disarmed")


def takeoff(master, target_altitude):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, target_altitude
    )
    while True:
        alt = get_current_position(master)
        # print(alt)
        if alt<584.18+target_altitude-0.5:
            continue
        else:
            print("Takeoff target reached")
            break
    

def set_position(master, height):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, 
        master.target_system,
        master.target_component,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        3576,
        0,0,-height,0,0,0,0,0,0,0,0))
    while True:
        alt = get_current_position(master)
        if alt<584.18+height-0.5:
            continue
        else:
            print("Desired position reached")
            break

def request_message(master,message_id):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # Confirmation
        message_id,  # Message ID (e.g., 33 for GLOBAL_POSITION_INT)
        0, 0, 0, 0, 0, 0  # Unused parameters
    )

def get_current_position(master):
    request_message(master,mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
        if msg:
            latitude = msg.lat / 1e7
            longitude = msg.lon / 1e7
            altitude = msg.alt / 1000.0  # Altitude in meters
            return altitude
        else:
            print('No message received, retrying...')

def main():
    master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
    

    # Change mode to guided
    changeMode(master, "GUIDED")

    #For arming give 1 as param.
    armDisarm(master, 1)

    #takeoff
    takeoff(master,2)

    # Takeoff upto certain height
    set_position(master, 10)

    # Comeback
    changeMode(master, "RTL")


if __name__=="__main__":
    main()
