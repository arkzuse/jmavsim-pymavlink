from pymavlink import mavutil
from utils.connect_to_sysid import *
from utils.wait_for_position_aiding import *
from utils.get_autopilot_info import *

conn = mavutil.mavlink_connection('udpin:localhost:14540')

conn.wait_heartbeat()
print(conn.target_system, conn.target_component)

def arm():
    conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
    conn.motors_armed_wait()


def disarm():
    conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
    conn.motors_disarmed_wait()

def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id=1):

    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    wait_until_position_aiding(mav_connection)

    autopilot_info = get_autopilot_info(mav_connection, tgt_sys_id)

    if autopilot_info["autopilot"] == "ardupilotmega":
        print("Connected to ArduPilot autopilot")
        mode_id = mav_connection.mode_mapping()["GUIDED"]
        takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

    elif autopilot_info["autopilot"] == "px4":
        print("Connected to PX4 autopilot")
        print(mav_connection.mode_mapping())
        mode_id = mav_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]

    else:
        raise ValueError("Autopilot not supported")


    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    # Command Takeoff
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")

    return takeoff_msg.result

def land(the_connection: mavutil.mavlink_connection, timeout: int = 10) -> int:
    """
    Sends a command for the drone to land.

    Args:
        the_connection (mavutil.mavlink_connection): The MAVLink connection to use.
        timeout (int): Time in seconds to wait for an acknowledgment.

    Returns:
        int: mavutil.mavlink.MAV_RESULT enum value.
    """

    # Send a command to land
    the_connection.mav.command_long_send(
        the_connection.target_system, 
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 
        0, 0, 0, 0, 0, 0, 0, 0
    )

    # Wait for the acknowledgment
    ack = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if ack is None:
        print('No acknowledgment received within the timeout period.')
        return None

    return ack.result


def get_global_pos(the_connection):
    return the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)


arm()
print(takeoff(conn, 25), 't')

while 1:
    if get_global_pos(conn).relative_alt == 25000:
        break

time.sleep(15)

land(conn)
disarm()
