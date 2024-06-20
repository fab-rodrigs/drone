# https://github.com/Delta-High-Team/cbr-2024/blob/main/drone_108/src/face_tracking/face_tracking/face_tracking_no_node.py
# https://dronekit-python.readthedocs.io/en/latest/guide/index.html
# https://mavlink.io/en/getting_started/
# https://ardupilot.org/planner/docs/mission-planner-installation.html

import time
import math
import numpy as np
from pymavlink import mavutil

def drone_connection(option="sitl"):
    match option:
        case "mavproxy":
            connect_string = 'udp:0.0.0.0:14551'
        case "sitl":
            connect_string = 'tcp:127.0.0.1:5763'
        case "serial":
            connect_string = '/dev/ttyAMA0'
        case _:
            print("\nConexão inválida!")
            return None

    print("Connecting on: ", connect_string)
    drone = mavutil.mavlink_connection(connect_string, baud=57600, source_system=1, source_component=2)
    drone.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))
    drone.set_mode("GUIDED")
    
    return drone


def set_home_and_ekf_origin(vehicle=None, latitude=-27.593683637373115, longitude=-48.54154216232123, altitude=0, timestamp=1715363270):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 0, 0, 0, 0, latitude, longitude, altitude
    )
    vehicle.mav.set_gps_global_origin_send(
        vehicle.target_system,
        int(latitude*1e7),
        int(longitude*1e7),
        int(altitude*1000),
        int(timestamp)
    )
    msg = receive_all_messages(vehicle, 'GLOBAL_POSITION_INT')
    if msg:
        altitude_offset = msg.alt / 1000.0
        if altitude_offset >= 0.05 or altitude_offset < 0.00:
            vehicle.mav.command_long_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0, 0, 0, 0, 0, latitude, longitude, altitude_offset
            )
            vehicle.mav.set_gps_global_origin_send(
                vehicle.target_system,
                int(latitude*1e7),
                int(longitude*1e7),
                int(altitude_offset*1000),
                int(timestamp)
            )
        else:
            print("Altitude já correta.")
        print("Altitude corrigida")


def send_position_target_local_ned(target_ned, vehicle=None, yaw_angle=0.0):
    msg = receive_all_messages(vehicle, 'ATTITUDE')
    msg_t = receive_all_messages(vehicle, 'LOCAL_POSITION_NED')

    x = target_ned[0] + msg_t.x
    y = target_ned[1] + msg_t.y
    z = -target_ned[2] + msg_t.z
    yaw_angle = msg.yaw + math.radians(yaw_angle)

    msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b101111111000 & 0xFF,
        x, y, z, 0, 0, 0, 0, 0, 0, yaw_angle, 0
    )

    vehicle.mav.send(msg)


def ned_to_local(ned_coordinates, drone_heading):
    rotation_matrix = np.array([
        [np.cos(drone_heading), -np.sin(drone_heading), 0],
        [np.sin(drone_heading), np.cos(drone_heading), 0],
        [0, 0, 1]
    ])
    return np.dot(rotation_matrix, ned_coordinates)


def receive_all_messages(vehicle=None, type_message='None'):
    vehicle.mav.request_data_stream_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    return vehicle.recv_match(type=type_message, blocking=True, timeout=5)


def adjust_drone_position(offset_x, offset_y, offset_z, vehicle=None):
    global adjust_x, adjust_yaw, adjust_z, correction

    if not 5000 <= offset_z <= 15000 and offset_z != 0:
        correction = True
        if offset_z < 5000:
            adjust_x = (-0.0001 * offset_z) + 0.6
            if adjust_x > 0.5:
                adjust_x = 0.5
            print(f"Indo para frente! {adjust_x}")

        elif offset_z > 15000:
            adjust_x = -1 * ((3E-05 * offset_z) - 0.35)
            if adjust_x < -0.5:
                adjust_x = -0.5
            print(f"Indo para trás! {adjust_x}")

    if correction:
        send_play_tone(vehicle)
        time.sleep(0.01 * adjust_x)
        now = receive_all_messages(vehicle, 'ATTITUDE')
        target_position = (adjust_x, 0, adjust_z)
        target_coord = ned_to_local(target_position, now.yaw)
        send_position_target_local_ned(target_coord, yaw_angle=adjust_yaw, vehicle=vehicle)
        correction = False
        adjust_x = 0.0
        adjust_yaw = 0
        adjust_z = 0.0


def send_play_tone(vehicle):
    tone_sequence = "EFG"
    vehicle.mav.play_tune_send(vehicle.target_system, vehicle.target_component, tone_sequence.encode())


def arm_and_takeoff(vehicle, target_altitude):
    vehicle.arducopter_arm()
    print("Armando..")
    vehicle.motors_armed_wait()
    print("Drone armado")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude
    )

    print("Iniciando o aguarde da mensagem de posição")
    while True:
        msg = receive_all_messages(vehicle, 'GLOBAL_POSITION_INT')
        if msg:
            altitude_local = msg.relative_alt / 1000.0
            print("Altitude: ", altitude_local)
            if altitude_local >= target_altitude * 0.98:
                time.sleep(0.5)
                break
    print("Altitude atingida")


def main(drone):
    try:
        while True:
            # Aqui você pode ajustar as coordenadas de destino conforme necessário
            target_ned = [5, 0, 0]  # Exemplo: mover 5 metros para a frente
            send_position_target_local_ned(target_ned, drone, yaw_angle=0)

            #latitude = 0
            #longitude = 0
            #altitude = 0
            #send_position_target_global_int(latitude, longitude, altitude, drone)
            
            time.sleep(1)
    except KeyboardInterrupt:
        pass

drone = drone_connection()
main(drone)
