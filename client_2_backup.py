#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

import threading
import socketio  # type: ignore

connection_string = "/dev/ttyACM0"

isRunWp = False
isFinished = False
waypoints = None
waypoint_index = 1
waypoint_index_recv = 2

flag_change_dis = False

DELAY = .6
time_prev = 0

def distance_to_current_waypoint(vehicle, waypoint):
    """
    Returns the ground distance in meters to the waypoint.
    """
    distance = get_distance_metres([vehicle.location.global_frame.lat, vehicle.location.global_frame.lon], [
                                   waypoint["lat"], waypoint["lng"]])
    return distance

@staticmethod
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metrtes between two LocationGlobalRelative objects.
    """

    R = 6378.0

    lat1 = aLocation1[0]
    lat2 = aLocation2[0]

    lon1 = aLocation1[1]
    lon2 = aLocation2[1]

    # Convert latitude and longitude from degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # Haversine formula
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * \
        math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in kilometers
    distance = R * c

    distance_m = distance * 1000

    return distance_m


def goto_waypoint(vehicle, waypoint):
    """
    Commands the vehicle to go to a specified waypoint.
    """
    point = LocationGlobalRelative(
        waypoint["lat"], waypoint["lng"], waypoint["alt"])
    vehicle.simple_goto(point, groundspeed=waypoint["speed"])


print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)

vehicle.wait_ready('autopilot_version')

# Initialize the Socket.IO client with reconnection options
sio = socketio.Client(
    reconnection=True, reconnection_attempts=20, reconnection_delay=1)

# Event to signal when the client is connected
connected_event = threading.Event()


def emit_frame(stream, direction, header, data, status=None):
    if sio.connected:
        sio.emit(stream, {
            "direction": direction,
            "header": header,
            "data": data,
            "status": status
        })
    else:
        print("Socket.IO client is not connected.")


@sio.event
def connect():
    print('Connected to server')
    connected_event.set()  # Signal that the client is connected


@sio.event
def disconnect():
    print('Disconnected from server')
    connected_event.clear()  # Clear the connected event


@sio.event
def reconnect():
    print('Reconnected to server')
    connected_event.set()  # Signal that the client is reconnected


@sio.event
def message(data):
    print('Message received:', data)


@sio.event
def drone(payload):
    global waypoints, isRunWp
    print(f"Drone event received: {payload} - mode of drone: {str(vehicle.mode.name)}")
    if str(vehicle.mode.name) == "GUIDED":
        if payload["header"] == "arm":
            if payload["data"] is True:
                vehicle.armed = True
                while not vehicle.armed:
                    time.sleep(.1)
                emit_frame("controlMsg", "web", "droneStatus",
                           "drone is armming...")
            elif payload["data"] is False:
                vehicle.armed = False
                emit_frame("controlMsg", "web", "droneStatus",
                           "drone is disarmming...")
        if payload["header"] == "takeoff":
            if payload["data"]["ctrl"] is True:
                vehicle.simple_takeoff(int(payload["data"]["alt"]))
        if payload["header"] == "land":
            if payload["data"] == True:
                print("Landing...")
                vehicle.mode = VehicleMode("LAND")
                while vehicle.armed:
                    print("Waiting for disarming...")
                    emit_frame("controlMsg", "web", "droneStatus",
                               "Waiting for disarming...")
                    time.sleep(1)
                vehicle.mode = VehicleMode("GUIDED")
                print("Landed and disarmed")
                emit_frame("controlMsg", "web", "droneStatus",
                           "Landed and disarmed")

        if payload["header"] == "set_home_gps":
            print(f"set home gps: {payload['data']}")
            my_location_alt = vehicle.location.global_frame
            my_location_alt.lat = payload['data']['lat']
            my_location_alt.lon = payload['data']['lon']
            my_location_alt.alt = payload['data']['alt']
            vehicle.home_location = my_location_alt
        if payload["header"] == "run_wp":
            point = LocationGlobalRelative(
                payload['data']["lat"], payload['data']["lng"], payload['data']["alt"])
            vehicle.simple_goto(point, groundspeed=payload['data']["speed"])
            print(f"run drone to: {payload['data']}")
        if payload["header"] == "run_all_wp":
            waypoints = payload['data']
            print(f"run all wp: {payload['data']}")
            isRunWp = True
            print(f"state of isrunwp: {isRunWp}")
        if payload["header"] == "start_all_wp":
            print(f"start all wp: {payload['data']}")
            isRunWp = payload['data']
        if payload["header"] == "command":
            print(f"command is: {payload['data']}")
        if payload["header"] == "returnToHome":
            print(f"returnToHome is: {payload['data']}")
            if payload['data'] == True:
                vehicle.mode = VehicleMode("RTL")
            else:
                vehicle.mode = VehicleMode("GUIDED")
        if payload["header"] == "runCoffee":
            print(f"runCoffee is: {payload['data']}")
#    else:
#        emit_frame("controlMsg", "web", "droneStatus", "change mode GUIDED in tx, please!!!")


def send_data_loop():
    global waypoint_index, waypoints, waypoint_index_recv, isFinished, time_prev, DELAY, vehicle, isRunWp, flag_change_dis
    # Wait for the client to connect before starting the data loop
    connected_event.wait()
    while True:
        time_curr = time.time()

        if time_curr - time_prev >= DELAY:
            time_prev = time_curr
            payload_infor_drone = {
                "firmware": str(vehicle.version),
                "type": str(vehicle.version.release_type()),
                "mode": str(vehicle.mode.name),
                "battery": {
                    "voltage": str(vehicle.battery.voltage),
                    "current": str(vehicle.battery.current),
                    "level": str(vehicle.battery.level)
                },
                "heartbeat": str(vehicle.last_heartbeat),
                "armed": vehicle.armed,
                "flagDisiableArmed": str(vehicle.is_armable),
                "status": str(vehicle.system_status.state)
            }
            emit_frame("controlMsg", "web",
                       "droneStatusInfor", payload_infor_drone)
            payload_infor_atitude = {
                "atitude": {
                    "pitch": float(format(vehicle.attitude.pitch, ".2f")),
                    "roll": float(format(vehicle.attitude.roll, ".2f")),
                    "yaw": float(format(vehicle.attitude.yaw, ".2f")),
                },
                "velocity": {
                    "vx": vehicle.velocity[0],
                    "vy": vehicle.velocity[1],
                    "vz": vehicle.velocity[2],
                },
                "heading": vehicle.heading,
            }
            emit_frame("controlMsg", "web", "droneStatusAtitude",
                       payload_infor_atitude)
            payload_infor_gps = {
                "sat": vehicle.gps_0.satellites_visible,
                "fix": vehicle.gps_0.fix_type,
                "Groundspeed": vehicle.groundspeed,
                "lat": vehicle.location.global_frame.lat,
                "lon": vehicle.location.global_frame.lon,
                "alt": vehicle.location.global_frame.alt,
                "homeLocation": {
                    "lat": vehicle.home_location.lat if vehicle.home_location is not None else 0,
                    "lon": vehicle.home_location.lon if vehicle.home_location is not None else 0,
                    "alt": vehicle.home_location.alt if vehicle.home_location is not None else 0,
                }
            }
            emit_frame("controlMsg", "web",
                       "droneStatusGps", payload_infor_gps)

            if waypoints is not None and str(vehicle.mode.name) == "GUIDED":
                if isRunWp is True:
                    if waypoint_index < len(waypoints):
                        current_waypoint = waypoints[waypoint_index]
                        if waypoint_index_recv != current_waypoint:
                            flag_change_dis = False
                            waypoint_index_recv = current_waypoint
                            goto_waypoint(vehicle, current_waypoint)
                        else:
                            distance = distance_to_current_waypoint(
                                vehicle, current_waypoint)
#                        emit_frame("controlMsg", "web", "droneStatus", f"distance to next point {waypoint_index} is: {distance}m")
                            print(f"Distance to waypoint: {distance:.2f} meters, waypoint: {waypoint_index}")
                            # Check if the vehicle has reached the waypoint
                            if distance < 1:  # You can adjust this threshold
                                if flag_change_dis is False:
                                    flag_change_dis = True
                                    print("Waypoint reached!")
                                    waypoint_index = waypoint_index + 1
#                            time.sleep(1)
                    else:
                        isFinished = True
                        isRunWp = False
                        waypoint_index = 0
#                    emit_frame("controlMsg", "web", "droneStatus", "drone is finshed...")
#            else:
#                emit_frame("controlMsg", "web", "droneStatus", "drone no start...")
#        else:
#            emit_frame("controlMsg", "web", "droneStatus", "drone no load waypoint or no change mode GUIDED,...")


# Start the data sending loop in a separate thread
data_thread = threading.Thread(target=send_data_loop)
data_thread.daemon = True
data_thread.start()

# Connect to the server (replace 'http://localhost:5000' with your server URL)
sio.connect('http://103.167.198.50:5000')

# Wait for events
sio.wait()
