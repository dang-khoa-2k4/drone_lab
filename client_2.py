#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading
import socketio  # type: ignore

class DroneController:
    def __init__(self, connection_string, server_url):
        self.connection_string = connection_string
        self.server_url = server_url
        self.vehicle = None
        self.sio = socketio.Client(reconnection=True, reconnection_attempts=20, reconnection_delay=1)
        self.connected_event = threading.Event()
        self.isRunWp = False
        self.isFinished = False
        self.waypoints = None
        self.waypoint_index = 1
        self.waypoint_index_recv = 2
        self.flag_change_dis = False
        self.DELAY = .6
        self.time_prev = 0

        # Connect to vehicle and server
        self.connect_vehicle()
        self.setup_socketio()

        # Start the data sending loop in a separate thread
        self.data_thread = threading.Thread(target=self.send_data_loop)
        self.data_thread.daemon = True
        self.data_thread.start()

        # Connect to the server
        self.sio.connect(self.server_url)
        self.sio.wait()

    def connect_vehicle(self):
        print(f"\nConnecting to vehicle on: {self.connection_string}")
        self.vehicle = connect(self.connection_string, wait_ready=True)
        self.vehicle.wait_ready('autopilot_version')

    def setup_socketio(self):
        @self.sio.event
        def connect():
            print('Connected to server')
            self.connected_event.set()  # Signal that the client is connected

        @self.sio.event
        def disconnect():
            print('Disconnected from server')
            self.connected_event.clear()  # Clear the connected event

        @self.sio.event
        def reconnect():
            print('Reconnected to server')
            self.connected_event.set()  # Signal that the client is reconnected

        @self.sio.event
        def message(data):
            print('Message received:', data)

        @self.sio.event
        def drone(payload):
            self.handle_drone_event(payload)

    def emit_frame(self, stream, direction, header, data, status=None):
        if self.sio.connected:
            self.sio.emit(stream, {
                "direction": direction,
                "header": header,
                "data": data,
                "status": status
            })
        else:
            print("Socket.IO client is not connected.")

    def distance_to_current_waypoint(self, waypoint):
        """
        Returns the ground distance in meters to the waypoint.
        """
        distance = self.get_distance_metres([self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon], [
                                           waypoint["lat"], waypoint["lng"]])
        return distance

    @staticmethod
    def get_distance_metres(aLocation1, aLocation2):
        """
        Returns the ground distance in meters between two locations.
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

    def goto_waypoint(self, waypoint):
        """
        Commands the vehicle to go to a specified waypoint.
        """
        point = LocationGlobalRelative(
            waypoint["lat"], waypoint["lng"], waypoint["alt"])
        self.vehicle.simple_goto(point, groundspeed=waypoint["speed"])

    def handle_drone_event(self, payload):
        """
        Handles drone-related events from the Socket.IO server.
        """
        print(f"Drone event received: {payload} - mode of drone: {str(self.vehicle.mode.name)}")
        if str(self.vehicle.mode.name) == "GUIDED":
            if payload["header"] == "arm":
                if payload["data"] is True:
                    self.vehicle.armed = True
                    while not self.vehicle.armed:
                        time.sleep(.1)
                    self.emit_frame("controlMsg", "web", "droneStatus", "drone is arming...")
                elif payload["data"] is False:
                    self.vehicle.armed = False
                    self.emit_frame("controlMsg", "web", "droneStatus", "drone is disarming...")
            if payload["header"] == "takeoff":
                if payload["data"]["ctrl"] is True:
                    self.vehicle.simple_takeoff(int(payload["data"]["alt"]))
            if payload["header"] == "land":
                if payload["data"] == True:
                    print("Landing...")
                    self.vehicle.mode = VehicleMode("LAND")
                    while self.vehicle.armed:
                        print("Waiting for disarming...")
                        self.emit_frame("controlMsg", "web", "droneStatus", "Waiting for disarming...")
                        time.sleep(1)
                    self.vehicle.mode = VehicleMode("GUIDED")
                    print("Landed and disarmed")
                    self.emit_frame("controlMsg", "web", "droneStatus", "Landed and disarmed")

            if payload["header"] == "set_home_gps":
                print(f"set home gps: {payload['data']}")
                my_location_alt = self.vehicle.location.global_frame
                my_location_alt.lat = payload['data']['lat']
                my_location_alt.lon = payload['data']['lon']
                my_location_alt.alt = payload['data']['alt']
                self.vehicle.home_location = my_location_alt
            if payload["header"] == "run_wp":
                point = LocationGlobalRelative(
                    payload['data']["lat"], payload['data']["lng"], payload['data']["alt"])
                self.vehicle.simple_goto(point, groundspeed=payload['data']["speed"])
                print(f"run drone to: {payload['data']}")
            if payload["header"] == "run_all_wp":
                self.waypoints = payload['data']
                print(f"run all wp: {payload['data']}")
                self.isRunWp = True
                print(f"state of isrunwp: {self.isRunWp}")
            if payload["header"] == "start_all_wp":
                print(f"start all wp: {payload['data']}")
                self.isRunWp = payload['data']
            if payload["header"] == "command":
                print(f"command is: {payload['data']}")
            if payload["header"] == "returnToHome":
                print(f"returnToHome is: {payload['data']}")
                if payload['data'] == True:
                    self.vehicle.mode = VehicleMode("RTL")
                else:
                    self.vehicle.mode = VehicleMode("GUIDED")
            if payload["header"] == "runCoffee":
                print(f"runCoffee is: {payload['data']}")

    def send_data_loop(self):
        """
        Sends vehicle data to the server at regular intervals.
        """
        # Wait for the client to connect before starting the data loop
        self.connected_event.wait()
        while True:
            time_curr = time.time()

            if time_curr - self.time_prev >= self.DELAY:
                self.time_prev = time_curr
                payload_infor_drone = {
                    "firmware": str(self.vehicle.version),
                    "type": str(self.vehicle.version.release_type()),
                    "mode": str(self.vehicle.mode.name),
                    "battery": {
                        "voltage": str(self.vehicle.battery.voltage),
                        "current": str(self.vehicle.battery.current),
                        "level": str(self.vehicle.battery.level)
                    },
                    "heartbeat": str(self.vehicle.last_heartbeat),
                    "armed": self.vehicle.armed,
                    "flagDisiableArmed": str(self.vehicle.is_armable),
                    "status": str(self.vehicle.system_status.state)
                }
                self.emit_frame("controlMsg", "web", "droneStatusInfor", payload_infor_drone)
                payload_infor_atitude = {
                    "atitude": {
                        "pitch": float(format(self.vehicle.attitude.pitch, ".2f")),
                        "roll": float(format(self.vehicle.attitude.roll, ".2f")),
                        "yaw": float(format(self.vehicle.attitude.yaw, ".2f")),
                    },
                    "velocity": {
                        "vx": self.vehicle.velocity[0],
                        "vy": self.vehicle.velocity[1],
                        "vz": self.vehicle.velocity[2],
                    },
                    "heading": self.vehicle.heading,
                }
                self.emit_frame("controlMsg", "web", "droneStatusAtitude", payload_infor_atitude)
                payload_infor_gps = {
                    "sat": self.vehicle.gps_0.satellites_visible,
                    "fix": self.vehicle.gps_0.fix_type,
                    "Groundspeed": self.vehicle.groundspeed,
                    "lat": self.vehicle.location.global_frame.lat,
                    "lon": self.vehicle.location.global_frame.lon,
                    "alt": self.vehicle.location.global_frame.alt,
                    "homeLocation": {
                        "lat": self.vehicle.home_location.lat if self.vehicle.home_location is not None else 0,
                        "lon": self.vehicle.home_location.lon if self.vehicle.home_location is not None else 0,
                        "alt": self.vehicle.home_location.alt if self.vehicle.home_location is not None else 0,
                    }
                }
                self.emit_frame("controlMsg", "web", "droneStatusGps", payload_infor_gps)

                if self.waypoints is not None and str(self.vehicle.mode.name) == "GUIDED":
                    if self.isRunWp is True:
                        if self.waypoint_index < len(self.waypoints):
                            current_waypoint = self.waypoints[self.waypoint_index]
                            if self.waypoint_index_recv != current_waypoint:
                                self.flag_change_dis = False
                                self.waypoint_index_recv = current_waypoint
                                self.goto_waypoint(current_waypoint)
                            else:
                                distance = self.distance_to_current_waypoint(current_waypoint)
                                print(f"Distance to waypoint: {distance:.2f} meters, waypoint: {self.waypoint_index}")
                                if distance < 1:  # You can adjust this threshold
                                    if self.flag_change_dis is False:
                                        self.flag_change_dis = True
                                        print("Waypoint reached!")
                                        self.waypoint_index += 1
                        else:
                            self.isFinished = True
                            self.isRunWp = False
                            self.waypoint_index = 0

if __name__ == "__main__":
    # Example usage
    controller = DroneController(connection_string="/dev/ttyACM0", server_url='http://103.167.198.50:5000')
