from math import degrees
import serial
import serial.tools.list_ports
import time
import threading
import queue
from pymavlink import mavutil

# sim_vehicle.py -v ArduPlane -f jsbsim --out 192.168.2.105:14550 --out 192.168.2.105:14551 --out 127.0.0.1:15000
MESSAGE_INTERVAL = 5
FC_PORT = "/dev/ttyACM0"
FC_SERIAL = "36003F001251303337323731"
FC_LOCATION = "3-1:1.0"
ARDUINO_YAW_SERIAL = "34331323635351517291"
ARDUINO_YAW_LOCATION = "3-5:1.0"
ARDUINO_ROLL_SERIAL = "34336313537351C0D082"
ARDUINO_ROLL_LOCATION = "3-3:1.0"
ARDUINO_PITCH_SERIAL = "7513330333235170E010"
ARDUINO_PITCH_LOCATION = "3-9:1.0"
ROLL_MULTIPLIER = 1
PITCH_MULTIPLIER = 1
YAW_MULTIPLIER = 1.0


def request_message_interval(master, message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,  # The MAVLink message ID
        1e6
        / frequency_hz,  # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0,
        0,
        0,
        0,  # Unused parameters
        0,  # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )


class FlightController:
    def __init__(self, flight_controller_port=FC_PORT):
        self.master = None
        self.message_interval = MESSAGE_INTERVAL

        self.flight_controller_port = flight_controller_port

        self.roll_angle = None
        self.pitch_angle = None
        self.yaw_angle = None
        self.roll_angle_radians = None
        self.pitch_angle_radians = None
        self.yaw_angle_radians = None

        self.is_updated = False
        self.attitude_lock = threading.Lock()

    def find_flight_controller_ports(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "mavlink" in port.description.lower():
                print(f"Found flight controller port: {port.device}")
                return port.device

        # print("Available ports:")
        # for port in ports:
        #     print(
        #         f"  {port.device}: {port.description}, {port.manufacturer}, {port.name}"
        #     )

        return None

    def connect_flight_controller(self):
        if self.flight_controller_port is None:
            self.flight_controller_port = self.find_flight_controller_ports()
        if self.flight_controller_port is None:
            print("No flight controller port found")
            return
        print(f"Connecting to flight controller on port: {self.flight_controller_port}")
        self.master = mavutil.mavlink_connection(
            self.flight_controller_port, autoreconnect=True
        )
        print("Connected to flight controller")

    def request_message_intervals(self):
        request_message_interval(
            self.master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, self.message_interval
        )

    def monitor_attitude(self):
        while True:
            try:
                response = self.master.recv_match(type="ATTITUDE", blocking=False)

                if response is None:
                    time.sleep(0.01)
                    continue

                if response and "ATTITUDE" in self.master.messages:
                    with self.attitude_lock:
                        self.roll_angle_radians = self.master.messages["ATTITUDE"].roll
                        self.pitch_angle_radians = self.master.messages[
                            "ATTITUDE"
                        ].pitch
                        self.yaw_angle_radians = self.master.messages["ATTITUDE"].yaw
                        self.roll_angle = degrees(self.roll_angle_radians)
                        self.pitch_angle = degrees(self.pitch_angle_radians)
                        self.yaw_angle = degrees(self.yaw_angle_radians)
                        self.is_updated = True

                # print(
                #     f"FC - Roll: {self.roll_angle:.2f}, Pitch: {self.pitch_angle:.2f}, Yaw: {self.yaw_angle:.2f}"
                # )
                # time.sleep(0.1)
            except Exception as e:
                print(f"FC monitoring error: {e}")


class Gimbal:
    def __init__(
        self,
    ):
        self.roll_motor = None
        self.roll_motor_port = None
        self.roll_motor_baudrate = 115200
        self.pitch_motor = None
        self.pitch_motor_port = None
        self.pitch_motor_baudrate = 115200
        self.yaw_motor = None
        self.yaw_motor_port = None
        self.yaw_motor_baudrate = 115200

    def connect_motors(self):
        self.roll_motor_port = self.find_motor(ARDUINO_ROLL_SERIAL)
        if not self.roll_motor_port:
            raise ValueError(f"Roll motor port not found")
        self.roll_motor = serial.Serial(
            self.roll_motor_port, self.roll_motor_baudrate, timeout=5
        )

        self.pitch_motor_port = self.find_motor(ARDUINO_PITCH_SERIAL)
        if not self.pitch_motor_port:
            raise ValueError(f"Pitch motor port not found")
        self.pitch_motor = serial.Serial(
            self.pitch_motor_port, self.pitch_motor_baudrate, timeout=5
        )

        self.yaw_motor_port = self.find_motor(ARDUINO_YAW_SERIAL)
        if not self.yaw_motor_port:
            raise ValueError(f"Yaw motor port not found")
        self.yaw_motor = serial.Serial(
            self.yaw_motor_port, self.yaw_motor_baudrate, timeout=5
        )

        print("Connected to motors")

    def find_motor(self, serial_number):
        print(f"Finding motor with serial number: {serial_number}")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if serial_number == port.serial_number:
                print(f"Found motor port: {port.device}")
                return port.device

        print("No motor port found")
        return None

    def check_motor_connection(self, motor, motor_name):
        """Check if motor connection is still alive"""
        try:
            if motor is None:
                return False
            # Try to read from motor port to check connection
            motor.timeout = 0.1  # Short timeout for health check
            motor.write(b"?\n")  # Send query command
            response = motor.read(10)  # Try to read response
            return motor.is_open
        except Exception as e:
            print(f"Motor {motor_name} connection error: {e}")
            return False

    def check_all_motors(self):
        """Check all motor connections and return status"""
        roll_ok = self.check_motor_connection(self.roll_motor, "roll")
        pitch_ok = self.check_motor_connection(self.pitch_motor, "pitch")
        yaw_ok = self.check_motor_connection(self.yaw_motor, "yaw")

        status = {
            "roll": roll_ok,
            "pitch": pitch_ok,
            "yaw": yaw_ok,
            "all_connected": roll_ok and pitch_ok and yaw_ok
        }

        if not status["all_connected"]:
            print(f"⚠️ Motor connection status: Roll={roll_ok}, Pitch={pitch_ok}, Yaw={yaw_ok}")

        return status

    def command_motor(self, type: str, value: float):
        start_time = time.time()

        try:
            if type == "roll":
                self.roll_motor.write(f"T{value}\n".encode())
            elif type == "pitch":
                self.pitch_motor.write(f"T{value}\n".encode())
            elif type == "yaw":
                self.yaw_motor.write(f"T{value}\n".encode())
            else:
                raise ValueError(f"Invalid motor type: {type}")

            # Measure command execution time
            exec_time = (time.time() - start_time) * 1000
            if exec_time > 50:  # If command takes more than 50ms
                print(f"⚠️ Slow motor command: {type} took {exec_time:.1f}ms")

        except Exception as e:
            print(f"❌ Motor command failed for {type}: {e}")
            # Check if connection is still alive
            self.check_motor_connection(getattr(self, f"{type}_motor"), type)


class SITL:
    def __init__(
        self,
        sitl_connection_type="udpin",
        sitl_address="127.0.0.1",
        sitl_port=15000,
    ):

        self.sitl_connection_type = sitl_connection_type
        self.sitl_address = sitl_address
        self.sitl_port = sitl_port
        self.master = None
        self.message_interval = MESSAGE_INTERVAL

        self.roll_angle = None
        self.pitch_angle = None
        self.yaw_angle = None
        self.roll_angle_radians = None
        self.pitch_angle_radians = None
        self.yaw_angle_radians = None

        self.is_updated = False
        self.attitude_lock = threading.Lock()

    def connect_sitl(self):
        self.master = mavutil.mavlink_connection(
            f"{self.sitl_connection_type}:{self.sitl_address}:{self.sitl_port}",
            autoreconnect=True,
        )
        print("Connected to SITL")

    def request_message_intervals(self):
        request_message_interval(
            self.master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, self.message_interval
        )

    def monitor_attitude(self):
        while True:
            try:
                response = self.master.recv_match(type="ATTITUDE", blocking=False)

                if response is None:
                    time.sleep(0.01)
                    continue

                if response and "ATTITUDE" in self.master.messages:
                    with self.attitude_lock:
                        self.roll_angle_radians = self.master.messages["ATTITUDE"].roll
                        self.pitch_angle_radians = self.master.messages[
                            "ATTITUDE"
                        ].pitch
                        self.yaw_angle_radians = self.master.messages["ATTITUDE"].yaw
                        self.roll_angle = degrees(self.roll_angle_radians)
                        self.pitch_angle = degrees(self.pitch_angle_radians)
                        self.yaw_angle = degrees(self.yaw_angle_radians)
                        self.is_updated = True
                    # print(
                    #         f"SITL - Roll: {self.roll_angle:.2f}, Pitch: {self.pitch_angle:.2f}, Yaw: {self.yaw_angle:.2f}"
                    #     )

                # time.sleep(0.1)
            except Exception as e:
                print(f"SITL monitoring error: {e}")


g = Gimbal()
g.connect_motors()
fc = FlightController(FC_PORT)
fc.connect_flight_controller()
fc.request_message_intervals()
fc.master.wait_heartbeat()
print("Connecting to SITL")
sitl = SITL()
sitl.connect_sitl()
sitl.request_message_intervals()
sitl.master.wait_heartbeat()
print("Starting monitoring threads...")


# def compare_attitudes():
#     global fc, sitl, g
#     while True:
#         # print(
#         #     f"FC: roll: {fc.roll_angle} pitch: {fc.pitch_angle} yaw: {fc.yaw_angle}, SITL: {sitl.roll_angle} pitch: {sitl.pitch_angle} yaw: {sitl.yaw_angle}"
#         # )
#         if sitl.roll_angle is not None and fc.roll_angle is not None:
#             roll_diff = round(float(fc.roll_angle_radians - sitl.roll_angle_radians), 2)
#             pitch_diff = round(
#                 float(sitl.pitch_angle_radians - fc.pitch_angle_radians), 2
#             )
#             yaw_diff = round(float(fc.yaw_angle_radians - sitl.yaw_angle_radians), 2)
#             print(
#                 f" Roll diff: {roll_diff}, Pitch diff: {pitch_diff}, Yaw diff: {yaw_diff}"
#             )
#             g.command_motor("roll", roll_diff * ROLL_MULTIPLIER)
#             g.command_motor("pitch", pitch_diff * PITCH_MULTIPLIER)
#             g.command_motor("yaw", yaw_diff * YAW_MULTIPLIER)
#         else:
#             print(
#                 f" Roll: {sitl.roll_angle}, Pitch: {sitl.pitch_angle}, Yaw: {sitl.yaw_angle}, FC Roll: {fc.roll_angle}, Pitch: {fc.pitch_angle}, Yaw: {fc.yaw_angle}"
#             )
#         time.sleep(0.1)


loop_count = 0
last_motor_check = time.time()
performance_log = []

while True:
    loop_start = time.time()
    loop_count += 1

    # Get FC data
    fc_response = fc.master.recv_match(type="ATTITUDE", blocking=False)
    if "ATTITUDE" in fc.master.messages:
        fc.roll_angle_radians = fc.master.messages["ATTITUDE"].roll
        fc.pitch_angle_radians = fc.master.messages["ATTITUDE"].pitch
        fc.yaw_angle_radians = fc.master.messages["ATTITUDE"].yaw
        fc.roll_angle = degrees(fc.roll_angle_radians)
        fc.pitch_angle = degrees(fc.pitch_angle_radians)
        fc.yaw_angle = degrees(fc.yaw_angle_radians)
        fc.is_updated = True

    # Get SITL data
    sitl_response = sitl.master.recv_match(type="ATTITUDE", blocking=False)
    if "ATTITUDE" in sitl.master.messages:
        sitl.roll_angle_radians = sitl.master.messages["ATTITUDE"].roll
        sitl.pitch_angle_radians = sitl.master.messages["ATTITUDE"].pitch
        sitl.yaw_angle_radians = sitl.master.messages["ATTITUDE"].yaw
        sitl.roll_angle = degrees(sitl.roll_angle_radians)
        sitl.pitch_angle = degrees(sitl.pitch_angle_radians)
        sitl.yaw_angle = degrees(sitl.yaw_angle_radians)
        sitl.is_updated = True

    # Command motors and calculate differences
    if sitl.roll_angle is not None and fc.roll_angle is not None:
        roll_diff = round(float(fc.roll_angle_radians - sitl.roll_angle_radians), 2)
        pitch_diff = round(float(sitl.pitch_angle_radians - fc.pitch_angle_radians), 2)
        yaw_diff = round(float(fc.yaw_angle_radians - sitl.yaw_angle_radians), 2)

        # Measure motor command performance
        motor_start = time.time()
        g.command_motor("roll", roll_diff * ROLL_MULTIPLIER)
        g.command_motor("pitch", pitch_diff * PITCH_MULTIPLIER)
        g.command_motor("yaw", yaw_diff * YAW_MULTIPLIER)
        motor_time = (time.time() - motor_start) * 1000

        # Log performance data
        loop_time = (time.time() - loop_start) * 1000
        performance_log.append(loop_time)

        # Print with performance metrics
        print(f"{time.time():.3f} Roll diff: {roll_diff}, Pitch diff: {pitch_diff}, Yaw diff: {yaw_diff} | Loop: {loop_time:.1f}ms, Motors: {motor_time:.1f}ms")

        # Check for performance degradation
        if loop_time > 200:  # If loop takes more than 200ms
            print(f"🚨 SLOW LOOP DETECTED: {loop_time:.1f}ms (should be ~100ms)")

    # Periodic motor connection check (every 10 seconds)
    if time.time() - last_motor_check > 10:
        motor_status = g.check_all_motors()
        if not motor_status["all_connected"]:
            print(f"🚨 MOTOR CONNECTION ISSUE DETECTED at loop {loop_count}")
        last_motor_check = time.time()

    # Performance analysis every 50 loops
    if loop_count % 50 == 0:
        if performance_log:
            avg_time = sum(performance_log[-50:]) / min(50, len(performance_log))
            print(f"📊 Performance Report (Loop {loop_count}): Avg loop time: {avg_time:.1f}ms")

            # Check for performance degradation
            if avg_time > 150:
                print(f"⚠️ PERFORMANCE DEGRADATION: Average loop time increased to {avg_time:.1f}ms")

    time.sleep(0.1)
