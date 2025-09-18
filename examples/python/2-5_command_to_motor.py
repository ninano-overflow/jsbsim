from math import degrees
import serial
import serial.tools.list_ports
import time
import threading
import queue
from pymavlink import mavutil

# sim_vehicle.py -v ArduPlane -f jsbsim --out 192.168.2.105:14550 --out 192.168.2.105:14551 --out 127.0.0.1:15000
MESSAGE_INTERVAL = 10
FC_PORT = "/dev/ttyACM0"
FC_SERIAL = "36003F001251303337323731"
FC_LOCATION = "3-1:1.0"
ARDUINO_YAW_SERIAL = "34331323635351517291"
ARDUINO_YAW_LOCATION = "3-5:1.0"
ARDUINO_ROLL_SERIAL = "34336313537351C0D082"
ARDUINO_ROLL_LOCATION = "3-3:1.0"
ARDUINO_PITCH_SERIAL = "7513330333235170E010"
ARDUINO_PITCH_LOCATION = "3-9:1.0"


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
                response = self.master.recv_match(type="ATTITUDE", blocking=True)

                if response and "ATTITUDE" in self.master.messages:
                    self.roll_angle_radians = self.master.messages["ATTITUDE"].roll
                    self.pitch_angle_radians = self.master.messages["ATTITUDE"].pitch
                    self.yaw_angle_radians = self.master.messages["ATTITUDE"].yaw
                    self.roll_angle = degrees(self.roll_angle_radians)
                    self.pitch_angle = degrees(self.pitch_angle_radians)
                    self.yaw_angle = degrees(self.yaw_angle_radians)
                    self.is_updated = True

                    # print(
                    #     f"FC - Roll: {self.roll_angle:.2f}, Pitch: {self.pitch_angle:.2f}, Yaw: {self.yaw_angle:.2f}"
                    # )

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

    def command_motor(self, type: str, value: float):
        print(f"Commanded {type} motor to {value}")

        if type == "roll":
            self.roll_motor.write(f"T{value}\n".encode())
        elif type == "pitch":
            self.pitch_motor.write(f"T{value}\n".encode())
        elif type == "yaw":
            self.yaw_motor.write(f"T{value}\n".encode())
        else:
            raise ValueError(f"Invalid motor type: {type}")


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
                response = self.master.recv_match(type="ATTITUDE", blocking=True)

                if response and "ATTITUDE" in self.master.messages:
                    self.roll_angle_radians = self.master.messages["ATTITUDE"].roll
                    self.pitch_angle_radians = self.master.messages["ATTITUDE"].pitch
                    self.yaw_angle_radians = self.master.messages["ATTITUDE"].yaw
                    self.roll_angle = degrees(self.roll_angle_radians)
                    self.pitch_angle = degrees(self.pitch_angle_radians)
                    self.yaw_angle = degrees(self.yaw_angle_radians)
                    self.is_updated = True
                    # print(
                    #         f"SITL - Roll: {self.roll_angle:.2f}, Pitch: {self.pitch_angle:.2f}, Yaw: {self.yaw_angle:.2f}"
                    #     )

            except Exception as e:
                print(f"SITL monitoring error: {e}")


g = Gimbal()
g.connect_motors()
fc = FlightController(FC_PORT)
fc.connect_flight_controller()
fc.request_message_intervals()
print("Connecting to SITL")
sitl = SITL()
sitl.connect_sitl()
sitl.request_message_intervals()
print("Starting monitoring threads...")


def compare_attitudes():
    global fc, sitl, g
    while True:
        # print(
        #     f"FC: roll: {fc.roll_angle} pitch: {fc.pitch_angle} yaw: {fc.yaw_angle}, SITL: {sitl.roll_angle} pitch: {sitl.pitch_angle} yaw: {sitl.yaw_angle}"
        # )
        if (
            sitl.roll_angle is not None
            and fc.roll_angle is not None
            and sitl.pitch_angle is not None
            and fc.pitch_angle is not None
            and sitl.yaw_angle is not None
            and fc.yaw_angle is not None
        ):
            roll_diff = round(float(sitl.roll_angle_radians - fc.roll_angle_radians), 2)
            pitch_diff = round(
                float(sitl.pitch_angle_radians - fc.pitch_angle_radians), 2
            )
            yaw_diff = round(float(fc.yaw_angle_radians - sitl.yaw_angle_radians), 2)
            print(
                f" Roll diff: {roll_diff}, Pitch diff: {pitch_diff}, Yaw diff: {yaw_diff}"
            )
            # g.command_motor("roll", round(float(sitl.roll_angle_radians - fc.roll_angle_radians), 2))
            # g.command_motor("pitch", round(float(sitl.pitch_angle_radians - fc.pitch_angle_radians), 2))
            g.command_motor("yaw", roll_diff)
        else:
            print(
                f" Roll: {sitl.roll_angle}, Pitch: {sitl.pitch_angle}, Yaw: {sitl.yaw_angle}, FC Roll: {fc.roll_angle}, Pitch: {fc.pitch_angle}, Yaw: {fc.yaw_angle}"
            )
        time.sleep(0.1)


def main():
    # Shared data queue and stop event
    # Create threads
    fc_thread = threading.Thread(target=fc.monitor_attitude, name="FC_Monitor")
    sitl_thread = threading.Thread(target=sitl.monitor_attitude, name="SITL_Monitor")
    compare_thread = threading.Thread(target=compare_attitudes, name="Comparator")

    # Start threads
    fc_thread.start()
    sitl_thread.start()
    compare_thread.start()

    try:
        print("Press Ctrl+C to stop...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")

        # Wait for threads to finish
        fc_thread.join(timeout=5)
        sitl_thread.join(timeout=5)
        compare_thread.join(timeout=5)

        print("All threads stopped.")


if __name__ == "__main__":
    main()
