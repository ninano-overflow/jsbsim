import serial
import serial.tools.list_ports
import time
from pymavlink import mavutil

# sim_vehicle.py -v ArduPlane -f jsbsim --out 192.168.2.105:14550 --out 192.168.2.105:14551 --out 127.0.0.1:15000
MESSAGE_INTERVAL = 10
FC_PORT = "/dev/ttyACM0"


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

    def find_flight_controller_ports(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "mavlink" in port.description.lower():
                print(f"Found flight controller port: {port.device}")
                return port.device

        print("Available ports:")
        for port in ports:
            print(
                f"  {port.device}: {port.description}, {port.manufacturer}, {port.name}"
            )

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
            while self.master.port.inWaiting() > 0:
                response = self.master.recv_match(type="ATTITUDE", blocking=True)

                if "ATTITUDE" in self.master.messages:
                    self.roll_angle = self.master.messages["ATTITUDE"].roll
                    self.pitch_angle = self.master.messages["ATTITUDE"].pitch
                    self.yaw_angle = self.master.messages["ATTITUDE"].yaw / 100
                    print(
                        f"Roll: {self.roll_angle}, Pitch: {self.pitch_angle}, Yaw: {self.yaw_angle}"
                    )


class Gimbal:
    def __init__(
        self,
        roll_motor_port="/dev/ttyACM0",
        pitch_motor_port="/dev/ttyACM1",
        yaw_motor_port="/dev/ttyACM2",
    ):
        self.roll_motor = None
        self.roll_motor_port = roll_motor_port
        self.roll_motor_baudrate = 115200
        self.pitch_motor = None
        self.pitch_motor_port = pitch_motor_port
        self.pitch_motor_baudrate = 115200
        self.yaw_motor = None
        self.yaw_motor_port = yaw_motor_port
        self.yaw_motor_baudrate = 115200

    def connect_motors(self):
        self.roll_motor = serial.Serial(
            self.roll_motor_port, self.roll_motor_baudrate, timeout=5
        )
        self.pitch_motor = serial.Serial(
            self.pitch_motor_port, self.pitch_motor_baudrate, timeout=5
        )
        self.yaw_motor = serial.Serial(
            self.yaw_motor_port, self.yaw_motor_baudrate, timeout=5
        )
        print("Connected to motors")

    def command_motor(self, type: str, value: float):
        if type == "roll":
            self.roll_motor.write(f"T{value}\n")
        elif type == "pitch":
            self.pitch_motor.write(f"T{value}\n")
        elif type == "yaw":
            self.yaw_motor.write(f"T{value}\n")
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
            response = self.master.recv_match(type="ATTITUDE", blocking=True)

            if "ATTITUDE" in self.master.messages:
                self.roll_angle = self.master.messages["ATTITUDE"].roll
                self.pitch_angle = self.master.messages["ATTITUDE"].pitch
                self.yaw_angle = self.master.messages["ATTITUDE"].yaw / 100
                print(
                    f"Roll: {self.roll_angle}, Pitch: {self.pitch_angle}, Yaw: {self.yaw_angle}"
                )


def main():
    fc = FlightController(FC_PORT)
    fc.connect_flight_controller()
    fc.request_message_intervals()
    fc.monitor_attitude()


if __name__ == "__main__":
    main()
