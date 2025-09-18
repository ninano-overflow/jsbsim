from math import degrees
import serial
import serial.tools.list_ports
import time
import threading
import queue
from pymavlink import mavutil

# sim_vehicle.py -v ArduPlane -f jsbsim --out 192.168.2.105:14550 --out 192.168.2.105:14551 --out 127.0.0.1:15000
MESSAGE_INTERVAL = 5
ARDUINO_YAW_SERIAL = "34331323635351517291"
ARDUINO_YAW_LOCATION = "3-5:1.0"
ARDUINO_ROLL_SERIAL = "34336313537351C0D082"
ARDUINO_ROLL_LOCATION = "3-3:1.0"
ARDUINO_PITCH_SERIAL = "7513330333235170E010"
ARDUINO_PITCH_LOCATION = "3-9:1.0"
ROLL_MULTIPLIER = 1
PITCH_MULTIPLIER = 1
YAW_MULTIPLIER = 1.0


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
            self.roll_motor_port,
            self.roll_motor_baudrate,
            timeout=0.1,
            write_timeout=0.1,
            inter_byte_timeout=0.05,
        )

        self.pitch_motor_port = self.find_motor(ARDUINO_PITCH_SERIAL)
        if not self.pitch_motor_port:
            raise ValueError(f"Pitch motor port not found")
        self.pitch_motor = serial.Serial(
            self.pitch_motor_port,
            self.pitch_motor_baudrate,
            timeout=0.1,
            write_timeout=0.1,
            inter_byte_timeout=0.05,
        )

        self.yaw_motor_port = self.find_motor(ARDUINO_YAW_SERIAL)
        if not self.yaw_motor_port:
            raise ValueError(f"Yaw motor port not found")
        self.yaw_motor = serial.Serial(
            self.yaw_motor_port,
            self.yaw_motor_baudrate,
            timeout=0.1,
            write_timeout=0.1,
            inter_byte_timeout=0.05,
        )

        # Clear any existing buffers
        self.roll_motor.reset_input_buffer()
        self.roll_motor.reset_output_buffer()
        self.pitch_motor.reset_input_buffer()
        self.pitch_motor.reset_output_buffer()
        self.yaw_motor.reset_input_buffer()
        self.yaw_motor.reset_output_buffer()

        print("Connected to motors with optimized timeouts")

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
            "all_connected": roll_ok and pitch_ok and yaw_ok,
        }

        if not status["all_connected"]:
            print(
                f"⚠️ Motor connection status: Roll={roll_ok}, Pitch={pitch_ok}, Yaw={yaw_ok}"
            )

        return status

    def reconnect_motor(self, motor_type):
        """Reconnect a specific motor"""
        try:
            print(f"🔄 Attempting to reconnect {motor_type} motor...")

            if motor_type == "roll":
                if self.roll_motor and self.roll_motor.is_open:
                    self.roll_motor.close()
                self.roll_motor = serial.Serial(
                    self.roll_motor_port,
                    self.roll_motor_baudrate,
                    timeout=0.1,
                    write_timeout=0.1,
                    inter_byte_timeout=0.05,
                )
                self.roll_motor.reset_input_buffer()
                self.roll_motor.reset_output_buffer()
            elif motor_type == "pitch":
                if self.pitch_motor and self.pitch_motor.is_open:
                    self.pitch_motor.close()
                self.pitch_motor = serial.Serial(
                    self.pitch_motor_port,
                    self.pitch_motor_baudrate,
                    timeout=0.1,
                    write_timeout=0.1,
                    inter_byte_timeout=0.05,
                )
                self.pitch_motor.reset_input_buffer()
                self.pitch_motor.reset_output_buffer()
            elif motor_type == "yaw":
                if self.yaw_motor and self.yaw_motor.is_open:
                    self.yaw_motor.close()
                self.yaw_motor = serial.Serial(
                    self.yaw_motor_port,
                    self.yaw_motor_baudrate,
                    timeout=0.1,
                    write_timeout=0.1,
                    inter_byte_timeout=0.05,
                )
                self.yaw_motor.reset_input_buffer()
                self.yaw_motor.reset_output_buffer()

            print(f"✅ {motor_type} motor reconnected successfully")
            return True
        except Exception as e:
            print(f"❌ Failed to reconnect {motor_type} motor: {e}")
            return False

    def command_motor(self, type: str, value: float):
        start_time = time.time()
        max_retries = 2

        for attempt in range(max_retries + 1):
            try:
                # Get the motor object
                motor = getattr(self, f"{type}_motor")

                if motor is None or not motor.is_open:
                    if attempt < max_retries:
                        print(
                            f"🔄 Motor {type} not available, attempting reconnection..."
                        )
                        if not self.reconnect_motor(type):
                            continue
                        motor = getattr(self, f"{type}_motor")
                    else:
                        print(f"❌ Motor {type} permanently unavailable")
                        return

                # Clear buffers before sending command
                motor.reset_input_buffer()
                motor.reset_output_buffer()

                # Send command
                command = f"T{value}\n".encode()
                motor.write(command)
                motor.flush()  # Force write

                # Measure command execution time
                exec_time = (time.time() - start_time) * 1000

                if exec_time > 50:  # If command takes more than 50ms
                    print(f"⚠️ Slow motor command: {type} took {exec_time:.1f}ms")
                    if exec_time > 200 and attempt < max_retries:  # Retry if very slow
                        print(
                            f"🔄 Retrying {type} motor command (attempt {attempt + 2})"
                        )
                        continue

                return  # Success, exit retry loop

            except serial.SerialTimeoutException:
                print(f"⏰ Timeout on {type} motor (attempt {attempt + 1})")
                if attempt < max_retries:
                    time.sleep(0.01)  # Brief delay before retry
                    continue
                else:
                    print(f"❌ {type} motor timeout after {max_retries + 1} attempts")

            except Exception as e:
                print(
                    f"❌ Motor command failed for {type} (attempt {attempt + 1}): {e}"
                )
                if attempt < max_retries:
                    # Try to reconnect
                    self.reconnect_motor(type)
                    continue
                else:
                    print(
                        f"❌ {type} motor permanently failed after {max_retries + 1} attempts"
                    )

    def reset_motor(self, type: str):
        self.command_motor(type, 0)

    def reset_motor_all(self):
        self.reset_motor("roll")
        self.reset_motor("pitch")
        self.reset_motor("yaw")


g = Gimbal()
g.connect_motors()
g.reset_motor_all()

time.sleep(3)
