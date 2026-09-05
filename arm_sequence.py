"""Reusable RoboGame arm sequence using an externally owned STM32 serial link.

Importing this module only defines constants and :class:`ArmSequence`. It never
opens a serial port or sends a command.
"""

import time


# This is a fixed upward move, not homing: there is no upper limit switch.
STARTUP_RAISE_STEPS = -1500
STARTUP_RAISE_PPS = 800


class ArmSequence:
    """Run the verified Servo and Stepper sequence on a supplied serial link."""

    def __init__(self, stm32_ser):
        self.ser = stm32_ser

    def tx(self, cmd):
        """Send one line to STM32."""
        print(f"[TX] {cmd}")
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()

    def wait_for(self, expected_prefix, timeout=10.0):
        """Wait for an STM32 response prefix and fail immediately on ERR."""
        deadline = time.time() + timeout

        while time.time() < deadline:
            line = self.ser.readline().decode(errors="ignore").strip()

            if not line:
                continue

            print(f"[RX] {line}")

            if line.startswith("ERR,"):
                raise RuntimeError(f"STM32 error: {line}")

            if line.startswith(expected_prefix):
                return line

        raise TimeoutError(f"Timeout waiting for: {expected_prefix}")

    def hello(self):
        self.tx("HELLO_STM32,1")
        self.wait_for("STM32_OK,1", 2.0)

    def stop(self, require_ack=False):
        """Send the global STOP command, optionally requiring its ACK."""
        self.tx("STOP")

        try:
            return self.wait_for("ACK,STOP", 2.0)
        except Exception as exc:
            if require_ack:
                raise
            print(f"[WARN] STOP response: {exc}")
            return None

    def servo_get(self, index):
        self.tx(f"SERVO_GET,{index}")
        line = self.wait_for(f"SERVO_POS,{index},", 2.0)
        return int(line.split(",")[2])

    def servo_set(self, index, angle, wait=0.0):
        """Set a Servo absolute angle and optionally wait for its motion."""
        self.tx(f"SERVO_SET,{index},{angle}")
        self.wait_for(f"ACK,SERVO_SET,{index},{angle}", 2.0)

        if wait > 0:
            time.sleep(wait)

    def servo_sweep(self, index, target, step_delay_ms):
        """Move a Servo by 1 degree per command, matching the old Sweep_To."""
        current = self.servo_get(index)

        if current == target:
            return

        direction = 1 if target > current else -1

        for angle in range(current + direction, target + direction, direction):
            self.servo_set(index, angle)
            time.sleep(step_delay_ms / 1000.0)

    def stepper_move(self, steps, pps=800):
        """Move the Stepper and wait for both ACK and MOVE_DONE."""
        self.tx(f"STEPPER_MOVE,{steps},{pps}")
        self.wait_for(f"ACK,STEPPER_MOVE,{steps},{pps}", 2.0)

        nominal = abs(steps) / float(pps)
        timeout = nominal * 2.0 + 8.0
        self.wait_for(f"STEPPER_MOVE_DONE,{steps}", timeout)

    def startup_raise(self):
        """Perform the verified fixed startup raise; this is not homing."""
        print()
        print("========================================")
        print("        STARTUP STEPPER RAISE")
        print("========================================")
        print(
            f"Raise: {STARTUP_RAISE_STEPS} steps "
            f"@ {STARTUP_RAISE_PPS} pps"
        )

        self.stepper_move(STARTUP_RAISE_STEPS, STARTUP_RAISE_PPS)

        print("Startup raise completed.")
        print("========================================")
        print()

    def group1(self):
        print()
        print("========== GROUP 1 START ==========")
        print()

        # Original Test_All delay.
        time.sleep(3.0)

        self.servo_set(0, 95)
        self.servo_set(3, 95)
        self.servo_set(4, 140)
        self.servo_set(2, 120)
        self.servo_sweep(1, 170, 10)
        time.sleep(0.5)

        self.stepper_move(3000, 800)
        self.servo_set(4, 105)
        time.sleep(0.5)
        self.stepper_move(-3000, 800)
        time.sleep(0.5)

        self.servo_set(1, 178)
        time.sleep(0.5)
        self.servo_set(2, 180)
        time.sleep(0.5)
        self.servo_set(2, 0)
        self.servo_set(1, 30)

        self.stepper_move(10500, 800)
        time.sleep(0.5)
        self.servo_set(4, 135)
        time.sleep(0.5)
        self.stepper_move(-10500, 800)
        time.sleep(0.5)

        print()
        print("========== GROUP 1 DONE ==========")
        print()

    def group2(self):
        print()
        print("========== GROUP 2 START ==========")
        print()

        self.servo_set(0, 0)
        self.servo_set(1, 90)
        time.sleep(0.5)
        self.servo_set(2, 50)
        self.servo_set(4, 140)
        time.sleep(0.5)

        self.stepper_move(2800, 800)
        time.sleep(0.5)
        self.servo_set(4, 105)
        time.sleep(0.5)
        self.stepper_move(-2800, 800)
        time.sleep(0.5)

        self.servo_set(2, 0)
        self.servo_set(1, 35)
        time.sleep(0.5)
        self.servo_set(0, 95)

        self.stepper_move(4800, 800)
        time.sleep(0.5)
        self.servo_set(4, 135)
        time.sleep(0.5)
        self.stepper_move(-4800, 800)
        time.sleep(0.5)

        print()
        print("========== GROUP 2 DONE ==========")
        print()

    def safe_home(self):
        """Return to the verified safe HOME while keeping Servo4 open."""
        print()
        print("Returning arm to SAFE HOME...")

        self.servo_set(4, 135)
        time.sleep(0.5)
        self.servo_set(0, 95)
        self.servo_set(1, 135)
        self.servo_set(2, 90)
        self.servo_set(3, 95)
        time.sleep(1.0)

        print("Arm returned to SAFE HOME.")

    def group3(self):
        print()
        print("========== GROUP 3 START ==========")
        print()

        self.servo_set(0, 180)
        self.servo_set(1, 90)
        time.sleep(0.5)
        self.servo_set(2, 50)
        time.sleep(0.5)
        self.servo_set(4, 140)
        time.sleep(0.5)
        self.servo_set(3, 15)
        time.sleep(0.5)

        self.stepper_move(2800, 800)
        time.sleep(0.5)
        self.servo_set(4, 105)
        time.sleep(0.5)
        self.stepper_move(-2800, 800)
        time.sleep(0.5)

        self.servo_set(3, 100)
        time.sleep(0.5)
        self.servo_set(2, 90)
        self.servo_set(1, 0)
        time.sleep(0.5)
        self.servo_set(0, 90)
        time.sleep(2.0)
        self.servo_set(2, 0)
        time.sleep(0.5)
        self.servo_sweep(1, 32, 20)
        time.sleep(2.0)

        self.servo_set(4, 135)
        time.sleep(0.5)
        self.servo_set(1, 0)
        time.sleep(2.0)
        self.servo_set(0, 180)
        time.sleep(0.5)
        self.servo_set(1, 135)
        time.sleep(0.5)

        self.safe_home()

        print()
        print("========== GROUP 3 DONE ==========")
        print()

    def run_all(self):
        print()
        print("========================================")
        print("       FULL BUILD SEQUENCE START")
        print("========================================")
        print()

        self.group1()
        self.group2()
        self.group3()

        print()
        print("========================================")
        print("       FULL BUILD SEQUENCE DONE")
        print("========================================")
        print()
