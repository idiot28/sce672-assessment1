#!/usr/bin/env python3
"""
ep_teleop_sdk.py - Standalone Keyboard Teleoperation and Telemetry Acquisition
for DJI RoboMaster EP Core using the native Python SDK (Zero ROS 2 Dependencies).

Course: SCE 672 - Advanced Robotics (KFUPM)
Assessment: Term Project Assessment 1
Author: PhD Robotics Candidate
Instructor: Prof. Aamer Iqbal Bhatti

Usage:
    python ep_teleop_sdk.py --conn_type rndis --duration 30 --output telemetry_rndis_30s.csv
    python ep_teleop_sdk.py --conn_type ap    --duration 30 --output telemetry_ap_30s.csv

Key Bindings:
    'W' / 'S' : Forward / Backward (vx = +/- 0.5 m/s)
    'A' / 'D' : Lateral strafing Left / Right (vy = -/+ 0.5 m/s)
    'Q' / 'E' : Yaw CCW / CW rotation (wz = +/- 45 deg/s)
    'Space'   : Emergency Stop (vx = vy = wz = 0)
    Key Release : Immediate Failsafe Stop
    'Ctrl+C' or 'Esc' : Exit script safely
"""

import os
import sys
import time
import csv
import argparse
import threading
from datetime import datetime

# Attempt to import robomaster native SDK
try:
    import robomaster
    from robomaster import robot
    from robomaster import chassis
    from robomaster import battery
    ROBOMASTER_AVAILABLE = True
except ImportError:
    ROBOMASTER_AVAILABLE = False
    print("[WARNING] robomaster Python SDK not installed in current environment.")
    print("          Running in simulation/mock mode for verification.")

# Keyboard capture via pynput (standard) or fallback
try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False
    print("[WARNING] pynput not installed. Non-blocking keyboard capture requires 'pip install pynput'.")


class RoboMasterTeleopTelemetry:
    def __init__(self, conn_type="rndis", ip=None, output_csv="telemetry.csv",
                 vx_val=0.5, vy_val=0.5, wz_val=45.0):
        self.conn_type = conn_type.lower()
        self.ip = ip
        self.output_csv = output_csv
        self.vx_val = vx_val
        self.vy_val = vy_val
        self.wz_val = wz_val

        # Current commanded velocities
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        # Latest telemetry state (thread-safe)
        self.lock = threading.Lock()
        self.battery_soc = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR in RPM
        self.is_running = False

        # Robot instance
        self.ep_robot = None

        # Active pressed keys tracker for failsafe
        self.pressed_keys = set()

    def connect(self):
        """Initializes connection to the RoboMaster EP chassis."""
        print(f"[*] Initializing RoboMaster EP connection (mode: {self.conn_type.upper()})...")
        if ROBOMASTER_AVAILABLE:
            self.ep_robot = robot.Robot()
            if self.conn_type == "rndis":
                # Default wired USB RNDIS gateway is 192.168.42.2
                self.ep_robot.initialize(conn_type="rndis")
            elif self.conn_type == "ap":
                # Direct Wi-Fi Hotspot mode (ROBOMASTER-XXXXXX)
                self.ep_robot.initialize(conn_type="ap")
            else:
                self.ep_robot.initialize(conn_type=self.conn_type)
            
            # Print hardware version info
            version = self.ep_robot.get_version()
            print(f"[+] Connected successfully! Robot Hardware Version: {version}")

            # Subscribe to telemetry streams at 10 Hz
            self.ep_robot.battery.sub_battery_info(freq=10, callback=self._on_battery_info)
            self.ep_robot.chassis.sub_attitude(freq=10, callback=self._on_attitude_info)
            self.ep_robot.chassis.sub_esc(freq=10, callback=self._on_esc_info)
            print("[+] Subscribed to Battery, Attitude (IMU), and ESC (Wheel Encoders) at 10 Hz.")
        else:
            print("[MOCK] Simulated connection established.")

    def _on_battery_info(self, percent):
        """Callback for battery state-of-charge."""
        with self.lock:
            self.battery_soc = float(percent)

    def _on_attitude_info(self, attitude_info):
        """Callback for chassis IMU attitude (yaw, pitch, roll)."""
        with self.lock:
            self.yaw, self.pitch, self.roll = attitude_info

    def _on_esc_info(self, esc_info):
        """Callback for 4-wheel ESC telemetry: (speed[4], angle[4], timestamp, state)."""
        speed, angle, ts, state = esc_info
        with self.lock:
            self.wheel_speeds = list(speed)

    def dispatch_motion(self, vx, vy, wz):
        """Dispatches velocity commands to the motion controller with failsafe clamp."""
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_wz = wz
        if ROBOMASTER_AVAILABLE and self.ep_robot:
            # RoboMaster SDK: x is forward/backward, y is lateral (right positive), z is rotation (deg/s)
            # In standard ROS convention, positive y is left, so invert sign for SDK drive_speed
            sdk_vy = -vy  # Left command moves robot left
            self.ep_robot.chassis.drive_speed(x=vx, y=sdk_vy, z=wz, timeout=1.0)
        print(f"\r[TELEOP] Command -> vx: {vx:+.2f} m/s, vy: {vy:+.2f} m/s, wz: {wz:+.1f} deg/s", end="", flush=True)

    def stop_motion(self):
        """Immediate failsafe stop."""
        self.dispatch_motion(0.0, 0.0, 0.0)

    def on_press(self, key):
        """Key press event handler."""
        try:
            k = key.char.lower() if hasattr(key, 'char') and key.char else None
        except Exception:
            k = None

        if k:
            self.pressed_keys.add(k)
            if k == 'w':
                self.dispatch_motion(self.vx_val, 0.0, 0.0)
            elif k == 's':
                self.dispatch_motion(-self.vx_val, 0.0, 0.0)
            elif k == 'a':
                self.dispatch_motion(0.0, self.vy_val, 0.0)
            elif k == 'd':
                self.dispatch_motion(0.0, -self.vy_val, 0.0)
            elif k == 'q':
                self.dispatch_motion(0.0, 0.0, self.wz_val)
            elif k == 'e':
                self.dispatch_motion(0.0, 0.0, -self.wz_val)
        else:
            # Check for Spacebar (emergency stop)
            if key == keyboard.Key.space:
                print("\n[FAILSAFE] Spacebar pressed! Emergency Stop executed.")
                self.stop_motion()
            elif key == keyboard.Key.esc:
                print("\n[EXIT] Esc pressed. Exiting...")
                self.is_running = False
                return False

    def on_release(self, key):
        """Key release event handler: immediate failsafe stop."""
        try:
            k = key.char.lower() if hasattr(key, 'char') and key.char else None
        except Exception:
            k = None

        if k in self.pressed_keys:
            self.pressed_keys.remove(k)

        # If movement keys are released and no movement keys remain pressed, stop chassis
        movement_keys = {'w', 's', 'a', 'd', 'q', 'e'}
        if not (self.pressed_keys & movement_keys):
            self.stop_motion()

    def telemetry_logger_loop(self, duration=30):
        """Logs telemetry concurrently at 10 Hz (period = 0.1s) into a CSV file."""
        print(f"\n[*] Starting telemetry logger -> {self.output_csv} (Target duration: {duration}s)")
        fieldnames = [
            "timestamp_iso",
            "time_elapsed_sec",
            "battery_soc_pct",
            "imu_yaw_deg",
            "imu_pitch_deg",
            "imu_roll_deg",
            "wheel_speed_fl_rpm",
            "wheel_speed_fr_rpm",
            "wheel_speed_rl_rpm",
            "wheel_speed_rr_rpm",
            "cmd_vx_mps",
            "cmd_vy_mps",
            "cmd_wz_dps"
        ]

        start_time = time.time()
        end_time = start_time + duration if duration > 0 else float('inf')

        with open(self.output_csv, mode='w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            sample_count = 0
            while self.is_running and (time.time() < end_time):
                t_now = time.time()
                t_elapsed = t_now - start_time

                with self.lock:
                    soc = self.battery_soc
                    yaw, pitch, roll = self.yaw, self.pitch, self.roll
                    w_fl, w_fr, w_rl, w_rr = self.wheel_speeds
                    cvx, cvy, cwz = self.cmd_vx, self.cmd_vy, self.cmd_wz

                row = {
                    "timestamp_iso": datetime.utcnow().isoformat(),
                    "time_elapsed_sec": f"{t_elapsed:.4f}",
                    "battery_soc_pct": f"{soc:.1f}",
                    "imu_yaw_deg": f"{yaw:.2f}",
                    "imu_pitch_deg": f"{pitch:.2f}",
                    "imu_roll_deg": f"{roll:.2f}",
                    "wheel_speed_fl_rpm": f"{w_fl:.1f}",
                    "wheel_speed_fr_rpm": f"{w_fr:.1f}",
                    "wheel_speed_rl_rpm": f"{w_rl:.1f}",
                    "wheel_speed_rr_rpm": f"{w_rr:.1f}",
                    "cmd_vx_mps": f"{cvx:.2f}",
                    "cmd_vy_mps": f"{cvy:.2f}",
                    "cmd_wz_dps": f"{cwz:.1f}"
                }
                writer.writerow(row)
                sample_count += 1

                # Maintain 10 Hz streaming rate (100 ms interval)
                time.sleep(0.1)

        print(f"\n[+] Telemetry acquisition complete: {sample_count} samples logged to {self.output_csv}")

    def run(self, duration=30):
        """Runs concurrent teleoperation and telemetry logging."""
        self.connect()
        self.is_running = True

        # Launch telemetry logger thread
        logger_thread = threading.Thread(target=self.telemetry_logger_loop, args=(duration,))
        logger_thread.daemon = True
        logger_thread.start()

        # Keyboard listener
        if PYNPUT_AVAILABLE:
            listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
            listener.start()
            print("[+] Keyboard teleoperation active. Press 'W/S/A/D/Q/E' to drive, Space to stop, Esc to exit.")

            try:
                logger_thread.join()
            except KeyboardInterrupt:
                print("\n[INTERRUPT] Caught KeyboardInterrupt. Shutting down...")
            finally:
                self.is_running = False
                listener.stop()
        else:
            print("[!] Running without keyboard listener (logger only). Waiting for duration...")
            try:
                logger_thread.join()
            except KeyboardInterrupt:
                print("\n[INTERRUPT] Shutting down...")
            finally:
                self.is_running = False

        self.cleanup()

    def cleanup(self):
        """Safely stops the robot and releases SDK resources."""
        print("[*] Performing safe shutdown...")
        self.stop_motion()
        time.sleep(0.2)
        if ROBOMASTER_AVAILABLE and self.ep_robot:
            try:
                self.ep_robot.chassis.unsub_esc()
                self.ep_robot.chassis.unsub_attitude()
                self.ep_robot.battery.unsub_battery_info()
                self.ep_robot.close()
                print("[+] Robot disconnected cleanly.")
            except Exception as e:
                print(f"[!] Warning during robot disconnect: {e}")


def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP Teleoperation & 10Hz Telemetry Logging (Native SDK)")
    parser.add_argument("--conn_type", type=str, choices=["rndis", "ap", "sta"], default="rndis",
                        help="Connection type: 'rndis' (wired USB) or 'ap' (direct Wi-Fi hotspot)")
    parser.add_argument("--ip", type=str, default=None, help="Target robot IP (optional)")
    parser.add_argument("--duration", type=int, default=30, help="Trial duration in seconds (default: 30)")
    parser.add_argument("--output", type=str, default=None, help="Output CSV path (default: telemetry_<conn_type>_<duration>s.csv)")
    parser.add_argument("--vx", type=float, default=0.5, help="Translational longitudinal speed in m/s (default: 0.5)")
    parser.add_argument("--vy", type=float, default=0.5, help="Translational lateral speed in m/s (default: 0.5)")
    parser.add_argument("--wz", type=float, default=45.0, help="Rotational yaw speed in deg/s (default: 45.0)")

    args = parser.parse_args()

    if not args.output:
        args.output = f"telemetry_{args.conn_type}_{args.duration}s.csv"

    teleop = RoboMasterTeleopTelemetry(
        conn_type=args.conn_type,
        ip=args.ip,
        output_csv=args.output,
        vx_val=args.vx,
        vy_val=args.vy,
        wz_val=args.wz
    )

    teleop.run(duration=args.duration)


if __name__ == "__main__":
    main()
