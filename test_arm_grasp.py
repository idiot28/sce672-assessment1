#!/usr/bin/env python3
"""
test_arm_grasp.py - Standalone Robotic Arm & Gripper Grasp Calibration Tool
for DJI RoboMaster EP Core.

Use this tool to test and calibrate:
1. Gripper opening width and closing force.
2. Robotic arm extension (x in mm) and height (y in mm).
3. Chassis forward nudge distance so the bottle slides securely into the open gripper claws.

Usage:
    python test_arm_grasp.py --conn_type rndis --grasp_x 195 --grasp_y -25 --nudge 1.0
    python test_arm_grasp.py --conn_type ap    --grasp_x 195 --grasp_y -25 --nudge 1.0
"""

import sys
import time
import argparse

try:
    import robomaster
    from robomaster import robot
    from robomaster import robotic_arm
    from robomaster import gripper
    from robomaster import chassis
    ROBOMASTER_AVAILABLE = True
except ImportError:
    ROBOMASTER_AVAILABLE = False
    print("[ERROR] robomaster SDK not found.")


def run_grasp_calibration(conn_type="rndis", grasp_x=195, grasp_y=-25, 
                          nudge_sec=1.0, nudge_speed=0.08, grip_power=85):
    print("==================================================================")
    print("      DJI RoboMaster EP Core - Arm & Gripper Calibration Test     ")
    print("==================================================================")
    print(f" Parameters:")
    print(f"   Connection:     {conn_type.upper()}")
    print(f"   Grasp Reach:    x = {grasp_x} mm (Range: 100 to 220 mm)")
    print(f"   Grasp Height:   y = {grasp_y} mm (Range: -40 to 100 mm)")
    print(f"   Nudge Duration: {nudge_sec} s @ {nudge_speed} m/s")
    print(f"   Grip Power:     {grip_power}%")
    print("==================================================================")
    print("\n[!] Please place the robot approximately 15-25 cm in front of the green bottle.")
    print("    Press ENTER to begin calibration routine (or Ctrl+C to abort)...")
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        print("\nAborted.")
        return

    if not ROBOMASTER_AVAILABLE:
        print("[MOCK] Simulating grasp calibration...")
        print("  1. Home arm: (120, 80)")
        print("  2. Open gripper")
        print(f"  3. Lower arm: (160, {grasp_y})")
        print(f"  4. Nudge forward {nudge_sec}s @ {nudge_speed} m/s")
        print(f"  5. Extend arm to: ({grasp_x}, {grasp_y})")
        print(f"  6. Close gripper (power={grip_power})")
        print("  7. Lift arm to: (120, 85)")
        print("[+] Mock grasp test completed successfully!")
        return

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type=conn_type)

    arm = ep_robot.robotic_arm
    grip = ep_robot.gripper
    chas = ep_robot.chassis

    try:
        # Step 1: Home arm to safe travel height
        print("\n[*] Step 1: Homing robotic arm to safe transit pose (120 mm, 80 mm)...")
        arm.moveto(x=120, y=80).wait_for_completed()
        time.sleep(0.5)

        # Step 2: Open gripper claws
        print("[*] Step 2: Opening gripper claws...")
        grip.open(power=50)
        time.sleep(1.0)

        # Step 3: Lower arm to ready-grasp height with moderate reach (160 mm, grasp_y)
        print(f"[*] Step 3: Lowering arm to ready-grasp pose (160 mm, {grasp_y} mm)...")
        arm.moveto(x=160, y=grasp_y).wait_for_completed()
        time.sleep(0.8)

        # Step 4: Nudge chassis forward slowly with gripper open
        if nudge_sec > 0:
            print(f"[*] Step 4: Inching chassis forward ({nudge_speed} m/s for {nudge_sec}s) so bottle enters gripper...")
            chas.drive_speed(x=nudge_speed, y=0, z=0, timeout=1.0)
            time.sleep(nudge_sec)
            chas.drive_speed(x=0, y=0, z=0, timeout=0.5)
            time.sleep(0.5)

        # Step 5: Extend arm forward into the bottle
        print(f"[*] Step 5: Extending arm to full grasp position ({grasp_x} mm, {grasp_y} mm)...")
        arm.moveto(x=grasp_x, y=grasp_y).wait_for_completed()
        time.sleep(0.8)

        # Step 6: Close gripper firmly around the bottle
        print(f"[*] Step 6: Clamping gripper with power={grip_power}...")
        grip.close(power=grip_power)
        time.sleep(1.5)

        # Step 7: Lift the bottle up
        print("[*] Step 7: Lifting bottle to transport height (120 mm, 85 mm)...")
        arm.moveto(x=120, y=85).wait_for_completed()
        time.sleep(1.0)

        print("\n==================================================================")
        print("[+] Grasp test complete! Did the robot successfully lift the bottle?")
        print("    - If the arm missed (too far): increase --grasp_x or --nudge")
        print("    - If the arm hit/knocked the bottle: decrease --grasp_x or --nudge")
        print("    - If the gripper slipped off: increase --grip_power or lower --grasp_y")
        print("==================================================================")
        print("\nPress ENTER to release bottle and finish...")
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            pass

        # Release bottle safely
        print("[*] Lowering and releasing bottle...")
        arm.moveto(x=grasp_x, y=grasp_y).wait_for_completed()
        grip.open(power=50)
        time.sleep(1.0)
        arm.moveto(x=120, y=80).wait_for_completed()

    finally:
        print("[*] Closing robot connection...")
        chas.drive_speed(x=0, y=0, z=0)
        ep_robot.close()
        print("[+] Done.")


def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP Arm & Gripper Grasp Calibration Tool")
    parser.add_argument("--conn_type", type=str, choices=["rndis", "ap", "sta"], default="rndis",
                        help="Connection type: 'rndis' (USB) or 'ap' (Wi-Fi)")
    parser.add_argument("--grasp_x", type=int, default=195,
                        help="Arm horizontal reach in mm (default: 195, range: 100 to 220)")
    parser.add_argument("--grasp_y", type=int, default=-25,
                        help="Arm vertical height in mm (default: -25, range: -40 to 100)")
    parser.add_argument("--nudge", type=float, default=1.0,
                        help="Chassis forward nudge duration in seconds (default: 1.0s, set 0 to disable)")
    parser.add_argument("--nudge_speed", type=float, default=0.08,
                        help="Chassis forward nudge speed in m/s (default: 0.08)")
    parser.add_argument("--power", type=int, default=85,
                        help="Gripper clamping power (default: 85, range: 1 to 100)")

    args = parser.parse_args()
    run_grasp_calibration(
        conn_type=args.conn_type,
        grasp_x=args.grasp_x,
        grasp_y=args.grasp_y,
        nudge_sec=args.nudge,
        nudge_speed=args.nudge_speed,
        grip_power=args.power
    )


if __name__ == "__main__":
    main()
