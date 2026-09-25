#!/usr/bin/env python3
"""
ep_autonomous_pick_and_place.py - Autonomous Pick-and-Place for DJI RoboMaster EP Core.

Mission:
1. Visually search and locate a green bottle using HSV color segmentation and contour geometry.
2. Visually servo the chassis towards the green bottle until within grasping range.
3. Deploy the robotic arm and close the gripper to securely grasp the bottle.
4. Search the environment for a solid white square on the ground.
5. Visually servo and approach the white square.
6. Lower the robotic arm, release the gripper to place the bottle on the white square.
7. Retract the robotic arm and reverse chassis to clear the target area.

Usage:
    # Run on physical robot (Wired RNDIS):
    python ep_autonomous_pick_and_place.py --conn_type rndis

    # Run on physical robot (Direct Wi-Fi AP):
    python ep_autonomous_pick_and_place.py --conn_type ap

    # Run in simulation/mock mode with synthetic visual feed:
    python ep_autonomous_pick_and_place.py --mock --gui

Course: SCE 672 - Advanced Robotics (KFUPM)
"""

import os
import sys
import time
import math
import argparse
import numpy as np
import cv2

# Attempt to import native robomaster SDK
try:
    import robomaster
    from robomaster import robot
    from robomaster import camera
    from robomaster import chassis
    from robomaster import robotic_arm
    from robomaster import gripper
    ROBOMASTER_AVAILABLE = True
except ImportError:
    ROBOMASTER_AVAILABLE = False


# =============================================================================
# FINITE STATE MACHINE (FSM) DEFINITION
# =============================================================================
class MissionState:
    INIT = "INIT"
    SEARCH_BOTTLE = "SEARCH_BOTTLE"
    APPROACH_BOTTLE = "APPROACH_BOTTLE"
    GRASP_BOTTLE = "GRASP_BOTTLE"
    SEARCH_WHITE_SQUARE = "SEARCH_WHITE_SQUARE"
    APPROACH_WHITE_SQUARE = "APPROACH_WHITE_SQUARE"
    PLACE_BOTTLE = "PLACE_BOTTLE"
    RETRACT_AND_FINISH = "RETRACT_AND_FINISH"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


# =============================================================================
# PERCEPTION MODULE
# =============================================================================
class RoboVision:
    def __init__(self):
        # HSV Color Thresholds for Green Bottle
        # Standard green plastic/glass in indoor lighting
        self.lower_green = np.array([35, 60, 40], dtype=np.uint8)
        self.upper_green = np.array([85, 255, 255], dtype=np.uint8)

        # HSV Color Thresholds for Solid White Square on Ground
        # Low saturation, high brightness
        self.lower_white = np.array([0, 0, 180], dtype=np.uint8)
        self.upper_white = np.array([180, 50, 255], dtype=np.uint8)

        self.kernel = np.ones((5, 5), np.uint8)

    def detect_green_bottle(self, frame):
        """
        Detects green bottle in frame.
        Returns:
            found (bool)
            centroid (cx, cy)
            bbox (x, y, w, h)
            annotated_frame
        """
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Color thresholding
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_candidate = None
        max_area = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 600:  # Noise threshold
                continue

            bx, by, bw, bh = cv2.boundingRect(c)
            aspect_ratio = bh / float(bw)

            # A bottle standing upright typically has height >= width
            if aspect_ratio >= 1.0 and area > max_area:
                max_area = area
                cx = bx + bw // 2
                cy = by + bh // 2
                best_candidate = {
                    "cx": cx,
                    "cy": cy,
                    "bbox": (bx, by, bw, bh),
                    "area": area,
                    "aspect_ratio": aspect_ratio
                }

        return best_candidate

    def detect_white_square(self, frame):
        """
        Detects solid white square on the ground (lower half of image).
        Returns:
            best_candidate dict or None
        """
        h, w = frame.shape[:2]
        # Ground ROI: focus on bottom 60% of frame
        ground_roi_y = int(h * 0.35)
        roi = frame[ground_roi_y:h, 0:w]

        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, self.lower_white, self.upper_white)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_candidate = None
        max_area = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 800:  # Minimum square area
                continue

            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = float(area) / hull_area if hull_area > 0 else 0.0

            # Solid white square has high solidity (> 0.80)
            if solidity >= 0.80 and area > max_area:
                bx, by, bw, bh = cv2.boundingRect(c)
                # Global coordinates
                gy = by + ground_roi_y
                cx = bx + bw // 2
                cy = gy + bh // 2

                max_area = area
                best_candidate = {
                    "cx": cx,
                    "cy": cy,
                    "bbox": (bx, gy, bw, bh),
                    "area": area,
                    "solidity": solidity
                }

        return best_candidate


# =============================================================================
# SYNTHETIC SIMULATION ENVIRONMENT (MOCK MODE)
# =============================================================================
class MockRoboMasterEnvironment:
    """Simulates camera video frames, chassis motion, and robotic arm/gripper state."""
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # degrees

        # Target positions in simulated world (meters)
        self.bottle_pos = np.array([1.5, 0.4])    # Green bottle at (1.5, 0.4)
        self.square_pos = np.array([0.8, -1.2])   # White square at (0.8, -1.2)

        self.arm_x = 120
        self.arm_y = 80
        self.gripper_open = True
        self.holding_bottle = False

    def drive_speed(self, x, y, z, timeout=1.0, dt=0.05):
        # Update simulated robot pose
        rad = math.radians(self.robot_yaw)
        dx = (x * math.cos(rad) - y * math.sin(rad)) * dt
        dy = (x * math.sin(rad) + y * math.cos(rad)) * dt
        self.robot_x += dx
        self.robot_y += dy
        self.robot_yaw = (self.robot_yaw - z * dt) % 360.0

    def generate_frame(self, width=640, height=480):
        frame = np.ones((height, width, 3), dtype=np.uint8) * 60  # dark floor

        # Draw grid lines on ground
        for gy in range(height // 2, height, 40):
            cv2.line(frame, (0, gy), (width, gy), (80, 80, 80), 1)

        robot_pos = np.array([self.robot_x, self.robot_y])

        # 1. Render White Square (if not holding bottle or during search)
        rel_sq = self.square_pos - robot_pos
        dist_sq = np.linalg.norm(rel_sq)
        angle_sq = math.degrees(math.atan2(rel_sq[1], rel_sq[0])) - self.robot_yaw
        angle_sq = (angle_sq + 180) % 360 - 180

        if dist_sq < 3.5 and abs(angle_sq) < 45:
            screen_x = int(width / 2 + (angle_sq / 45.0) * (width / 2))
            screen_y = int(height * 0.4 + (1.0 / max(0.4, dist_sq)) * 90)
            sz = int(120 / max(0.4, dist_sq))
            pts = np.array([
                [screen_x - sz, screen_y - sz // 2],
                [screen_x + sz, screen_y - sz // 2],
                [screen_x + sz + 15, screen_y + sz // 2],
                [screen_x - sz - 15, screen_y + sz // 2]
            ], np.int32)
            cv2.fillPoly(frame, [pts], (240, 240, 240))
            cv2.putText(frame, "WHITE SQUARE", (screen_x - 45, screen_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (50, 50, 50), 1)

        # 2. Render Green Bottle
        if not self.holding_bottle:
            rel_bt = self.bottle_pos - robot_pos
            dist_bt = np.linalg.norm(rel_bt)
            angle_bt = math.degrees(math.atan2(rel_bt[1], rel_bt[0])) - self.robot_yaw
            angle_bt = (angle_bt + 180) % 360 - 180

            if dist_bt < 3.5 and abs(angle_bt) < 45:
                bx = int(width / 2 + (angle_bt / 45.0) * (width / 2))
                by = int(height * 0.35 + (1.0 / max(0.3, dist_bt)) * 80)
                bw = int(35 / max(0.3, dist_bt))
                bh = int(85 / max(0.3, dist_bt))
                # Draw green bottle body
                cv2.rectangle(frame, (bx - bw//2, by - bh), (bx + bw//2, by), (40, 190, 40), -1)
                # Draw cap
                cv2.rectangle(frame, (bx - bw//4, by - bh - 12), (bx + bw//4, by - bh), (30, 140, 30), -1)
                cv2.putText(frame, "GREEN BOTTLE", (bx - bw, by - bh - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (40, 220, 40), 1)
        else:
            # Bottle is in gripper (show at bottom center)
            cv2.rectangle(frame, (width//2 - 25, height - 90), (width//2 + 25, height - 10), (40, 200, 40), -1)
            cv2.putText(frame, "[IN GRIPPER]", (width//2 - 35, height - 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        return frame


# =============================================================================
# MAIN AUTONOMOUS CONTROLLER CLASS
# =============================================================================
class AutonomousPickAndPlace:
    def __init__(self, conn_type="rndis", mock=False, gui=True,
                 grasp_x=195, grasp_y=-25, nudge_sec=1.0, grasp_power=85):
        self.conn_type = conn_type
        self.mock = mock or (not ROBOMASTER_AVAILABLE)
        self.gui = gui

        # Grasp tuning parameters
        self.grasp_x = grasp_x
        self.grasp_y = grasp_y
        self.nudge_sec = nudge_sec
        self.grasp_power = grasp_power

        self.vision = RoboVision()
        self.state = MissionState.INIT
        self.state_start_time = time.time()

        self.robot = None
        self.chassis = None
        self.arm = None
        self.gripper = None
        self.camera = None
        self.mock_env = None

        # Tracking variables
        self.search_direction = 1.0  # 1: CCW, -1: CW
        self.approach_stable_count = 0
        self.lost_target_count = 0

    def initialize(self):
        """Initializes robot hardware modules or mock simulation."""
        print(f"[*] Initializing Autonomous Pick-and-Place (Mode: {'MOCK' if self.mock else self.conn_type.upper()})...")

        if not self.mock:
            self.robot = robot.Robot()
            self.robot.initialize(conn_type=self.conn_type)
            self.chassis = self.robot.chassis
            self.arm = self.robot.robotic_arm
            self.gripper = self.robot.gripper
            self.camera = self.robot.camera

            # Start video stream
            self.camera.start_video_stream(display=False)
            print("[+] Hardware connected: Chassis, Robotic Arm, Gripper, and Camera active.")

            # Home arm & open gripper
            self.arm.moveto(x=120, y=80).wait_for_completed()
            self.gripper.open()
            time.sleep(1.0)
        else:
            self.mock_env = MockRoboMasterEnvironment()
            print("[+] Initialized Mock Simulation Environment.")

        self.transition(MissionState.SEARCH_BOTTLE)

    def transition(self, new_state):
        """State transition helper."""
        prev = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.approach_stable_count = 0
        self.lost_target_count = 0
        print(f"\n==================================================")
        print(f"[*] FSM TRANSITION: [{prev}] -> [{new_state}]")
        print(f"==================================================")

    def get_frame(self):
        """Retrieves camera frame from SDK or mock generator."""
        if not self.mock and self.camera:
            frame = self.camera.read_cv2_image(strategy="newest")
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
            return frame
        else:
            return self.mock_env.generate_frame()

    def set_motion(self, vx, vy, wz):
        """Dispatches chassis velocities."""
        if not self.mock and self.chassis:
            # RoboMaster SDK: positive y is right, positive z is CCW
            sdk_vy = -vy
            self.chassis.drive_speed(x=vx, y=sdk_vy, z=wz, timeout=0.5)
        elif self.mock_env:
            self.mock_env.drive_speed(x=vx, y=vy, z=wz)

    def stop_motion(self):
        self.set_motion(0.0, 0.0, 0.0)

    # -------------------------------------------------------------------------
    # STATE HANDLERS
    # -------------------------------------------------------------------------
    def handle_search_bottle(self, frame):
        """State: Rotate chassis in-place to locate the green bottle."""
        target = self.vision.detect_green_bottle(frame)

        if target:
            print(f"[+] Green Bottle detected at centroid ({target['cx']}, {target['cy']}) | Area: {target['area']:.0f}")
            self.stop_motion()
            self.transition(MissionState.APPROACH_BOTTLE)
            return

        # Rotate chassis slowly
        rot_speed = 25.0 * self.search_direction
        self.set_motion(0.0, 0.0, rot_speed)

        # Timeout check (reverse search after 8s)
        if time.time() - self.state_start_time > 8.0:
            self.search_direction *= -1.0
            self.state_start_time = time.time()

    def handle_approach_bottle(self, frame):
        """State: Visual servoing towards the green bottle."""
        h, w = frame.shape[:2]
        center_x = w / 2.0
        target = self.vision.detect_green_bottle(frame)

        if not target:
            self.lost_target_count += 1
            if self.lost_target_count >= 15:
                print("[!] Lost visual contact with bottle during approach. Re-entering search...")
                self.transition(MissionState.SEARCH_BOTTLE)
            return
        else:
            self.lost_target_count = 0

        cx = target["cx"]
        bx, by, bw, bh = target["bbox"]
        base_y = by + bh

        # Normalized coordinates relative to frame height/width
        norm_bh = bh / float(h)
        norm_base_y = base_y / float(h)

        # 1. Proportional Yaw Control (Align target tightly with center)
        err_x = (cx - center_x) / center_x  # [-1.0, 1.0]
        wz_cmd = -err_x * 35.0              # degrees/s

        # 2. Proportional Forward Velocity (Decelerate as bottle approaches)
        # Proximity achieved when bottle occupies >= 45% frame height or base reaches lower 85%
        if norm_bh >= 0.45 or norm_base_y >= 0.85:
            # Check fine lateral alignment (|err_x| < 0.08)
            if abs(err_x) < 0.08:
                self.approach_stable_count += 1
                if self.approach_stable_count >= 3:
                    print(f"[+] Grasping proximity achieved! (norm_bh: {norm_bh:.2f}, base_y: {norm_base_y:.2f}). Halting.")
                    self.stop_motion()
                    self.transition(MissionState.GRASP_BOTTLE)
                    return
            vx_cmd = 0.04
        else:
            self.approach_stable_count = 0
            dist_factor = max(0.1, (0.45 - norm_bh) / 0.45)
            vx_cmd = float(np.clip(0.35 * dist_factor, 0.10, 0.35))

        self.set_motion(vx_cmd, 0.0, wz_cmd)

    def handle_grasp_bottle(self):
        """State: Execute robotic arm and gripper grasp sequence with pre-grasp open and forward nudge."""
        print("[*] Executing robotic arm grasp sequence...")
        self.stop_motion()
        time.sleep(0.4)

        # Step 1: Open Gripper claws before approaching
        print("[*] Step 1: Opening gripper claws...")
        if not self.mock and self.gripper:
            self.gripper.open(power=50)
        elif self.mock_env:
            self.mock_env.gripper_open = True
        time.sleep(1.0)

        # Step 2: Lower Robotic Arm to ready-grasp height (x=160 mm, y=grasp_y)
        print(f"[*] Step 2: Lowering robotic arm to ready-grasp pose (x=160 mm, y={self.grasp_y} mm)...")
        if not self.mock and self.arm:
            self.arm.moveto(x=160, y=self.grasp_y).wait_for_completed()
        elif self.mock_env:
            self.mock_env.arm_x, self.mock_env.arm_y = 160, self.grasp_y
        time.sleep(1.0)

        # Step 3: Nudge chassis forward slowly with open claws so bottle slides inside
        if self.nudge_sec > 0:
            print(f"[*] Step 3: Inching forward (0.08 m/s for {self.nudge_sec}s) so bottle enters gripper...")
            self.set_motion(0.08, 0.0, 0.0)
            time.sleep(self.nudge_sec)
            self.stop_motion()
            time.sleep(0.4)

        # Step 4: Extend arm to full grasp position (x=grasp_x, y=grasp_y)
        print(f"[*] Step 4: Extending arm forward to grasp position (x={self.grasp_x} mm, y={self.grasp_y} mm)...")
        if not self.mock and self.arm:
            self.arm.moveto(x=self.grasp_x, y=self.grasp_y).wait_for_completed()
        elif self.mock_env:
            self.mock_env.arm_x, self.mock_env.arm_y = self.grasp_x, self.grasp_y
        time.sleep(0.8)

        # Step 5: Close Gripper firmly around bottle
        print(f"[*] Step 5: Clamping gripper firmly with power={self.grasp_power}...")
        if not self.mock and self.gripper:
            self.gripper.close(power=self.grasp_power)
        elif self.mock_env:
            self.mock_env.gripper_open = False
            self.mock_env.holding_bottle = True
        time.sleep(1.2)

        # Step 6: Lift Arm to Transport/Travel Pose (x=120 mm, y=85 mm)
        print("[*] Step 6: Lifting bottle to transport position (x=120 mm, y=85 mm)...")
        if not self.mock and self.arm:
            self.arm.moveto(x=120, y=85).wait_for_completed()
        elif self.mock_env:
            self.mock_env.arm_x, self.mock_env.arm_y = 120, 85
        time.sleep(1.0)

        print("[+] Bottle successfully grasped! Proceeding to find white square.")
        self.transition(MissionState.SEARCH_WHITE_SQUARE)

    def handle_search_white_square(self, frame):
        """State: Rotate chassis in-place to locate the solid white square on the ground."""
        target = self.vision.detect_white_square(frame)

        if target:
            print(f"[+] White Square detected at centroid ({target['cx']}, {target['cy']}) | Area: {target['area']:.0f}")
            self.stop_motion()
            self.transition(MissionState.APPROACH_WHITE_SQUARE)
            return

        # Rotate chassis slowly
        rot_speed = 25.0 * self.search_direction
        self.set_motion(0.0, 0.0, rot_speed)

        # Timeout check
        if time.time() - self.state_start_time > 10.0:
            self.search_direction *= -1.0
            self.state_start_time = time.time()

    def handle_approach_white_square(self, frame):
        """State: Visual servoing towards the white square placement zone."""
        h, w = frame.shape[:2]
        center_x = w / 2.0
        target = self.vision.detect_white_square(frame)

        if not target:
            self.lost_target_count += 1
            if self.lost_target_count >= 15:
                print("[!] Lost visual contact with white square. Re-searching...")
                self.transition(MissionState.SEARCH_WHITE_SQUARE)
            return
        else:
            self.lost_target_count = 0

        cx = target["cx"]
        cy = target["cy"]
        area = target["area"]

        # 1. Proportional Yaw Control
        err_x = (cx - center_x) / center_x
        wz_cmd = -err_x * 30.0

        # 2. Forward Velocity
        # Stop when square is directly in front of the front bumper (cy >= 350 px or area >= 25000 px)
        if cy >= 350 or area >= 25000:
            self.approach_stable_count += 1
            if self.approach_stable_count >= 3:
                print("[+] Reached white square placement zone! Halting chassis.")
                self.stop_motion()
                self.transition(MissionState.PLACE_BOTTLE)
                return
            vx_cmd = 0.05
        else:
            self.approach_stable_count = 0
            dist_factor = max(0.1, (350.0 - cy) / 350.0)
            vx_cmd = float(np.clip(0.3 * dist_factor, 0.10, 0.30))

        self.set_motion(vx_cmd, 0.0, wz_cmd)

    def handle_place_bottle(self):
        """State: Lower arm, open gripper to deposit bottle on white square."""
        print("[*] Executing bottle placement sequence...")
        self.stop_motion()
        time.sleep(0.5)

        # 1. Lower Arm to Placement Pose (x=185 mm, y=-30 mm)
        print("[*] Lowering robotic arm to ground surface (x=185 mm, y=-30 mm)...")
        if not self.mock and self.arm:
            self.arm.moveto(x=185, y=-30).wait_for_completed()
        elif self.mock_env:
            self.mock_env.arm_x, self.mock_env.arm_y = 185, -30
        time.sleep(1.2)

        # 2. Open Gripper to release bottle
        print("[*] Opening gripper to release bottle...")
        if not self.mock and self.gripper:
            self.gripper.open()
        elif self.mock_env:
            self.mock_env.gripper_open = True
            self.mock_env.holding_bottle = False
        time.sleep(1.0)

        # 3. Retract Arm to Travel Pose
        print("[*] Retracting robotic arm to home pose (x=120 mm, y=80 mm)...")
        if not self.mock and self.arm:
            self.arm.moveto(x=120, y=80).wait_for_completed()
        elif self.mock_env:
            self.mock_env.arm_x, self.mock_env.arm_y = 120, 80
        time.sleep(1.0)

        print("[+] Bottle successfully placed on white square!")
        self.transition(MissionState.RETRACT_AND_FINISH)

    def handle_retract_and_finish(self):
        """State: Reverse chassis away from placed bottle and complete mission."""
        print("[*] Reversing chassis (-0.3 m/s) to clear placed bottle...")
        start_rev = time.time()
        while time.time() - start_rev < 1.2:
            self.set_motion(-0.25, 0.0, 0.0)
            time.sleep(0.05)

        self.stop_motion()
        print("\n==================================================")
        print("[+] MISSION ACCOMPLISHED: Green bottle picked and placed on white square!")
        print("==================================================")
        self.transition(MissionState.COMPLETED)

    # -------------------------------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------------------------------
    def run(self, max_duration=90):
        """Main execution loop."""
        self.initialize()
        start_time = time.time()

        try:
            while self.state != MissionState.COMPLETED and (time.time() - start_time < max_duration):
                frame = self.get_frame()
                annotated = frame.copy()

                # Process state
                if self.state == MissionState.SEARCH_BOTTLE:
                    self.handle_search_bottle(frame)
                elif self.state == MissionState.APPROACH_BOTTLE:
                    self.handle_approach_bottle(frame)
                elif self.state == MissionState.GRASP_BOTTLE:
                    self.handle_grasp_bottle()
                elif self.state == MissionState.SEARCH_WHITE_SQUARE:
                    self.handle_search_white_square(frame)
                elif self.state == MissionState.APPROACH_WHITE_SQUARE:
                    self.handle_approach_white_square(frame)
                elif self.state == MissionState.PLACE_BOTTLE:
                    self.handle_place_bottle()
                elif self.state == MissionState.RETRACT_AND_FINISH:
                    self.handle_retract_and_finish()

                # Overlay UI info
                if self.gui:
                    h, w = frame.shape[:2]
                    # Target detections for annotation
                    bottle = self.vision.detect_green_bottle(frame)
                    if bottle:
                        bx, by, bw, bh = bottle["bbox"]
                        cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                        cv2.circle(annotated, (bottle["cx"], bottle["cy"]), 5, (0, 0, 255), -1)
                        cv2.putText(annotated, f"Bottle ({bottle['area']:.0f}px)", (bx, by - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    square = self.vision.detect_white_square(frame)
                    if square:
                        sx, sy, sw, sh = square["bbox"]
                        cv2.rectangle(annotated, (sx, sy), (sx + sw, sy + sh), (255, 255, 255), 2)
                        cv2.circle(annotated, (square["cx"], square["cy"]), 5, (255, 0, 0), -1)
                        cv2.putText(annotated, f"White Sq ({square['area']:.0f}px)", (sx, sy - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    # State text
                    cv2.rectangle(annotated, (10, 10), (380, 50), (0, 0, 0), -1)
                    cv2.putText(annotated, f"STATE: {self.state}", (20, 38),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                    # Crosshair
                    cv2.line(annotated, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 165, 255), 1)
                    cv2.line(annotated, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 165, 255), 1)

                    cv2.imshow("RoboMaster EP - Autonomous Pick and Place", annotated)
                    if cv2.waitKey(30) & 0xFF == ord('q'):
                        print("\n[!] User requested abort via 'q' key.")
                        break

                time.sleep(0.03)

        except KeyboardInterrupt:
            print("\n[!] Process interrupted by user.")
        finally:
            self.cleanup()

    def cleanup(self):
        """Releases robot resources and cleans up OpenCV windows."""
        print("[*] Cleaning up robot resources...")
        self.stop_motion()
        time.sleep(0.2)

        if not self.mock and self.robot:
            try:
                if self.camera:
                    self.camera.stop_video_stream()
                self.robot.close()
                print("[+] Robot disconnected cleanly.")
            except Exception as e:
                print(f"[!] Disconnect warning: {e}")

        if self.gui:
            cv2.destroyAllWindows()


# =============================================================================
# CLI ENTRY POINT
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP Autonomous Pick-and-Place (Green Bottle to White Square)")
    parser.add_argument("--conn_type", type=str, choices=["rndis", "ap", "sta"], default="rndis",
                        help="Physical connection type to robot: 'rndis' (wired USB) or 'ap' (direct Wi-Fi hotspot)")
    parser.add_argument("--mock", action="store_true",
                        help="Run in mock simulation mode (generates synthetic visual feed and simulated motion)")
    parser.add_argument("--no-gui", dest="gui", action="store_false",
                        help="Disable OpenCV visual display window")
    parser.add_argument("--duration", type=int, default=90,
                        help="Maximum mission timeout in seconds (default: 90)")
    parser.add_argument("--grasp_x", type=int, default=195,
                        help="Robotic arm grasp horizontal reach in mm (default: 195, range: 100-220)")
    parser.add_argument("--grasp_y", type=int, default=-25,
                        help="Robotic arm grasp vertical height in mm (default: -25, range: -40 to 100)")
    parser.add_argument("--nudge", type=float, default=1.0,
                        help="Chassis forward nudge duration in seconds (default: 1.0s, set 0 to disable)")
    parser.add_argument("--power", type=int, default=85,
                        help="Gripper clamping power (default: 85, range: 1-100)")

    args = parser.parse_args()

    app = AutonomousPickAndPlace(
        conn_type=args.conn_type,
        mock=args.mock,
        gui=args.gui,
        grasp_x=args.grasp_x,
        grasp_y=args.grasp_y,
        nudge_sec=args.nudge,
        grasp_power=args.power
    )
    app.run(max_duration=args.duration)


if __name__ == "__main__":
    main()
