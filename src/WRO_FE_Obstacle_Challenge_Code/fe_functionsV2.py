# === FE_functions.py ===
# Contains reusable motor, servo, and image processing functions for Open Challenge

import RPi.GPIO as GPIO
import time
import statistics
import math
import board
import busio
import smbus2
import cv2
import numpy as np
import sys
import signal
import adafruit_vl53l1x
import adafruit_bno055
from adafruit_pca9685 import PCA9685

# === Motor Pins ===
PWMA = 18
AIN1 = 24
AIN2 = 23
STBY = 25
LED_PIN = 16
START_BUTTON_PIN = 26

# Servo SG90 PCA TCA
SERVO_MIN_US = 500
SERVO_MAX_US = 2400
PCA_SERVO_CHANNEL = 0         	# PCA9685 channel number for servo
PCA_FREQ = 50     				# Servo frequency (Hz)

# Servo angle limits
STEERING_CENTER = 95
SERVO_DEVIATION_LIMIT = 30
SERVO_MAX_RIGHT = STEERING_CENTER + SERVO_DEVIATION_LIMIT		# 90+30 
SERVO_MAX_LEFT = STEERING_CENTER - SERVO_DEVIATION_LIMIT		# 90-30
last_servo_angle = STEERING_CENTER  # to track previous angle
MAX_SERVO_STEP = 5  # max change in angle per update (degrees)

# TCA9548A Config
TCA_ADDRESS = 0x70
BNO055_CHANNEL = 3
LEFT_CHANNEL = 0
CENTER_CHANNEL = 1
RIGHT_CHANNEL = 2
PCA_TCA_CHANNEL = 4  # TCA9548A channel for PCA9685

# PID Configuration
TARGET_DISTANCE = 20   # Initial 0 cm
TOLERANCE = 3      # cm
KP = 1.0 
KI = 0.0
KD = 0.0
FRONT_DISTANCE = 60

# === Color Masks ===
"""
rBlue = [[10, 144, 86], [125, 164, 104]]  # By Tracing
rOrange = [[68, 132, 158], [110, 162, 177]]  # By Tracing
rBlack = [[0, 119, 119], [72, 137, 137]]		# By Tracing
rMagenta = [[26, 143, 123], [51, 168, 141]]		# Not tuned
rRed = [[0, 137, 113], [50, 168, 149]]			# By Tracing
rGreen = [[9, 105, 127], [78, 123, 148]]			# By Tracing
"""

rMagenta = [[16, 147, 127], [88, 171, 144]]		# Not tuned
rRed = [[17, 134, 126], [86, 162, 149]]			# By Tracing
rGreen = [[20, 102, 128], [86, 131, 155]]			# By Tracing

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup([PWMA, AIN1, AIN2, STBY, LED_PIN], GPIO.OUT)
motor_pwm = GPIO.PWM(PWMA, 1000)
motor_pwm.start(0)


# === Motor Control ===
def standby(on):
    GPIO.output(STBY, GPIO.HIGH if on else GPIO.LOW)

def forward(speed_percent):
    standby(True)
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(speed_percent)

def reverse(speed_percent):
    standby(True)
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(AIN2, GPIO.HIGH)
    motor_pwm.ChangeDutyCycle(speed_percent)

def stop_car():
    standby(False)
    motor_pwm.ChangeDutyCycle(0)
    
def perform_turn(pca, imu_sensor, direction, angle_change=45, motor_speed=95):
    # Set servo
    if direction == "right":
        set_servo_angle(pca, SERVO_MAX_RIGHT)
    else:
        set_servo_angle(pca, SERVO_MAX_LEFT)
    
    time.sleep(0.02)

    # Get starting heading
    start_heading = get_heading(imu_sensor)
    if start_heading is None:
        raise Exception("Cannot read heading from BNO055")

    #print(f"🧭 Starting Heading: {start_heading:.1f}°")

    # Start moving forward
    forward(motor_speed)

    while True:
        current_heading = get_heading(imu_sensor)
        if current_heading is None:
            continue

        diff = (current_heading - start_heading + 540) % 360 - 180
        #print(f"Heading: {current_heading:.1f}° | Δ: {diff:.1f}°", end="\r")

        if (direction == "right" and diff >= angle_change) or \
           (direction == "left" and diff <= -angle_change):
            break

        time.sleep(0.05)

    #stop_car()
    #set_servo_angle(pca, STEERING_CENTER)
    print(f"\n✅ {direction.capitalize()} turn complete. Final heading: {current_heading:.1f}°")
    


def perform_reverse_turn_to_heading(pca, imu_sensor, direction, target_heading, motor_speed=100):

    if direction == "left":
        set_servo_angle(pca, SERVO_MAX_RIGHT)
        target_heading = (target_heading - 60) % 360		# OSec:50, CSec:Cm 60, ISec:cm20,70
    else:
        set_servo_angle(pca, SERVO_MAX_LEFT)
        target_heading = (target_heading + 60) % 360		# OSec:50, CSec:cm 60, ISec:cm20,70

    time.sleep(0.05)
    print(f"**Reverse Turning {direction.upper()}... Target: {target_heading}°")
    reverse(motor_speed)

    while True:
        current_heading = get_heading(imu_sensor)
        if current_heading is None:
            continue

        # Calculate signed heading difference (-180 to 180)
        diff = (current_heading - target_heading + 540) % 360 - 180
        #print(f"Reverse Turning {direction.upper()}... Heading: {current_heading:.1f}°, "
        #      f"Target: {target_heading}°, Δ: {diff:.1f}°", end="\r")

        # Stop condition based on direction
        if (direction == "right" and diff >= 0) or \
           (direction == "left" and diff <= 0):
            break

        time.sleep(0.02)

    stop_car()
    set_servo_angle(pca, STEERING_CENTER)
    print(f"\n✅ Reverse {direction.capitalize()} turn complete. Final heading: {current_heading:.1f}°")


def right_rllr(pca, imu_sensor, turn_change=45):
    perform_turn(pca, imu_sensor, direction="right", angle_change=turn_change)
    perform_turn(pca, imu_sensor, direction="left", angle_change=turn_change)
    perform_turn(pca, imu_sensor, direction="left", angle_change=turn_change)
    perform_turn(pca, imu_sensor, direction="right", angle_change=turn_change)
    stop_car()
    set_servo_angle(pca, STEERING_CENTER)
  

def left_lrrl(pca, imu_sensor, turn_change=45):
    perform_turn(pca, imu_sensor, direction="left", angle_change=turn_change)
    perform_turn(pca, imu_sensor, direction="right", angle_change=turn_change)
    perform_turn(pca, imu_sensor, direction="right", angle_change=turn_change + 5)
    perform_turn(pca, imu_sensor, direction="left", angle_change=turn_change)
    stop_car()
    set_servo_angle(pca, STEERING_CENTER)
    
def move_back():
    stop_car()
    time.sleep(0.1)
    set_servo_angle(STEERING_CENTER)
    time.sleep(0.1)
    reverse(50)
    time.sleep(0.5)
    stop_car()
    time.sleep(0.1)
    forward(60)
    
def three_point_turn1():
    stop()
    time.sleep(0.1)
    set_servo_angle(SERVO_MAX_RIGHT)
    time.sleep(0.1)
    reverse(50)
    time.sleep(1.5)
    stop_car()
    time.sleep(0.1)
    forward(60)
    
def decide_target_heading(turn_number, turn_direction):

    if turn_direction == "right":
        if turn_number in (1, 5, 9, 13):
            return 0
        elif turn_number in (2, 6, 10):
            return 90
        elif turn_number in (3, 7, 11):
            return 180
        elif turn_number in (4, 8, 12):
            return 270

    elif turn_direction == "left":
        if turn_number in (1, 5, 9, 13):
            return 360
        elif turn_number in (2, 6, 10):
            return 270
        elif turn_number in (3, 7, 11):
            return 180
        elif turn_number in (4, 8, 12):
            return 90

    return None  # In case of invalid input

def drive_straight_until_pillar(pca, imu_sensor, target_heading, motor_speed=95, kp=1.0, y_threshold=340):
    """
    Drive straight with gyro correction until a pillar is detected with y > y_threshold.
    """
    global cPillar  # Must be updated in your vision loop

    print(f"🚗 Driving straight at heading {target_heading}° until pillar y > {y_threshold}")
    forward(motor_speed)

    try:
        while True:
            # --- Gyro correction ---
            heading = get_heading(imu_sensor)
            if heading is None:
                time.sleep(0.05)
                continue

            error = (heading - target_heading + 180) % 360 - 180
            correction = kp * error
            set_servo_angle(pca, int(STEERING_CENTER - correction))

            # --- Check pillar ---
            if cPillar.area > 0:  # Pillar detected
                if cPillar.y > y_threshold:
                    print(f"🛑 Pillar detected: y={cPillar.y}, area={cPillar.area}")
                    break
                else:
                    print(f"📏 Pillar detected but too far (y={cPillar.y})")
            else:
                print("🔍 No pillar detected — continuing straight.")

            time.sleep(0.05)
    finally:
        stop_car()
        print("✅ Stopped at pillar")
    
def drive_straight_to_second(pca, imu_sensor, target_heading, motor_speed=95, kp=1.5, duration=None):
    print(f"**Driving straight at heading {target_heading}° Time {duration:.1f}")
    forward(motor_speed)  # Start moving forward
    start_time = time.time()

    while True:
        heading = get_heading(imu_sensor)
        if heading is None:
            time.sleep(0.02)
            continue  # Skip this correction cycle

        # Calculate shortest error (handles wrap-around from 359 → 0)
        error = (heading - target_heading + 180) % 360 - 180
        correction = kp * error
        steering_angle = int(STEERING_CENTER - correction)
        set_servo_angle(pca, steering_angle)

        # Exit if time exceeded
        if duration is not None and (time.time() - start_time) >= duration:
            break

        time.sleep(0.02)  # 50 Hz loop

    stop_car()
    set_servo_angle(pca, STEERING_CENTER)
    print("✅ Stopped 'drive_straight_to_second' ")

def drive_straight_until_obstacle(pca, imu_sensor, target_heading, vl_sensor, tca_channel=1,  motor_speed=95, kp=1.5, stop_distance=20):
    print(f"** Driving straight at heading {target_heading}° until obstacle < {stop_distance} cm")
    forward(motor_speed)
    try:
        while True:
            # --- Gyro correction ---
            heading = get_heading(imu_sensor)
            if heading is None:
                continue  # skip this loop if IMU data is invalid
            print(f"Head: {heading}, T Head: {target_heading}")
            error = (heading - target_heading + 180) % 360 - 180
            correction = kp * error
            set_servo_angle(pca, int(STEERING_CENTER - correction))

            # --- Front sensor check ---
            distanceC = get_distance(vl_sensor, tca_channel)
            if distanceC is not None and distanceC < stop_distance:
                stop_car()
                print(f"Obstacle detected at {distanceC:.2f} cm — stopping.")
                break

            time.sleep(0.01)  # 50 Hz loop
    finally:
        stop_car()
        set_servo_angle(pca, STEERING_CENTER)
        print("✅ Stopped 'drive_straight_until_obstacle'")


def set_servo_angle(pca, angle):
    # Limit range
    angle = max(0, min(180, angle))
    angle = max(STEERING_CENTER - SERVO_DEVIATION_LIMIT, min(STEERING_CENTER + SERVO_DEVIATION_LIMIT, angle))
    
    # Select PCA9685 channel through TCA9548A
    select_tca_channel(PCA_TCA_CHANNEL)
    
    # Convert angle directly to duty cycle
    pulse_us = SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
    period_us = 1000000 / pca.frequency  # e.g., 50Hz → 20000µs
    duty_cycle = int((pulse_us / period_us) * 65535)
    
    # Send to PCA9685
    pca.channels[PCA_SERVO_CHANNEL].duty_cycle = duty_cycle
    #time.sleep(0.01)

def led_on():
    GPIO.output(LED_PIN, GPIO.HIGH)

def led_off():
    GPIO.output(LED_PIN, GPIO.LOW)
    
def select_tca_channel(channel):
    with smbus2.SMBus(1) as tca:
        tca.write_byte(TCA_ADDRESS, 1 << channel)
    time.sleep(0.1)

# === Sensor Initialization ===
def init_vl53l1x(channel):
    select_tca_channel(channel)
    i2c = board.I2C()
    sensor = adafruit_vl53l1x.VL53L1X(i2c)
    sensor.distance_mode = 2			# 1=short, 2=medium 3=long
    sensor.timing_budget = 100			# greater timing budget means more accurate reading (100-500)
    sensor.start_ranging()
    return sensor

def init_bno055():
    select_tca_channel(BNO055_CHANNEL)
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    time.sleep(1)
    return sensor

def init_pca9685():
    select_tca_channel(PCA_TCA_CHANNEL)
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = PCA_FREQ
    time.sleep(0.5)
    return pca

# === Sensor Reading ===
def get_distance(sensor, channel):
    select_tca_channel(channel)
    time.sleep(0.05)
    if sensor.data_ready:
        distance = sensor.distance
        sensor.clear_interrupt()
        if distance is not None:
            return distance
    return None

def get_heading(imu_sensor):
    select_tca_channel(BNO055_CHANNEL)
    time.sleep(0.04)
    euler_data = imu_sensor.euler
    if euler_data is not None and euler_data[0] is not None:
        return round(euler_data[0])
    return None

#Turn the robot by specified degrees either left or right from current heading.
def turn_to_heading(current_heading, turn_direction, degrees=90, speed=95):
    
    if turn_direction == "right":
        target = (current_heading + degrees) % 360
    else:
        target = (current_heading - degrees + 360) % 360

    print(f"@ Turning {turn_direction.upper()} to {target}°.........")

    # Set turn direction using servo steering
    turn_angle = STEERING_CENTER + SERVO_DEVIATION_LIMIT if turn_direction == "right" else STEERING_CENTER - SERVO_DEVIATION_LIMIT
    set_servo_angle(pca, turn_angle)
    forward(speed)

    while True:
        heading = get_heading(imu_sensor)
        if heading is None:
            continue

        error = (heading - target + 180) % 360 - 180
        print(f"Turning... Heading: {heading}°, Target: {target}°, Error: {error}°")
        if abs(error) < 5:
            break
        time.sleep(0.05)

    stop()
    set_servo_angle(pca, STEERING_CENTER)
    print(f"✅ Turn complete. Current Heading: {heading}°")
    time.sleep(0.05)
    
def wait_for_distance(sensor, channel, label, resume_speed=95):
    """Read distance sensor, stop if None, wait until valid, then resume."""
    value = get_distance(sensor, channel)
    if value is None:
        stop()
        print(f"⚠️ {label} sensor read failed. Waiting...")
        while value is None:
            value = get_distance(sensor, channel)
            time.sleep(0.05)
        print(f"✅ {label} sensor recovered: {value:.2f} cm")
    forward(resume_speed)
    return value

def wait_for_heading(sensor, label="Heading", resume_speed=95):
    """Read heading from IMU, stop if None, wait until valid, then resume."""
    value = get_heading(sensor)
    if value is None:
        stop()
        print(f"⚠️ {label} read failed. Waiting...")
        while value is None:
            value = get_heading(sensor)
            time.sleep(0.05)
        print(f"✅ {label} recovered: {value}°")
    forward(resume_speed)
    return value

def wait_for_start():
    print("Waiting for start button...")
    while True:
        # Wait until button is pressed (goes LOW)
        if GPIO.input(START_BUTTON_PIN) == GPIO.LOW:
            # Debounce check
            time.sleep(0.05)  # 50 ms debounce delay
            if GPIO.input(START_BUTTON_PIN) == GPIO.LOW:
                print("Button pressed! Starting robot...")
                break
        time.sleep(0.01)  # Reduce CPU usage
    
# === Image Processing ===
def find_contours(img_lab, lab_range, ROI):
    img_segmented = img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
    lower_mask = np.array(lab_range[0])
    upper_mask = np.array(lab_range[1])
    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)
    mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    return contours

def max_contour(contours, ROI):
    maxArea, maxX, maxY, mCnt = 0, 0, 0, 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            x += ROI[0] + w // 2
            y += ROI[1] + h
            if area > maxArea:
                maxArea, maxX, maxY, mCnt = area, x, y, cnt
    return [maxArea, maxX, maxY, mCnt]

def display_roi(img, ROIs, color):
    for ROI in ROIs:
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)
    return img
