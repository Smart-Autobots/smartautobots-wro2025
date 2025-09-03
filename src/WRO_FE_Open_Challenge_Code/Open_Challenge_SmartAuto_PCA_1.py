#PCA_1 = First Working Open Challenge with Out Side Wall, Gyro(BNO055), SG90 Servo With PCA and BO DC Motor
# With Start button and LED Working

import RPi.GPIO as GPIO
import time
import statistics
import board
import busio
import smbus2
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
TARGET_DISTANCE = 30   # Initial 0 cm
TOLERANCE = 3      # cm


# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup([PWMA, AIN1, AIN2, STBY, LED_PIN], GPIO.OUT)
motor_pwm = GPIO.PWM(PWMA, 1000)
motor_pwm.start(0)


# === Basic Functions ===
def standby(on):
    GPIO.output(STBY, GPIO.HIGH if on else GPIO.LOW)

def forward(speed_percent):
    standby(True)
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(speed_percent)

def stop():
    standby(False)
    motor_pwm.ChangeDutyCycle(0)
    
def set_servo_angle(angle):
    
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

# Returns the target heading based on turn number (t) and turn direction ("left" or "right").
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

def perform_forward_turn_to_heading(direction, target_heading, motor_speed=100):
    #Forward turn until the IMU heading reaches the given target heading.
    
    if direction == "right":
        set_servo_angle(SERVO_MAX_RIGHT)
        target_heading = (target_heading - 6) % 360
        #target_heading = (target_heading + 90) % 360
    else:
        set_servo_angle(SERVO_MAX_LEFT)
        target_heading = (target_heading + 6) % 360
        #target_heading = (target_heading - 90) % 360

    time.sleep(0.02)
    print(f"**Forward Turning {direction.upper()}... Target: {target_heading}°")
    forward(motor_speed)

    while True:
        current_heading = get_heading(imu_sensor)
        if current_heading is None:
            time.sleep(0.02)
            continue

        # Calculate signed heading difference (-180 to 180)
        diff = (current_heading - target_heading + 540) % 360 - 180
        #print(f"Forward Turning {direction.upper()}... Heading: {current_heading:.1f}°, "
        #      f"Target: {target_heading}°, Δ: {diff:.1f}°", end="\r")

        # Stop condition based on direction
        if (direction == "right" and diff >= 0) or \
           (direction == "left" and diff <= 0):
            break

        time.sleep(0.01)

    stop()
    set_servo_angle(STEERING_CENTER)
    print(f"\n✅ Forward {direction.capitalize()} turn complete. Final heading: {current_heading:.1f}°")

def drive_straight_to_second(target_heading, motor_speed=100, kp=1.5, duration=None):
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
        set_servo_angle(steering_angle)

        # Exit if time exceeded
        if duration is not None and (time.time() - start_time) >= duration:
            break

        time.sleep(0.02)  # 50 Hz loop

    stop()
    set_servo_angle(STEERING_CENTER)
    print("✅ Stopped 'drive_straight_to_second' ")


#Turn the robot by specified degrees either left or right from current heading.
def turn_to_heading(current_heading, turn_direction, degrees=90, speed=100):
    
    if turn_direction == "right":
        target = (current_heading + degrees) % 360
    else:
        target = (current_heading - degrees + 360) % 360

    print(f"@ Turning {turn_direction.upper()} to {target}°.........")

    # Set turn direction using servo steering
    turn_angle = STEERING_CENTER + SERVO_DEVIATION_LIMIT if turn_direction == "right" else STEERING_CENTER - SERVO_DEVIATION_LIMIT
    set_servo_angle(turn_angle)
    forward(speed)

    while True:
        heading = get_heading(imu_sensor)
        if heading is None:
            continue

        #error = (heading - target + 180) % 360 - 180
        error = (heading - target + 540) % 360 - 180  # gives range [-180, +180]
        print(f"Turning... Heading: {heading}°, Target: {target}°, Error: {error}°")
        if (turn_direction == "left" and error < 0) or \
           (turn_direction == "right" and error > 0):
            break
        #if abs(error) < 5:
        #    break
        time.sleep(0.05)

    stop()
    set_servo_angle(STEERING_CENTER)
    print(f"✅ Turn complete. Current Heading: {heading}°")
    time.sleep(0.05)

def wait_for_distance(sensor, channel, label, resume_speed=100, max_wait=2):
    
    value = get_distance(sensor, channel)
    
    if value is None:
        stop()
        print(f"⚠️ {label} sensor read failed. Waiting...")

        start_time = time.time()
        while value is None:
            value = get_distance(sensor, channel)
            if time.time() - start_time > max_wait:
                print(f"❌ {label} sensor timeout after {max_wait}s — resuming anyway.")
                value = 150.0
                break
            time.sleep(0.03)

        if value is not None:
            print(f"✅ {label} sensor recovered: {value:.2f} cm")

    forward(resume_speed)
    #time.sleep(0.05)
    return value

def wait_for_heading(sensor, label="Heading", resume_speed=100):
    """Read heading from IMU, stop if None, wait until valid, then resume."""
    value = get_heading(sensor)
    if value is None:
        stop()
        print(f"⚠️ {label} read failed. Waiting...")
        while value is None:
            value = get_heading(sensor)
            time.sleep(0.03)
        print(f"✅ {label} recovered: {value}°")
    forward(resume_speed)
    # time.sleep(0.05)
    return value

def led_on():
    GPIO.output(LED_PIN, GPIO.HIGH)
    
def led_off():
    GPIO.output(LED_PIN, GPIO.LOW)

def wait_for_start():
    print("Waiting for start button")
    while True:
        # wait until button is pressed (goes LOW)
        if GPIO.input(START_BUTTON_PIN) == GPIO.LOW:
            # Debounce check
            time.sleep(0.05)  # 50ms debounce delay
            if GPIO.input(START_BUTTON_PIN) == GPIO.LOW:
                print("Button pressed! Start robot...")
                break
        time.sleep(0.01) # Reduce CPU usage

    
# === Main Execution ===
if __name__ == '__main__':
    try:
        led_on()
        time.sleep(1)
        wait_for_start()
        time.sleep(1)
        led_off()

        Start_Time = 0
        End_Time = 0
        Start_Time = time.time()
        time.sleep(0.1)
        left_sensor = init_vl53l1x(LEFT_CHANNEL)
        time.sleep(0.1)
        center_sensor = init_vl53l1x(CENTER_CHANNEL)
        time.sleep(0.1)
        right_sensor = init_vl53l1x(RIGHT_CHANNEL)
        time.sleep(0.1)
        pca = init_pca9685()
        time.sleep(0.05)
        imu_sensor = init_bno055()
        #imu_sensor.mode = 0x08
        time.sleep(0.5)
        print("✅ Left, Rigth, Center and BNO55 are ready")

        base_heading = get_heading(imu_sensor)
        print(f"✅ Initial Heading: {base_heading}°")
        
        # Go straight until left or right distance > 120
        startsection = True
        turnDir = None
        turnCount = 1			# We are updating it form Trun 2

        FRONT_DISTANCE = 60			# Front Wall Distance required before trun in cm
        DC_Speed = 95				# DC Motor Speed
        set_servo_angle(STEERING_CENTER)
        forward(DC_Speed)  # Set desired speed
        
        integral = 0
        last_error = 0
        KP = 1.5 
        KI = 0.0
        KD = 0.0
        
        while True:
            distanceC = wait_for_distance(center_sensor, CENTER_CHANNEL, "Center")
        
            print(f"Center: {distanceC:.2f} cm")
            #print(f"Heading: {heading}°, Left: {distanceL:.2f} cm, Center: {distanceC:.2f} cm, Right: {distanceR:.2f} cm")
            
            if distanceC > FRONT_DISTANCE:
                if startsection is True:
                    heading   = wait_for_heading(imu_sensor)
                    gerror = (heading - base_heading + 180) % 360 - 180
                    gcorrection = int(gerror * 1.5 )
                    target_angle = STEERING_CENTER - gcorrection
                    set_servo_angle(target_angle)
                    #print(f"Heading: {heading}°, TargetAngle: {target_angle}°, Center: {distanceC:.2f} cm")
                    #print(f"TargetAngle: {target_angle}°")
                
                else:
                    if turnDir == "right":
                        distanceL = wait_for_distance(left_sensor, LEFT_CHANNEL, "Left")
                        error = TARGET_DISTANCE - distanceL
                        #print(f"Left Wall: {distanceL:.2f} cm")
                    
                    elif turnDir == "left":
                        distanceR = wait_for_distance(right_sensor, RIGHT_CHANNEL, "Right")
                        error = distanceR - TARGET_DISTANCE
                        #print(f"Right Wall: {distanceR:.2f} cm")
                    
                    # PID terms
                    #integral += error
                    #derivative = error - last_error
                    #correction = 1.7*error + KI*integral + KD*derivative
                    correction = 1.7*error
                    target_angle = STEERING_CENTER + int(correction)
                    set_servo_angle(target_angle)
                    #print(f"TargetAngle: {target_angle}°")
                    
        
            elif distanceC < FRONT_DISTANCE:
                stop()
                time.sleep(0.3)
                if startsection is True:
                    distanceL = wait_for_distance(left_sensor, LEFT_CHANNEL, "Left")
                    distanceR = wait_for_distance(right_sensor, RIGHT_CHANNEL, "Right")
                    if distanceL > 120:
                        turnDir = "left"
                        print("✅ Stop: Left is open. Turn direction = LEFT")
                        turnCount += 1
                        target_heading = decide_target_heading(turnCount, turnDir)
                        perform_forward_turn_to_heading(turnDir, target_heading, motor_speed=100)
                        time.sleep(0.3)
                        target_heading = decide_target_heading(turnCount, turnDir)
                        drive_straight_to_second(target_heading, duration=5)
                        distanceR = wait_for_distance(right_sensor, RIGHT_CHANNEL, "Right", 0)
                        TARGET_DISTANCE = distanceR
                        #TARGET_DISTANCE = max(18, min(distanceR, 30))
                        
                    
                    elif distanceR > 120:
                        turnDir = "right"
                        print("✅ Stop: Right is open. Turn direction = RIGHT")
                        turnCount += 1
                        target_heading = decide_target_heading(turnCount, turnDir)
                        perform_forward_turn_to_heading(turnDir, target_heading, motor_speed=100)
                        time.sleep(0.3)
                        target_heading = decide_target_heading(turnCount, turnDir)
                        drive_straight_to_second(target_heading, duration=5)
                        distanceL = wait_for_distance(left_sensor, LEFT_CHANNEL, "Left",  0)
                        TARGET_DISTANCE = distanceL
                        #TARGET_DISTANCE = max(18, min(distanceL, 30))

                    startsection = False
                    print(" Starting Section Complete...!")
                    #stop()
                    #time.sleep(5)
                    forward(DC_Speed)
                
                else:
                    if turnCount == 12:
                        stop()
                        print("12 Turns Complete! Driving straight with gyro for 3s...")
                        if turnDir == "left":
                            turnCount += 1
                            print(f"Section #{turnCount} Started...AntiClockwise")
                            target_heading = decide_target_heading(turnCount, turnDir)
                            perform_forward_turn_to_heading(turnDir, target_heading, motor_speed=100)
                            time.sleep(0.3)
                            target_heading = decide_target_heading(turnCount, turnDir)
                            drive_straight_to_second(target_heading, duration=3)
                            
                        elif turnDir == "right":
                            turnCount += 1
                            print(f"Section #{turnCount} Started...Clockwise")
                            target_heading = decide_target_heading(turnCount, turnDir)
                            perform_forward_turn_to_heading(turnDir, target_heading, motor_speed=100)
                            time.sleep(0.3)
                            target_heading = decide_target_heading(turnCount, turnDir)
                            drive_straight_to_second(target_heading, duration=3)

                        stop()
                        End_Time = time.time() - Start_Time
                        print("✅ Robot stopped after final straight")
                        print(f"End Time = {End_Time}")
                        break  # exit the loop

                    # Perform the turn and increment the counter
                    if turnDir == "left":
                        turnCount += 1
                        print(f"Section #{turnCount} Started...AntiClockwise")
                        target_heading = decide_target_heading(turnCount, turnDir)
                        perform_forward_turn_to_heading(turnDir, target_heading, motor_speed=100)
                        time.sleep(0.3)
                        target_heading = decide_target_heading(turnCount, turnDir)
                        drive_straight_to_second(target_heading, duration=5)
                        distanceR = wait_for_distance(right_sensor, RIGHT_CHANNEL, "Right", 0)
                        TARGET_DISTANCE = distanceR
                        print(f"Wall Distance Right: {TARGET_DISTANCE}")
                        #TARGET_DISTANCE = max(18, min(distanceR, 30))
                        
                        
                    elif turnDir == "right":
                        turnCount += 1
                        print(f"Section #{turnCount} Started...Clockwise")
                        target_heading = decide_target_heading(turnCount, turnDir)
                        perform_forward_turn_to_heading(turnDir, target_heading, motor_speed=95)
                        time.sleep(0.5)
                        target_heading = decide_target_heading(turnCount, turnDir)
                        drive_straight_to_second(target_heading, duration=5)
                        distanceL = wait_for_distance(left_sensor, LEFT_CHANNEL, "Left", 0)
                        TARGET_DISTANCE = distanceL
                        print(f"Wall Distance Left: {TARGET_DISTANCE}")
                        #TARGET_DISTANCE = max(18, min(distanceL, 30))

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Stopping motor and cleaning up GPIO")
        stop()
        motor_pwm.ChangeDutyCycle(0)
        time.sleep(0.5)
        GPIO.cleanup()
