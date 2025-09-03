# ~ if pillar is too close then reverse add this in find pillar contour
# ~ Perfect Stop in Starting Section is not Done


import cv2
from picamera2 import Picamera2
from time import sleep
import RPi.GPIO as GPIO
import numpy as np
from fe_functionsV2 import *
import math
# ------------------------------{ function / class declarations }---------------------------------#
class Pillar:
    def __init__(self, area, dist, x, y, target): 
        self.area = area #pillar area
        self.dist = dist #pillar distance from bottom middle point of screen
        self.x = x #pillar x
        self.y = y #pillar y
        self.target = target #stores either target of green pillars or target of red pillarss
        self.w = 0
        self.h = 0
    
    def setDimentions(self, w, h):
        self.w = w
        self.h = h

#takes in contours, selects a pillar and returns its information in the format of a Pillar object
def find_pillar(contours, target, p, colour): 
    
    global turnDir, maxDist
    num_p = 0
    
    for cnt in contours: 
        area = cv2.contourArea(cnt)
        #if areas are large enough for the specific colour pillar
        # you can set this to avoid small areas or small missleading detections
        if (area > 1000 and colour == "red") or (area > 1200 and colour == "green"):
            
            #get width, height, and x and y coordinates by bounding rect
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
            x += ROI3[0] + w // 2 
            y += ROI3[1] + h
            #calculates the distance between the pillar and the bottom middle of the screen
            temp_dist = round(math.dist([x, y], [320, 480]), 0)
            
            #if the pillar is close enough add it to the number of pillars
            if 160 < temp_dist < 380: #180, 390
                num_p += 1
            
            #if this pillar is closer than the current pillar replace the current pillar and update its information
            if temp_dist < p.dist:
                p.area = area
                p.dist = temp_dist
                p.y = y
                p.x = x
                p.target = target
                p.setDimentions(w, h)

    return p, num_p

# === Safe shutdown handler ===
def stop_all(sig, frame):
    print("\n🛑 Emergency stop activated (Ctrl+C)")
    try:
        set_servo_angle(STEERING_CENTER)  # Center steering
        stop_car()                        # Stop DC motor
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("✅ Robot stopped safely.")
    except Exception as e:
        print(f"⚠️ Shutdown error: {e}")
    sys.exit(0)

# Attach signal handler for Ctrl+C
signal.signal(signal.SIGINT, stop_all)

if __name__ == '__main__':
    time.sleep(5)
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    time.sleep(2)
    
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
    #print("✅ Left, Rigth, Center, ServoPCA and BNO55 are ready")

    #base_heading = get_heading(imu_sensor)
    heading = get_heading(imu_sensor)
    while heading is None:
        time.sleep(0.05)  # very short wait
        heading = get_heading(imu_sensor)
    base_heading = heading
    
    print(f"✅ Initial Heading: {base_heading}°")
    
    #target X coordinates for red and green pillar
    redTarget = 110
    greenTarget = 530
    lapsComplete = False
    
    #variable that keeps track of the target of the last pillar the car has passed
    lastTarget = 0

    # tracks whether a pillar is in the first section right in front of the car,
    # used in three-point turn determination
    pillarAtStart = -1
    endConst = 30
    
    #Regions of Interest [x1,y1,x2,y2]
    ROI1 = [0, 175, 330, 265] #for finding left lane
    ROI2 = [330, 175, 640, 265] #for finding right lane
    ROI3 = [redTarget - 50, 120, greenTarget + 50, 345] #for finding signal pillars
    ROI4 = [200, 260, 440, 310] #for detecting blue and orange lines on mat
    ROI5 = [0, 0, 0, 0] #for turns at corners
    ROIs = [ROI1, ROI2, ROI3, ROI4, ROI5]
    
    kp = 1.0 #proportional value for PD steering 0.015
    kd = 0.01 #derivative value for PD steering 0.01
    
    straightConst = 95 				#angle in which car goes straight
    angle = 95 						#variable for the current angle of the car
    prevAngle = angle				#variable tracking the angle of the previous iteration
    
    speed = 100 #variable for initial speed of the car
    reverseSpeed = 80 #variable for speed of the car going backwards

    error = 0 #value containing the difference between a pillar's x coordinate and its target x coordinate
    prevError = 0 #stores previous error
    maxDist = 370 #no pillar can be detected if further than this value
    
    debug = False #boolean for debug mode
    
    #used to track time elapsed
    pTimer = time.time()
    start = False
    
    startSection = True 
    pillar_was_detected = False
    
    target_heading = 0
    turnDir = "none"	#storing the only direction the car is turning during the run
    #turnDir = "left"
    
    t = 1 				#tracks number of turns or Sections
    led_on()
    time.sleep(1)
    wait_for_start()
    led_off()
    time.sleep(1)
    set_servo_angle(pca, STEERING_CENTER)
    time.sleep(1)
    
    

#--------------------------------{ main loop }---------------------------------------#
    while True:
        fps_start = time.time()							#used in fps calculation
        
        img = picam2.capture_array()					#get an image from pi camera
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab) 	# convert from BGR to HSV
        img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0)	#perform Gaussian Blur
                
        #find contours of pillars
        contours_red = find_contours(img_lab, rRed, ROI3)
        contours_green = find_contours(img_lab, rGreen, ROI3)
        
        #count number of red and green pillars
        num_pillars_g = 0
        num_pillars_r = 0
        
        #make a temporary pillar object
        temp = Pillar(0, 1000000, 0, 0, greenTarget)

        #find pillar to navigate around
        cPillar, num_pillars_g = find_pillar(contours_green, greenTarget, temp, "green")
        cPillar, num_pillars_r = find_pillar(contours_red, redTarget, cPillar, "red")

        # Draw contours
        cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], contours_green, -1, (0, 255, 0), 2)
        cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], contours_red, -1, (0, 0, 255), 2)
        
        heading = get_heading(imu_sensor)
        if heading is None:
            time.sleep(0.05)
            continue
        
        if t==13 and not lapsComplete:
            lapsComplete = True
            led_off()
            stop_car()
            print("All Laps Completed...")
            break
        
        
#---------------------------{ pillar detected }---------------------------#
            
        if cPillar.area > 0 and not lapsComplete:
            # pillar is too close so reverse car
            if cPillar.y > 340:
                set_servo_angle(pca, STEERING_CENTER)
                reverse(60)
                time.sleep(1)
            # pillar at perfect distance to avoid it
            elif cPillar.y > 300:
                # if pillar in starting section so avoid parking lot
                pillar_was_detected = True
                if cPillar.target == greenTarget:
                    print(f"Geen Pillar,> area: {cPillar.area},> Y: {cPillar.y}")
                elif cPillar.target == redTarget:
                    print(f"Red Pillar,> area: {cPillar.area},> Y: {cPillar.y}")
                
                if t in (1,5,9,13) or startSection is True:
                    print("Start Section So Tight Turning")
                    if turnDir == "none":
                        if cPillar.target == greenTarget:
                            left_lrrl(pca, imu_sensor, 32)
                        elif cPillar.target == redTarget:
                            right_rllr(pca, imu_sensor, 32)
                    if turnDir == "right":
                        if cPillar.target == greenTarget:
                            left_lrrl(pca, imu_sensor, 32)
                        elif cPillar.target == redTarget:
                            right_rllr(pca, imu_sensor, 50)      
                    elif turnDir == "left":
                        if cPillar.target == greenTarget:
                            left_lrrl(pca, imu_sensor, 50)
                        elif cPillar.target == redTarget:
                            right_rllr(pca, imu_sensor, 32)    
                else:
                    if cPillar.target == greenTarget:
                        left_lrrl(pca, imu_sensor, 47)
                    elif cPillar.target == redTarget:
                        right_rllr(pca, imu_sensor, 47)
                
#-------------------------{ No pillar detected }-------------------------#
        elif cPillar.area == 0 and not lapsComplete:
            
            distanceC = get_distance(center_sensor, CENTER_CHANNEL)
            
            if distanceC is not None and distanceC < 100 and startSection is False:
                stop_car()
                print(f"Obstacle detected at {distanceC:.2f} cm — stopping.")
                
                target_heading = decide_target_heading(t, turnDir)
                drive_straight_until_obstacle(pca, imu_sensor, target_heading, center_sensor, stop_distance=27)
            
                perform_reverse_turn_to_heading(pca, imu_sensor, turnDir, target_heading, motor_speed=100)
                t += 1
                target_heading = decide_target_heading(t, turnDir)
                drive_straight_to_second(pca, imu_sensor, target_heading, duration=1)
                
            elif distanceC is not None and distanceC < 100 and startSection is True:
                
                target_heading = base_heading
                drive_straight_until_obstacle(pca, imu_sensor, target_heading, center_sensor, stop_distance=27)
                time.sleep(1)
                distanceL = get_distance(left_sensor, LEFT_CHANNEL)
                distanceR = get_distance(right_sensor, RIGHT_CHANNEL)
                if distanceL > 120:
                    turnDir = "left"
                    print("✅ Stop: Left is open. Turn direction = LEFT")
                elif distanceR > 120:
                    turnDir = "right"
                    print("✅ Stop: Right is open. Turn direction = RIGHT")
                    
                perform_reverse_turn_to_heading(pca, imu_sensor, turnDir, target_heading, motor_speed=100)
                t += 1
                target_heading = decide_target_heading(t, turnDir)
                drive_straight_to_second(pca, imu_sensor, target_heading, duration=1)
                startSection = False
        
        
        heading = get_heading(imu_sensor)
        while heading is None:
            time.sleep(0.05)  # very short wait
            heading = get_heading(imu_sensor)
        
        if startSection is True:
            target_hrading = base_heading
        else:
            target_heading = decide_target_heading(t, turnDir)
            
        error = (heading - target_heading + 180) % 360 - 180
        correction = kp * error
        set_servo_angle(pca, int(STEERING_CENTER - correction))
        forward(speed)
        
        print(f">>>>>>>Debug, TurnDir: {turnDir}, Turn: {t}, Target: {target_heading:.1f}°, Error: {error:.1f}°, ")
        
        """
        if not start and not lapsComplete: 
            time.sleep(3)
            set_servo_angle(pca, STEERING_CENTER)
            forward(speed)
            print("Robot Started Forward.")
            start = True
            
            #if the pillar is large enough and the x-coordinate is close enough to its target set lastTarget to the current pillars target 
            if cPillar.target == greenTarget and cPillar.x > 320 and cPillar.area > 1000:
                lastTarget = greenTarget

            elif cPillar.target == redTarget and cPillar.x < 320 and cPillar.area > 1000:
                lastTarget = redTarget
              
            error = cPillar.x - cPillar.target
            angle = int(straightConst + error * cKp + (error - prevError) * cKd)
            angle = max(0, angle) #make sure angle value is over 0
        """
        

        # Show frame
        cv2.imshow("Red & Green Contours", img)    
            
        # Keep ~30 fps
        #while int(1 / (time.time() - fps_start)) > 30:
            #pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()
    GPIO.cleanup()