import cv2  # used to access camera vision
import numpy as np  #used for colour sensing
import serial
import time
import math # used for paralax adjustment 


cam_offset_x = 0   # cm
cam_offset_y = 0   # cm
cam_offset_z = 0   # cm positive is toward object
distance_to_target = 20  # cm
servo_x_angle = None   # used to track servo anlge (debugging)
servo_y_angle = None
last_servo_x_angle = None # used to prevent overflow of signals to arduino
last_servo_y_angle = None
CAMERA_MIN_X_ANGLE = 0   # the max and minimum limit for the servo angles based on the fov of the camera
CAMERA_MAX_X_ANGLE = 180
CAMERA_MIN_Y_ANGLE = 0
CAMERA_MAX_Y_ANGLE = 180 

def correct_parallax(raw_x_angle, raw_y_angle):
    # Convert to radians
    raw_x_rad = math.radians(raw_x_angle - 90)  # -90 so that 90° is center
    raw_y_rad = math.radians(raw_y_angle - 90)

    # Correct horizontal 
    corrected_x_rad = math.atan(
        math.tan(raw_x_rad) + (cam_offset_x / distance_to_target)
    )

    # Correct vertical
    corrected_y_rad = math.atan(
        math.tan(raw_y_rad) + (cam_offset_y / distance_to_target)
    )

    # Convert back to servo angle
    corrected_x_angle = int(math.degrees(corrected_x_rad) + 90)
    corrected_y_angle = int(math.degrees(corrected_y_rad) + 90)

    return corrected_x_angle, corrected_y_angle

def Camera_vision_max_x(lower, upper, value):
    return max(lower, min(upper, value))

def Camera_vision_max_y(lower, upper, value):
    return max(lower, min(upper, value))

# user input function to choose colour
def Colour_to_detect(hsv, colourname):
    if colourname == "red":
        lower_red_1 = np.array([0,120,70])
        upper_red_1 = np.array([10,255,255])
        lower_red_2 = np.array([170,120,70])
        upper_red_2 = np.array([180,255,255])

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)       # red wraps around HSV spetcrum and rewuires both limits/bounds
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        red_mask = mask1 + mask2
        return red_mask
    
    elif colourname == "green":                         #green bounds for HSV into an array 
        lower_green_1 = np.array([40,70,70])
        upper_green_1 = np.array([80,255,255])
        green_mask = cv2.inRange(hsv, lower_green_1, upper_green_1)
        return green_mask
    
    elif colourname == "blue":                      #blue bounds for HSV into an array 
        lower_blue_1 = np.array([100,150,0])
        upper_blue_1 = np.array([140,255,255])
        blue_mask = cv2.inRange(hsv, lower_blue_1, upper_blue_1)
        return blue_mask

    else:
        print(f"Colour '{colourname}' not recognized.")
        return np.zeros(hsv.shape[:2], dtype=np.uint8)  

print("what colour to detect:")
colourname = input()                            # what colour to detect
print("your colour is:" + colourname)
cap = cv2.VideoCapture(0)  # the number refers to whcib camera, 0 is built in 
if not cap.isOpened():
    print(" Error: Could not open webcam.")
    exit()


print("Connecting to Arduino...")
arduino = serial.Serial('COM7', 9600)
time.sleep(2)
print("Connected!")  #connect to arduino running at 9600 baud in port 7


while True:

    ret, frame = cap.read()
    if  not ret:
        print("camera read failed")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = Colour_to_detect(hsv,colourname) #creaet mask aof that choosen colur


    if frame is None or frame.size == 0:
        print("Invalid frame")
        continue
    if mask is None or mask.size == 0:
        print("Mask is empty — check color name or HSV values.")
        continue 

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    coordinates = cv2.findNonZero(mask)                     # remove haxe and clear up mask (glow around object etc)

    
    if coordinates is not None:
        avg = np.mean(coordinates, axis=0)[0]
        x, y = int(avg[0]) , int(avg[1])
        servo_x_angle = CAMERA_MIN_X_ANGLE + (x / frame.shape[1]) * (CAMERA_MAX_X_ANGLE - CAMERA_MIN_X_ANGLE)
        servo_y_angle = CAMERA_MIN_Y_ANGLE + (y / frame.shape[0]) * (CAMERA_MAX_Y_ANGLE - CAMERA_MIN_Y_ANGLE)#convert the pixels into angles and retrict those angle to the servo limits of 180 to 0
        servo_x_angle = max(CAMERA_MIN_X_ANGLE, min(CAMERA_MAX_X_ANGLE, servo_x_angle)) #setting limit based on how far the servo can turn
        servo_y_angle = max(CAMERA_MIN_Y_ANGLE, min(CAMERA_MAX_Y_ANGLE, servo_y_angle))      
        if servo_x_angle != last_servo_x_angle or servo_y_angle != last_servo_y_angle:
            try:
               # print("Per Parlax" , servo_y_angle, servo_x_angle)  #debugging 
                servo_x_angle, servo_y_angle = correct_parallax(servo_x_angle, servo_y_angle)  # correct the angle based on paralax
               # print("Paralax" , servo_y_angle, servo_x_angle)  # debugging
                arduino.write(f"{servo_x_angle},{servo_y_angle}\n".encode()) # send signal ro arduino 
                arduino.flush()                                                 # clean/clear the feed for arduino to prevent clogging the signal
                last_servo_x_angle = servo_x_angle
                last_servo_y_angle = servo_y_angle
                time.sleep(0.1)  # Slight delay to prevent spamming of signals
            except Exception as e:
                print("Serial write failed:", e)



        
        


    else:
        pass
        # print("No color detected in frame.")

    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('Original Frame', frame)
    # cv2.imshow(colourname + 'Mask Only', mask)
    cv2.imshow( colourname +'Highlighted', result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break  # pressing q breaks the loop anf allows for the next commands to run shutting it all down 
cap.release()
cv2.destroyAllWindows()