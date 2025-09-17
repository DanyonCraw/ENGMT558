import webbrowser
import cv2
import numpy as np
import serial
import time

#opens esp32 camera stream
url = "http://192.168.1.252:81/stream"
webbrowser.open_new(url)
cap = cv2.VideoCapture(url)


arduino = serial.Serial(port="COM4", baudrate=115200, timeout=1)

#coordiantes for cropped video perpendicular to the boat
y1, y2 = 824, 1024 #100, 612
x1, x2 = 590, 690  #0, 412
#lower limit for requred number of pixels required for colour to be considered detected
defult = 100
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    if arduino.in_waiting:
        print("Arduino:", arduino.readline().decode().strip())

    #crops video capture
    cropped_frame = frame[y1:y2, x1:x2]

    hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

    #colour limits for colour detection
    lower_red = np.array([0, 120, 120])
    upper_red = np.array([10, 255, 255])
   
 
    lower_blue = np.array([100, 120, 80])
    upper_blue = np.array([130, 255, 255])
 
    lower_yellow = np.array([20, 120, 120])
    upper_yellow = np.array([35, 255, 255])
 
    lower_green = np.array([45, 120, 120])
    upper_green = np.array([75, 255, 255])

    #colour masks. converts image to black and white where white indicates colour
    mask_red1 = cv2.inRange(hsv, lower_red, upper_red)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    #counts number of coloured (white) pixels in frame
    red_count = cv2.countNonZero(mask_red)
    blue_count = cv2.countNonZero(mask_blue)
    yellow_count = cv2.countNonZero(mask_yellow)
    green_count = cv2.countNonZero(mask_green)

    #finds the most dominat colour
    my_colours = [red_count, blue_count, yellow_count, green_count, defult]
    largest_number = max(my_colours)
    position = my_colours.index(largest_number)
    
    if position == 0:
        dominant_colour ="r"
    elif position == 1:
        dominant_colour  = "b"
    elif position == 2:
        dominant_colour  = "y"
    elif position == 3:
        dominant_colour  = "g"
    elif position == 4:
        dominant_colour  = "n"
        print("colour not detected")
    
    #sends colour to arduino
    if dominant_colour == "n":
        print(dominant_colour)
    else:
        print(dominant_colour)
        arduino.write(dominant_colour.encode())

    #show stream
    cv2.imshow("Test Stream", cropped_frame)
    cv2.imshow("Test Stream", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
