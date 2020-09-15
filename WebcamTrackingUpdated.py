#TITLE:                 WebcamTrackingUpdated.py
#AUTHORS:               Bradley Ward, Bernislain Momo
#DATE:                  6/25/2019
#DESCRIPTION:           Tracking an circular object using a Logitech Webcam C270 using color thresholding & contour data

#ADAPTED FROM:          CoolRunnings.py | ball_tracking.py (https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/)
#ORIGINAL AUTHORS:      Jan Herber, Harry Allen, Marius Thomas, Andre Beyer, Fabian Nitschke, Jovana Sierra | Adrian Rosebrock

from collections import deque
from imutils.video import VideoStream
from time import sleep
import numpy as np
import argparse
import cv2
import time
import imutils
import base64
import serial
import struct

##Constant/variable declarations
minArea 	        = 500		# minimal area of detected shapes
epsiFactor	        = 0.02		# factor for epsilon of the Ramer-Douglas-Peucker algorithm (std. 0.01) #so far 0.02 seems fine
hwcirc		        = 180		# maximum allowed deviation of the ellipse (circle picture) aspect ratio in percent
extellIpMin	        = 95		# maximum allowed deviation of the ellipse (circle picture) area in percent
epsixFactor	        = 0.014		# factor for epsilon of the Ramer-Douglas-Peucker algorithm (std. 0.01) #so far 0.02 seems fine
tempWRed                = 0             # temp value for width of red object (to avoid crashing due to loss of data)
tempHRed                = 0             # temp value for height of red object (to avoid crashing due to loss of data)
tempWGreen              = 0             # temp value for width of green object (to avoid crashing due to loss of data)
tempHGreen              = 0             # temp value for height of green object (to avoid crashing due to loss of data)
counter                 = 0             # frame counter
(dX, dY)                = (0, 0)        # puck position
(strikerX, strikerY)    = (0, 0)        # striker position
direction               = ""            # direction for striker to move in
enable                  = 0             # enables the motor movement after tracking the puck for a few frames
dYLastFrame             = 0             # saves the last Y position of the puck
dXLastFrame             = 0             # saves the last X position of the puck
initializeStriker       = 0             # allows the striker to be centered at the start up
strikerCenter           = 379           # center of striker based on Y position
pixelsPerStep           = .078          # number of pixels moved per step of stepper motor

#Exclusion range of !red
lower_red1 = np.array([0,110,70])       #Lower range of red
upper_red1 = np.array([10, 255,255])
lower_red2 = np.array([165,110,70])     #Upper range of red
upper_red2 = np.array([180, 255,255])

#Exclusion range of green
lower_green = np.array([20, 100, 100])  # Range of green
upper_green = np.array([70, 255,255])

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=32,
	help="max buffer size")
args = vars(ap.parse_args())

# initialize the list of tracked points, the frame counter,
# and the coordinate deltas
pts = deque(maxlen=args["buffer"])

ser = serial.Serial('COM4', baudrate = 9600, timeout = 1)
time.sleep(3.0)

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=0).start()
 
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])
 
# allow the camera or video file to warm up
time.sleep(2.0)

while True:
        # grab the current frame
        frame = vs.read()

        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if args.get("video", False) else frame

        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
                break

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=1080, height=4000)
    
        ##Image color filter
    
        #Converts image to the HSV colorspace and applies a blur to remove jagged edges that can effect the detection algorithms
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.medianBlur(hsv,3)
    
        #HSV covers two red ranges. bitwise_or() combines the two ranges into 1 picture 
        thresh_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),cv2.inRange(hsv, lower_red2, upper_red2))
        #python2: finds all edged objects and puts them into the contours_red object
        contours_red, h = cv2.findContours(thresh_red,1,2)

        #do the same for green
        contours_green, h = cv2.findContours(cv2.inRange(hsv, lower_green, upper_green),1,2)

        ##GREEN OBJECT DETECTION:

        for cnt in contours_green:    #Iterates through contours_red
        
                ##Basic edge analysis

                #Finds the number of edges within a specific arc-length range and returns it to approx
                approxGreen = cv2.approxPolyDP(cnt,epsiFactor*cv2.arcLength(cnt,True),True)
                #Calculates the area of the object
                areaGreen = cv2.contourArea(cnt)
                #Assigns the moment data of the object to M
                MGreen = cv2.moments(cnt)
            
                ##Object constraints

                #Does the object have a moment 
                if(MGreen['m00'] > 0):
                        #Does the object have at least 8 edges
                        if (len(approxGreen) >= 8 and (cv2.contourArea(cnt) >= minArea)):
                                #minxy = (x, y) #(minw, minh) = (minimum Width, minimum Heighth) #minAngle = angle of rotation
                                (minxy,(minwGreen,minhGreen),minAngleGreen) = cv2.fitEllipse(cnt)
                                if minwGreen != 0:
                                        tempWGreen = minwGreen
                                if minhGreen != 0:
                                        tempHGreen = minhGreen

                                if minwGreen != 0 and minhGreen != 0:
                                        #Area of the minimum ellipse
                                        minEnclAreaGreen = int((np.pi * minwGreen * minhGreen)/4)

                                else:
                                        #Area of the minimum ellipse
                                        minEnclAreaGreen = int((np.pi * tempWGreen * tempHGreen)/4)
                                #Area of the non-ellipse object
                                areaGreen = int(cv2.contourArea(cnt))                            

                                ##OBJECT TOLERANCE CHECK
                                #stresses the object's dimensions to determine if it is a circle
                                if( (100*areaGreen/minEnclAreaGreen) >= extellIpMin and (minhGreen*100/minwGreen) <= hwcirc and (minwGreen*100/minhGreen) <= hwcirc):
                                        cv2.drawContours(frame,[cnt],0,(0,255,0),2)

                                        # calculate x,y coordinate of center
                                        if MGreen["m00"] != 0:
                                                strikerX = int(MGreen["m10"] / MGreen["m00"])
                                                strikerY = int(MGreen["m01"] / MGreen["m00"])

                                                # if striker hasn't been centered intially
                                                if initializeStriker == 0:
                                                        initializeStriker = 1
                                                        # check if striker center is more than 3 pixels in either direction from center
                                                        if strikerY < strikerCenter - 2:
                                                                # calculate number of steps from the striker to the center
                                                                difference = strikerCenter - strikerY
                                                                numOfSteps = round((difference/pixelsPerStep)/32)
                                                                # put steps and direction into a struct
                                                                data = struct.pack('2B', numOfSteps, 0)
                                                                # write the data to the MSP430
                                                                ser.write(data)
                                                                print(ser.readline())
                                                                print(ser.readline())

                                                        if strikerY > strikerCenter + 2:
                                                                # calculate number of steps from the striker to the center
                                                                difference =  strikerY - strikerCenter
                                                                numOfSteps = round((difference/pixelsPerStep)/32)
                                                                # put steps and direction into a struct
                                                                data = struct.pack('2B', numOfSteps, 1)
                                                                # write the data to the MSP430
                                                                ser.write(data)
                                                                print(ser.readline())
                                                                print(ser.readline())
                                                        

                                        # draw a dot on the center of the striker based on its current position
                                        cv2.circle(frame, (strikerX, strikerY), 5, (255, 255, 255), -1)

                                        # save striker position if greater than zero, i.e. not detected
                                        if strikerX > 0:
                                                tempStriker = strikerX

        ##RED OBJECT DETECTION:

        for cntRed in contours_red:    #Iterates through contours_red
        
                ##Basic edge analysis

                #Finds the number of edges within a specific arc-length range and returns it to approx
                approxRed = cv2.approxPolyDP(cntRed,epsiFactor*cv2.arcLength(cntRed,True),True)
                #Calculates the area of the object
                areaRed = cv2.contourArea(cntRed)
                #Assigns the moment data of the object to M
                MRed = cv2.moments(cntRed)

                ##Object constraints

                #Does the object have a moment 
                if(MRed['m00'] > 0):
                        #Does the object have at least 8 edges
                        if (len(approxRed) >= 8 and (cv2.contourArea(cntRed) >= minArea)):
                                #minxy = (x, y) #(minw, minh) = (minimum Width, minimum Heighth) #minAngle = angle of rotation
                                (minxy,(minwRed,minhRed),minAngleRed) = cv2.fitEllipse(cntRed)
                                # store the height and width of the ellipse object if it isn't currently zero
                                if minwRed != 0:
                                        tempWRed = minwRed
                                if minhRed != 0:
                                        tempHRed = minhRed

                                # if data is available for both, then calculate using the non-temp values
                                if minwRed != 0 and minhRed != 0:
                                        #Area of the minimum ellipse
                                        minEnclAreaRed = int((np.pi * minwRed * minhRed)/4)
                                
                                # otherwise use temp values to find area
                                else:
                                        #Area of the minimum ellipse
                                        minEnclAreaRed = int((np.pi * tempWRed * tempHRed)/4)
                                        
                                #Area of the non-ellipse object
                                areaRed = int(cv2.contourArea(cntRed))                            

                                ##OBJECT TOLERANCE CHECK
                                #stresses the object's dimensions to determine if it is a circle
                                if( (100*areaRed/minEnclAreaRed) >= extellIpMin) and ((minhRed*100/minwRed) <= hwcirc) and ((minwRed*100/minhRed) <= hwcirc):
                                        cv2.drawContours(frame,[cntRed],0,(0,255,0),2)

                                        # calculate x,y coordinate of center
                                        if MRed["m00"] != 0:
                                                dX = int(MRed["m10"] / MRed["m00"])
                                                dY = int(MRed["m01"] / MRed["m00"])
                                                
                                                # check if there is two Y positions of the puck available to check and if a 1/3rd of a second has passed
                                                if (counter%10) == 0 and enable == 1:
                                                        # see if the puck has moved atleast 5 pixels in either direction
                                                        if(dY - dYLastFrame > 5) or (dY - dYLastFrame < -5):
                                                                # find the distance between the Y positions of the striker and the puck and determine the number of steps
                                                                difference = dY - strikerY
                                                                numOfSteps = round((difference/pixelsPerStep)/32)

                                                                # if the number of steps is negative, set direction to 1 
                                                                if numOfSteps < 0:
                                                                        print(abs(numOfSteps))
                                                                        # if the number of steps is greater than 2
                                                                        if abs(numOfSteps) > 2:
                                                                                # put steps and direction into a struct
                                                                                data = struct.pack('2B', abs(numOfSteps)-1, 1)
                                                                                # write the data to the MSP430
                                                                                ser.write(data)
                                                                                print(ser.readline())
                                                                                print(ser.readline())
                                                                                
                                                                # otherwise, set direction to 0        
                                                                else:
                                                                        print(numOfSteps)
                                                                        # if the number of steps is greater than 2
                                                                        if numOfSteps > 2:
                                                                                # put steps and direction into a struct
                                                                                data = struct.pack('2B', numOfSteps-1, 0)
                                                                                # write the data to the MSP430
                                                                                ser.write(data)
                                                                                print(ser.readline())
                                                                                print(ser.readline())

                                                # draw a dot in the center of the puck
                                                cv2.circle(frame, (dX, dY), 5, (255, 255, 255), -1)

                                                if (counter%10) == 0:
                                                        # enable the writing of data to the MSP430
                                                        enable = 1
                                                        # store the current X and Y position for the next position check
                                                        dYLastFrame = dY
                                                        dXLastFrame = dX
                                                        # reset the difference 
                                                        difference =0
                                
                                
                                        else:
                                                # if there is no moment data, set the X and Y position to zero
                                                dX, dY = 0, 0
                                        
        # check to see every 5 seconds if the striker to needs to be centered
        if (counter%150) == 0 and enable == 1 and initializeStriker == 1:
                #if dY == dYLastFrame:
                # check if the former X position is greater than the current X position
                if dX < dXLastFrame:
                        
                        # check if striker center is more than
                        # 3 pixels in either direction from center
                        if strikerY < strikerCenter - 2:
                                # calculate number of steps from
                                # the striker to the center
                                difference = strikerCenter - strikerY
                                numOfSteps = round((difference/pixelsPerStep)/32)
                                # put steps and direction into a struct
                                data = struct.pack('2B', numOfSteps, 0)
                                # write the data to the MSP430
                                ser.write(data)
                                print(ser.readline())
                                print(ser.readline())

                        if strikerY > strikerCenter + 2:
                                # calculate number of steps from the striker to the center
                                difference =  strikerY - strikerCenter
                                numOfSteps = round((difference/pixelsPerStep)/32)
                                # put steps and direction into a struct
                                data = struct.pack('2B', numOfSteps, 1)
                                # write the data to the MSP430
                                ser.write(data)
                                print(ser.readline())
                                print(ser.readline())

        # save striker position if greater than zero, i.e. not detected
        if strikerX == 0:
                tempStriker = strikerX


        cv2.putText(frame, "dX: {}, dY: {}".format(dX, dY),
		(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
		0.5, (255, 255, 255), 1)
        
        cv2.putText(frame, "strikerX: {}, strikerY: {}".format(strikerX, strikerY),
		(10, frame.shape[0] - 25), cv2.FONT_HERSHEY_SIMPLEX,
		0.5, (255, 255, 255), 1)

        cv2.putText(frame, "counter: {}".format(counter),
		(10, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX,
		0.5, (255, 255, 255), 1)

        # show the frame to our screen and increment the frame counter
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        
        counter += 1
        
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
                break

# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()
 
# otherwise, release the camera
else:
	vs.release()

ser.close()
 
# close all windows
cv2.destroyAllWindows()
