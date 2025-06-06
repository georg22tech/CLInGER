#**********************************
#Name: Group_3_VideoTracking
#Author: Madeleine Kane (UoM ID: 10819933)
#Date edited: 27/04/2025
#Version: 7
#**********************************
#Based on code from pyimagesearch.com:
#Author: Adrian Rosebrock
#Avaliable: https://pyimagesearch.com/2018/07/30/opencv-object-tracking/
#***********************************


# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time
import cv2
import csv, os

current_dir = os.getcwd()

global tracker
global trackerFulc

#create and open csv file for x,y coords
# boost, mil, tld, csrt, med
f = open("final_continuum_2.csv", "w")
print('file created at {}/final_continuum_1.csv'.format(current_dir))
#set headers in file
f.write("x, y, xFulc, yFulc \n")
print("x, y, xFulc, yFulc \n")

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
	help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="kcf",
	help="OpenCV object tracker type")
args = vars(ap.parse_args())

# extract the OpenCV version info
(major, minor) = cv2.__version__.split(".")[:2]
# if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
# function to create our object tracker
if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(args["tracker"].upper())
	trackerFulc = cv2.Tracker_create(args["tracker"].upper())
# otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
# approrpiate object tracker constructor:
else:
    tracker_type=args.get("tracker")
    if tracker_type == 'BOOSTING': #problems with background
        tracker = cv2.legacy.TrackerBoosting_create()
        trackerFulc = cv2.legacy.TrackerBoosting_create()
    if tracker_type == 'MIL': #good but slow
        tracker = cv2.TrackerMIL_create()
        trackerFulc = cv2.TrackerMIL_create()
    if tracker_type == 'KCF': #doesn't work with csv
        tracker = cv2.TrackerKCF_create()
        trackerFulc = cv2.TrackerKCF_create()
    if tracker_type == 'TLD': #BAD
        tracker = cv2.legacy.TrackerTLD_create()
        trackerFulc = cv2.legacy.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW': #quick analysis and good result on tester
        tracker = cv2.legacy.TrackerMedianFlow_create()
        trackerFulc = cv2.legacy.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN': #needs additional files installed
        tracker = cv2.TrackerGOTURN_create()
        trackerFulc = cv2.TrackerGOTURN_create()
    if tracker_type == 'MOSSE': #doesn't work with csv
        tracker = cv2.legacy.TrackerMOSSE_create()
        trackerFulc = cv2.legacy.TrackerMOSSE_create()
    if tracker_type == "CSRT": #BAD
        tracker = cv2.TrackerCSRT_create()
        trackerFulc = cv2.TrackerCSRT_create()

# initialize the bounding box coordinates of the object we are going
# to track
initBB = None
initBBFulc = None

# if a video path was not supplied, grab the reference to the web cam
if not args.get("video", False):
	print("[INFO] starting video stream...")
	vs = VideoStream(src=0).start()
	time.sleep(1.0)
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])
# initialize the FPS throughput estimator
fps = None

# grab the current frame, then handle if we are using a
# VideoStream or VideoCapture object
frame = vs.read()
frame = frame[1] if args.get("video", False) else frame
#resize
frame = imutils.resize(frame, width=500)
(H, W) = frame.shape[:2]
# select the bounding box of the object we want to track and press ENTER or SPACE
initBB = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
initBBFulc = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
# start OpenCV object tracker using the supplied bounding box coordinates, then start the FPS throughput estimator as well
tracker.init(frame, initBB)
trackerFulc.init(frame, initBBFulc)
fps = FPS().start()
    
# loop over frames from the video stream
while True:
	# grab the current frame, then handle if we are using a
	# VideoStream or VideoCapture object
	frame = vs.read()
	frame = frame[1] if args.get("video", False) else frame
	# check to see if we have reached the end of the stream
	if frame is None:
		break
	# resize the frame (so we can process it faster) and grab the
	# frame dimensions
	frame = imutils.resize(frame, width=500)
	(H, W) = frame.shape[:2]
        # check to see if we are currently tracking an object
	if (initBB is not None) and (initBBFulc is not None):
		# grab the new bounding box coordinates of the object
		(success, box) = tracker.update(frame)
		(successFulc, boxFulc) = trackerFulc.update(frame)
		# check to see if the tracking was a success
		if success:
			(x, y, w, h) = [int(v) for v in box]
			cv2.rectangle(frame, (x, y), (x + w, y + h),
				(0, 255, 0), 2)
		if successFulc:
                        (xFulc, yFulc, wFulc, hFulc) = [int(v) for v in boxFulc]
                        cv2.rectangle (frame, (xFulc, yFulc),
                                       (xFulc + wFulc, yFulc + hFulc),(0, 255, 0), 2)
		#add to the csv file
		f.write(str(x) + ", " + str(y) +", " + str(xFulc) + ", " + str(yFulc) + "\n")
		print(str(x) + ", " + str(y) +", " + str(xFulc) + ", " + str(yFulc) + "\n")
		# update the FPS counter
		fps.update()
		fps.stop()
		# initialize the set of information we'll be displaying on
		# the frame
		info = [
			("Tracker", args["tracker"]),
			("Success", "Yes" if success and successFulc else "No"),
			("FPS", "{:.2f}".format(fps.fps())),
		]
		# loop over the info tuples and draw them on our frame
		for (i, (k, v)) in enumerate(info):
			text = "{}: {}".format(k, v)
			cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
				cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
# if we are using a webcam, release the pointer
if not args.get("video", False):
    vs.stop()
# otherwise, release the file pointer
else:
    vs.release()
# close all windows
cv2.destroyAllWindows()
#close the file
f.close()
