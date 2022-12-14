import apriltag
import cv2
import numpy as np

options = apriltag.DetectorOptions(families='tag36h11')
detector = apriltag.Detector(options)

feed = cv2.VideoCapture(0)

if not feed.isOpened():
	print("Cannot open camera.")
	exit()

while True: 

	ret, frame = feed.read()

	if not ret:
		print("Can't receive frame")
		break

	# TODO: Implement llpython
	llpython = [1,1,1,1,1,1,1,1]

	# Load the image and convert it to grayscale
	# print("[INFO] loading image...")
	cv2.imshow("Frame", frame)
	
	image = cv2.imread(frame)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# Detect AprilTags in the input image
	# print("[INFO] detecting AprilTags...")
	results = detector.detect(gray)
	# print("[INFO] {} total apriltags detected".format(len(results)))

	largestArea = 0
	returnContour = np.array([[0, 0], [0, 0], [0, 0], [0, 0]], dtype = np.int32)

	# loop over the AprilTag detection results
	for r in results:
		# extract the bounding box coordinates for the AprilTag
		# and convert each of the coordinate pairs to integers
		(ptA, ptB, ptC, ptD) = r.corners
		ptB = (int(ptB[0]), int(ptB[1]))
		ptC = (int(ptC[0]), int(ptC[1]))
		ptD = (int(ptD[0]), int(ptD[1]))
		ptA = (int(ptA[0]), int(ptA[1]))

		# draw the bounding box of the AprilTag detection
		cv2.line(image, ptA, ptB, (0, 255, 0), 2)
		cv2.line(image, ptB, ptC, (0, 255, 0), 2)
		cv2.line(image, ptC, ptD, (0, 255, 0), 2)
		cv2.line(image, ptD, ptA, (0, 255, 0), 2)
		# draw the center coordinates of the AprilTag
		(cX, cY) = (int(r.center[0]), int(r.center[1]))
		cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
		# draw the tag family on the image
		tagFamily = r.tag_family.decode("utf-8")
		cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		# print("[INFO] tag family: {}".format(tagFamily))

		# figure out contours?
		# contour = np.array([[ptA[0], ptA[1]], [ptB[0], ptB[1]], [ptC[0], ptC[1]], [ptD[0], ptD[1]]], dtype = np.int32)
		# contourArea = abs((ptA[0] - ptD[0])) * abs((ptA[1] - ptD[1]))

		# if contourArea > largestArea:
			# largestArea = contourArea
			# returnContour = contour

	if cv2.waitKey(1) & 0xFF == ord('q'): break

feed.release()
cv2.destroyAllWindows()
