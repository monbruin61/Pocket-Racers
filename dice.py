import cv2
import numpy as np
import RPi.GPIO as GPIO
from threading import Thread
from queue import Queue
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

class PiVideoStream:
    def __init__(self, resolution=(640, 480), framerate=32):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="bgr", use_video_port=True)
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False


    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self
    

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
        # return the frame most recently read
        return self.frame


    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream(resolution=(640,480)).start()
time.sleep(2.0)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7, GPIO.OUT)
pi_pwm = GPIO.PWM(7, 50)
pi_pwm.start(0)

def output_result(dot_count):
    global pi_pwm
    duty_cycles = range(0, 97, 8)
    try:
        pi_pwm.ChangeDutyCycle(duty_cycles[dot_count])
    except:
        pi_pwm.ChangeDutyCycle(0)

bdp_white = cv2.SimpleBlobDetector_Params()
''' UPDATE THESE PARAMETERS FOR YOUR WHITE DIE BLOB DETECTION '''
bdp_white = cv2.SimpleBlobDetector_Params()
bdp_white.filterByArea = True
bdp_whiteilterByConvexity = False
bdp_white.filterByCircularity = True
bdp_white.filterByInertia = False
bdp_white.filterByColor = True
bdp_white.blobColor = 0
bdp_white.minCircularity = 0.5
bdp_white.maxCircularity = 1
bdp_white.minArea = 5


detector_white = cv2.SimpleBlobDetector_create(bdp_white)

def white_dice(img):
    scaleX = .5
    scaleY = .5
    new_dims = (int(img.shape[1] * scaleX), int(img.shape[0] * scaleY))
    img = cv2.resize(img, new_dims)

    color_code = cv2.COLOR_BGR2GRAY
    img = cv2.cvtColor(img, color_code)

    img = cv2.GaussianBlur(img,ksize=(27,27),sigmaX=0)

    img = cv2.adaptiveThreshold(img, maxValue=255, adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C, thresholdType=cv2.THRESH_BINARY_INV, blockSize=33, C=10)

    #white_up = dilate
    #black_up = erode
    
    img = cv2.dilate(img, np.ones((7,7), np.uint8), iterations=1)
    img = cv2.erode(img, np.ones((7,7), np.uint8), iterations=1)
    img = cv2.dilate(img, np.ones((7,7), np.uint8), iterations=1)
    img = cv2.erode(img, np.ones((7,7), np.uint8), iterations=1)

    # img = cv2.dilate(img, np.ones((5,5), np.uint8), iterations=1)
    # img = cv2.erode(img, np.ones((5,5), np.uint8), iterations=1)
    # img = cv2.dilate(img, np.ones((5,5), np.uint8), iterations=1)
    # img = cv2.erode(img, np.ones((5,5), np.uint8), iterations=1)
    # img = cv2.dilate(img, np.ones((3,3), np.uint8), iterations=1)
    # img = cv2.erode(img, np.ones((3,3), np.uint8), iterations=1)
    cv2.imshow("Capture", img)
    points = detector_white.detect(img)

    # return img
    return points

bdp_color = cv2.SimpleBlobDetector_Params()
''' UPDATE THESE PARAMETERS FOR YOUR COLORED DICE BLOB DETECTION '''
bdp_color.filterByColor = True

detector_color = cv2.SimpleBlobDetector_create(bdp_color)

def nothing(val):
    pass


def colored_dice(img):
    scaleX = .6
    scaleY = .6
    new_dims = (int(img.shape[1] * scaleX), int(img.shape[0] * scaleY))
    img = cv2.resize(img, new_dims)


    blurred_img = cv2.GaussianBlur(img,ksize=(5,5),sigmaX=0)

    color_code = cv2.COLOR_BGR2HSV_FULL
    # bgr_rgb = cv2.COLOR_BGR2RGB

    # img = cv2.cvtColor(img, bgr_rgb)
    hsv = cv2.cvtColor(blurred_img, color_code)

    


    

    maskG = cv2.inRange(hsv, (90, 100, 0), (115, 255, 97))

    # maskR = cv2.inRange(hsv, (100, 0, 60), (170, 60, 170))

    # maskB = cv2.inRange(hsv, (115, 0, 80), (130, 255, 255))
    maskB = cv2.inRange(hsv, (113, 25, 0), (162, 255, 110))

    #combine the two masks
    mask = np.bitwise_or(maskG, maskB)







    # cv2.imshow("Green Mask", maskG)
    # # cv2.imshow("Red Mask", maskR)
    # cv2.imshow("Blue Mask", maskB)
    cv2.imshow("masked", mask)

    # cv2.imshow("Red + Green Mask", mask)


    points = detector_white.detect(mask)

    # return img
    return points


frame_count = 0
try:
    while True:
        result = vs.read()
        frame_count += 1
        img = cv2.rotate(result, cv2.ROTATE_180)

        ''' PART 1 '''

        ''' GENERAL FILTERING '''

        ''' WHITE DIE '''
        # points = white_dice(img)

        ''' COLORED DICE '''
        points = colored_dice(img)

        # Display the processed image with window title "Capture".
        # Test your filters by adding them directly to this while loop
        # (or creating a new function), then uncommenting the line below:
        
        # cv2.imshow("Capture", points)
        

        if frame_count % 3 == 0:
            # print(output_result(len(points)))
            print(len(points))

        k = cv2.waitKey(3)
        if k == ord('q'):
            # If you press 'q' in the OpenCV window, the program will stop running.
            break
        elif k == ord('p'):
            # If you press 'p', the camera feed will be paused until you press
            # <Enter> in the terminal.
            input()
except KeyboardInterrupt:
    pass

# Clean-up: stop running the camera and close any OpenCV windows
cv2.destroyAllWindows()
vs.stop()
