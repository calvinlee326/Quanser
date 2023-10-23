# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports
"""
image_interpretation.py
Skills activity code for image interpretation lab guide.
Students will perform camera calibration along with line detection.
Please review Lab Guide - Image Interpretation PDF
"""
from pal.products.qcar import QCarCameras,QCarRealSense
from hal.utilities.image_processing import ImageProcessing
import time
import numpy as np
import cv2
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : ImageInterpretation Class Setup

class ImageInterpretation():

    def __init__(self,
            imageSize,
            frameRate,
            streamInfo,
            chessDims,
            boxSize):

        # Camera calibration constants:
        self.NUMBER_IMAGES = 15

        # List of variables given by students
        self.imageSize      = imageSize
        self.chessboardDim  = [chessDims,chessDims]
        self.frameRate      = frameRate
        self.boxSize        = boxSize
        self.sampleRate     = 1/self.frameRate
        self.calibFinished  = False

        # List of camera intrinsic properties :
        self.CSICamIntrinsics = np.eye(3, 3, dtype=np.float32)
        # CSI camera intrinsic matrix at resolution [820, 410] is:
        # [[318.86    0.00  401.34]
        #  [  0.00  312.14  201.50]
        #  [  0.00    0.00    1.00]]
        self.CSIDistParam = np.ones((1,5),dtype= np.float32)
        # CSI camera distorion paramters at resolution [820, 410] are:
        # [[-0.9033  1.5314 -0.0173 0.0080 -1.1659]]

        self.d435CamIntrinsics = np.eye(3,3,dtype= np.float32)
        # D435 RGB camera intrinsic matrix at resolution [640, 480] is:
        # [[455.20    0.00  308.53]
        #  [  0.00  459.43  213.56]
        #  [  0.00    0.00    1.00]]
        self.d435DistParam = np.ones((1,5), dtype= np.float32)
        # D435 RGB camera distorion paramters at resolution [640, 480] are:
        # [[-5.1135e-01  5.4549 -2.2593e-02 -6.2131e-03 -2.0190e+01]]

        # Final Image streamed by CSI or D435 camera
        self.streamD435 = np.zeros((self.imageSize[1][0],self.imageSize[1][1]))
        self.streaCSI = np.zeros((self.imageSize[0][0],self.imageSize[0][1]))

        # Information for interfacing with front CSI camera
        enableCameras = [False, False, False, False]
        enableCameras[streamInfo[0]] = True

        self.frontCSI = QCarCameras(
            frameWidth  = self.imageSize[0][0],
            frameHeight = self.imageSize[0][1],
            frameRate   = self.frameRate[0],
            enableRight = enableCameras[0],
            enableBack  = enableCameras[1],
            enableLeft  = enableCameras[2],
            enableFront = enableCameras[3]
        )

        # Information for interfacing with Realsense camera
        self.d435Color = QCarRealSense(
            mode=streamInfo[1],
            frameWidthRGB  = self.imageSize[1][0],
            frameHeightRGB = self.imageSize[1][1],
            frameRateRGB   = self.frameRate[1]
        )

        # Initialize calibration tool:
        self.camCalibTool = ImageProcessing()

    def camera_calibration(self):

        # saving images
        savedImages = []
        imageCount = 0
        cameraType = "csi"

        while True:
            startTime = time.time()

            # Read RGB information for front csi first, D435 rgb second
            if cameraType == "csi":
                self.frontCSI.readAll()
                endTime = time.time()
                image = self.frontCSI.csi[3].imageData
                computationTime = endTime-startTime
                sleepTime = self.sampleRate[0] \
                    - (computationTime % self.sampleRate[0])

            if cameraType == "D435":
                self.d435Color.read_RGB()
                endTime = time.time()
                image =self.d435Color.imageBufferRGB
                computationTime = endTime-startTime
                sleepTime = self.sampleRate[1] \
                    - (computationTime % self.sampleRate[1])

            # Use cv2 to display current image
            cv2.imshow("Camera Feed", image)

            msSleepTime = int(1000 * sleepTime)
            if  msSleepTime <= 0:
                msSleepTime = 1
            if cv2.waitKey(msSleepTime) & 0xFF == ord('q'):
                imageCount +=1
                print("saving Image #: ", imageCount)
                grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                savedImages.append(grayImage)

                if imageCount == self.NUMBER_IMAGES and cameraType == "csi":
                    print("Implement calibration for CSI camera images: ")

                    # ===== SECTION B1 - CSI Camera Parameter Estimation =====
                    print("Camera calibration for front csi")
                    self.CSICamIntrinsics = np.eye(3, 3, dtype=np.float32)
                    self.CSIDistParam = np.ones((1, 5), dtype=np.float32)

                    # Quanser's use of OpenCV for camera calibration
                    '''
                    self.CSICamIntrinsics, self.CSIDistParam = \
                        self.camCalibTool.calibrate_camera(
                            savedImages,
                            self.chessboardDim,
                            self.boxSize)
                    '''

                    # Printed output for students
                    text = "CSI camera intrinsic matrix at resolution {} is:"
                    print(text.format(self.imageSize[0][:]))
                    print(self.CSICamIntrinsics)

                    text = ("CSI camera distortion parameters "
                        + "at resolution {} are: ")
                    print(text.format(self.imageSize[0][:]))
                    print(self.CSIDistParam)

                    cameraType = "D435"
                    savedImages = []
                    imageCount = 0

                if imageCount == self.NUMBER_IMAGES and cameraType == "D435":
                    print("Implement calibration for "
                        + "realsense D435 camera images:")

                    # ===== SECTION B2 - D435 Camera Parameter Estimation =====
                    print("Camera calibration for  D435 RGB camera")
                    self.d435CamIntrinsics = np.eye(3, 3, dtype=np.float32)
                    self.d435DistParam = np.ones((1, 5), dtype=np.float32)

                    # Quanser's use of OpenCV for camera calibration
                    '''
                    self.d435CamIntrinsics, self.d435DistParam = \
                        self.camCalibTool.calibrate_camera(
                            savedImages,
                            self.chessboardDim,
                            self.boxSize
                        )
                    '''

                    # Printed output for students
                    text = ("D435 RGB camera intrinsic matrix "
                        + "at resolution {} is:")
                    print(text.format(self.imageSize[1][:]) )
                    print(self.d435CamIntrinsics)

                    text = ("D435 RGB camera distortion parameters"
                        + "at resolution {} are: ")
                    print(text.format(self.imageSize[1][:]))
                    print(self.d435DistParam)

                    # Use completed distortion correction on D435 image
                    imageShape = np.shape(image)
                    for count, distImg in enumerate(savedImages):
                        undist = self.camCalibTool.undistort_img(
                            distImg,
                            self.d435CamIntrinsics,
                            self.d435DistParam
                        )
                        cv2.imshow("RectifiedImages", undist)
                        cv2.waitKey(500)
                    break

        print("Both Cameras calibrated!")

        self.calibFinished = True
        cv2.destroyAllWindows()

    def line_detection(self, cameraType):

        startTime = time.time()

        while True:

            # Check which stream will be used for line detection:
            if cameraType == "csi":

                self.frontCSI.readAll()
                endTime = time.time()
                image = self.frontCSI.csi[3].imageData
                computationTime = endTime-startTime
                sleepTime = self.sampleRate[0] \
                    - (computationTime % self.sampleRate[0])
                cameraIntrinsics = self.CSICamIntrinsics
                cameraDistortion = self.CSIDistParam

            if cameraType == "D435":

                self.d435Color.read_RGB()
                endTime = time.time()
                image = self.d435Color.imageBufferRGB
                computationTime = endTime-startTime
                sleepTime = self.sampleRate[1] \
                    - (computationTime % self.sampleRate[1])
                cameraIntrinsics = self.d435CamIntrinsics
                cameraDistortion = self.d435DistParam

            # ============= SECTION C1 - Image Correction =============
            print("Implement image correction for raw camera image... ")
            undistortedImage = image


            # Quanser's use of OpenCV for distortion correction
            '''
            imageShape = np.shape(image)
            undistortedImage = self.camCalibTool.undistort_img(
                image,
                cameraIntrinsics,
                cameraDistortion
            )
            '''

            # ============= SECTION C2 - Image Filtering =============
            print("Implement image filter on distortion corrected image... ")
            filteredImage = image

            # Quanser's solution for image filtering for feature detection
            '''
            filteredImage = self.camCalibTool.do_canny(undistortedImage)
            # filteredImage = self.camCalibTool.doSobel(image)
            '''

            # ============= SECTION C3 - Feature Extraction =============
            print("Extract line information from filtered image... ")
            linesImage, lines = image, []

            # Quanser's solution for feature extraction
            '''
            linesImage, lines = self.camCalibTool.extract_lines(
                filteredImage,
                undistortedImage
            )
            '''

            print("Display image with lines found... ")
            imageDisplayed = image

            # Use cv2 to display current image
            cv2.imshow("Lines Image", imageDisplayed)
            msSleepTime = int(1000*sleepTime)
            if  msSleepTime <= 0:
                msSleepTime = 1

            cv2.waitKey(msSleepTime)

    def stop_cameras(self):
        # Stopping the image feed for both cameras
        # self.frontCSI.terminate()
        self.d435Color.terminate()

#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Main
def main():

    '''
    INPUTS:
    imageSize           = [[L,W],[L,W]] 2x2 array which specifies the
                            resultion for the CSI camera and the D435
    frameRate           = [CSIframeRate, D435frameRate] 2x1 array for
                            image frame rates for CSI, D435 camera
    streamInfo          = [CSIIndex,D435Stream] 2x1 array to specify the
                            CSI camera index and the D435 image stream
    chessDims           = Specify the number of cells of the square
                            chessboard used for calibration
    boxSize             = Float value to specify the size of the grids
                            in the chessboard. Be mindful of the physical
                            units being used
    '''

    # ======== SECTION A - Student Inputs for Image Interpretation ===========
    cameraInterfacingLab = ImageInterpretation(
        imageSize=[[820,410], [640,480]],
        frameRate=np.array([30, 30]),
        streamInfo=[3, "RGB"],
        chessDims=6,
        boxSize=1
    )

    ''' Students decide the activity they would like to do in the
    ImageInterpretation Lab

    List of current activities:
    - Calibrate   (interfacing skill activity)
    - Line Detect (line detection skill activity)
    '''
    camMode = "Calibrate"

    # ========= SECTION D - Camera Intrinsics and Distortion Coeffs. =========
    cameraMatrix  = np.array([
        [495.84,   0.00, 408.03],
        [0.00, 454.60, 181.21],
        [0.00,   0.00,   1.00]
    ])

    distortionCoefficients = np.array([
        -0.57513,
        0.37175,
        -0.00489,
        -0.00277,
        -0.11136
    ])

    if camMode == "Calibrate":
        try:
            cameraInterfacingLab.camera_calibration()
            if cameraInterfacingLab.calibFinished == True \
                    and camMode == "Calibrate":
                print("calibration process done, stopping cameras...")
                cameraInterfacingLab.stop_cameras()

        except KeyboardInterrupt:
            cameraInterfacingLab.stop_cameras()

    if camMode == "Line Detect":
        try:
            text = "Specify the camera used for line detection (csi/D435): "
            cameraType = input(text)
            if cameraType == "csi" :
                cameraInterfacingLab.CSICamIntrinsics = cameraMatrix
                cameraInterfacingLab.CSIDistParam     = distortionCoefficients
                cameraInterfacingLab.line_detection(cameraType)

            elif cameraType =="D435":
                cameraInterfacingLab.d435CamIntrinsics = cameraMatrix
                cameraInterfacingLab.d435DistParam     = distortionCoefficients
                cameraInterfacingLab.line_detection(cameraType)
            else:
                print("Invalid camera type")

        except KeyboardInterrupt:
            cameraInterfacingLab.stop_cameras()

    print("Camera Interfacing lab has been stopped....")
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Run
if __name__ == '__main__':
    main()
#endregion