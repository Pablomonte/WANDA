# PiCamera Viewer Script
# This script creates a GUI application for controlling and viewing a Raspberry Pi camera
# with features specifically designed for astrophotography and telescope control.
# It includes functionality for image capture, processing, and motor control for telescope movement.

# Import required libraries
import tkinter as tk          # For creating the GUI interface
from tkinter import filedialog # For file dialog operations
from PIL import Image, ImageTk # For image processing and display in tkinter
import time                   # For timing and delays
import datetime              # For timestamp operations
import cv2                    # OpenCV for image processing
import numpy as np           # For numerical operations and array handling
import subprocess           # For running system commands
import threading            # For running multiple operations concurrently
import RPi.GPIO as GPIO     # For controlling Raspberry Pi GPIO pins

# Set up GPIO mode to use Broadcom (BCM) pin numbering
# This is important for correctly identifying the GPIO pins on the Raspberry Pi
GPIO.setmode(GPIO.BCM)

# Configure GPIO pins for two stepper motors
# First stepper motor (Right Ascension/RA motor) - Controls east-west movement
# These pins will be used in sequence to control the motor steps
GPIO.setup(6,  GPIO.OUT)   # Phase 1 of RA motor
GPIO.setup(13, GPIO.OUT)   # Phase 2 of RA motor
GPIO.setup(19, GPIO.OUT)   # Phase 3 of RA motor
GPIO.setup(26, GPIO.OUT)   # Phase 4 of RA motor

# Second stepper motor (Declination/DEC motor) - Controls north-south movement
GPIO.setup(12, GPIO.OUT)   # Phase 1 of DEC motor
GPIO.setup(16, GPIO.OUT)   # Phase 2 of DEC motor
GPIO.setup(20, GPIO.OUT)   # Phase 3 of DEC motor
GPIO.setup(21, GPIO.OUT)   # Phase 4 of DEC motor

# Function to reset camera exposure settings
def ResetCamera():
    global ExpSec, ExpMicSec
    # Convert exposure time from seconds to microseconds
    ExpMicSec = int(ExpSec * 1000000)
    
# Initialize default exposure settings
ExpSec = 0.25  # Default exposure time in seconds
ExpMicSec = int(ExpSec * 1000000)  # Convert to microseconds for camera command

# Configure camera's defective pixel correction (DPC)
# DPC helps improve image quality by handling defective pixels on the sensor
# Options available:
# 0 = Disabled (no correction)
# 1 = Mapped DPC (uses a map of known defective pixels)
# 2 = Dynamic DPC (detects and corrects defective pixels in real-time)
# 3 = Both Mapped and Dynamic DPC
StrCommand = "sudo vcdbg set imx477_dpc 3"  # Enable both types of DPC
try:
    res = subprocess.check_call(StrCommand, shell=True)
    print ("DPC is Set")
except:
    print ("DPC is NOT Set")

# Capture 2 initial images at startup to ensure camera is working
# Configure raspistill command with various parameters:
# -md 3: Sets camera mode 3 (specific resolution)
# -ex off: Turns off auto exposure
# -awb off: Turns off auto white balance
# -awbg 1.6,1.7: Sets manual white balance gains
# -drc off: Disables dynamic range compression
# -st: Enables statistics
# -t 60: Sets timeout to 60ms
# -bm: Enables burst mode
# -r: Enables raw capture
# -n: Disables preview window
# -q 70: Sets JPEG quality to 70%
Raspistill = 'raspistill -md 3 -ex off -awb off -awbg 1.6,1.7 -drc off -st -t 60 -bm -r -n -q 70 '
AnalogGain = 16  # Initial analog gain setting for the camera

# Capture first initialization image
StrCommand = Raspistill + ' -o temp/capture1.jpg -ss ' + str(ExpMicSec) + ' -ag ' + str(AnalogGain)
dt_now = datetime.datetime.now()
StrCapture = "Pre Capture 1 at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
print(StrCapture)
try:
    res = subprocess.check_call(StrCommand, shell=True)
except:
    print ("Capture Error")

# Capture second initialization image
# Two images are captured to ensure stable camera operation
StrCommand = Raspistill + ' -o temp/capture2.jpg -ss ' + str(ExpMicSec) + ' -ag ' + str(AnalogGain)
dt_now = datetime.datetime.now()
StrCapture = "Pre Capture 2 at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
print(StrCapture)
try:
    res = subprocess.check_call(StrCommand, shell=True)
except:
    print ("Capture Error")

# Set up white balance gains for image processing
WBGR = 1.4  # Red channel gain
WBGG = 1.2  # Green channel gain
WBGB = 1.5  # Blue channel gain
WBGL = 1.0  # Overall luminance gain
# Format white balance options for dcraw command
WBGOption = '-r %4.3f %4.3f %4.3f %4.3f' % (WBGR, WBGG, WBGB, WBGL)

# Move first capture to working file for processing
StrCommand = 'mv temp/capture1.jpg temp/capture.jpg'
res = subprocess.check_call(StrCommand, shell=True)
dt_now = datetime.datetime.now()
StrCapture = "Tif pre-conversion at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
print(StrCapture)

# Convert RAW image to TIFF using dcraw
# -T: Write TIFF instead of PPM
# -4: 16-bit linear output
# -q 0: Read camera pixel values with no interpolation
StrCommand = 'dcraw -T -4 -q 0 temp/capture.jpg'
res = subprocess.check_call(StrCommand, shell=True)

# Initialize image processing arrays and parameters
# Read the initial TIFF image
FrameImage = cv2.imread('temp/capture.tiff', -1)  # -1 flag reads image unchanged
# Create empty array for image stacking
StackImage = np.array(FrameImage*0, dtype = np.float32)

# Set up sensor and display parameters
SensorW = 4056  # Sensor width in pixels
SensorH = 3040  # Sensor height in pixels
xZoomCenter = int(SensorW/2)  # Initial X center for zoom window
yZoomCenter = int(SensorH/2)  # Initial Y center for zoom window
ZoomWindowHW = 128  # Half-width of zoom window
# Create initial cropped image for zoom display
CropImage = FrameImage[yZoomCenter-ZoomWindowHW:yZoomCenter+ZoomWindowHW, 
                      xZoomCenter-ZoomWindowHW:xZoomCenter+ZoomWindowHW]
# Process zoom image for display (scale and clip values)
ZoomImage = (CropImage/16).clip(2,255).astype(np.uint8)

# Initialize camera operation flags
fRunCamera = 1  # Main camera operation flag
fCapture = 2    # Current capture state
fCapRead = 2    # Current read state
fImageReady = 0 # Image ready for processing flag

# Define thread function for continuous camera capture
def RunCamera():
    """
    Thread function that handles continuous camera capture.
    Alternates between two capture files to avoid conflicts
    while reading and writing images.
    """
    global fRunCamera, fCapture, fImageReady
    global ExpMicSec, Raspistill, fCapRead, AnalogGain
    while (fRunCamera):
        if(fCapture == fCapRead):  # Only capture when previous image has been read
            # Alternate between two capture files
            if(fCapture == 1):
                StrCommand = Raspistill + ' -o temp/capture2.jpg -ss ' + str(ExpMicSec) + ' -ag ' + str(AnalogGain)
            elif(fCapture == 2):
                StrCommand = Raspistill + ' -o temp/capture1.jpg -ss ' + str(ExpMicSec) + ' -ag ' + str(AnalogGain)
            try:
                res = subprocess.check_call(StrCommand, shell=True)
            except:
                print ("Capture Error")
                return

            # Log capture time
            dt_now = datetime.datetime.now()
            StrCapture = "CAMERA " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
            print("%s Capture%d" % (StrCapture, fCapture))
            
            # Toggle capture state
            fCapture += 1
            if(fCapture > 2):
                fCapture = 1
            time.sleep(0.1)
        time.sleep(0.1)
    print("Camera Stopped")

# Start the camera capture thread
t1 = threading.Thread(target=RunCamera)
t1.start()

# Define thread function for RAW image conversion
def ConvRaw():
    """
    Thread function that handles RAW to TIFF conversion.
    Works in conjunction with the camera capture thread to process images
    as they are captured.
    """
    global fRunCamera, fCapture, fImageReady, fCapRead
    global ExpMicSec, WBGOption
    while (fRunCamera):
        if(fCapture == fCapRead):
            # No new image to process
            time.sleep(0.1)
            continue
        elif(fImageReady == 0):
            fCapRead = fCapture
            # Move the appropriate capture file to working file
            if(fCapRead == 1):
                StrCommand = 'mv temp/capture1.jpg temp/capture.jpg'
                res = subprocess.check_call(StrCommand, shell=True)
            elif(fCapRead == 2):
                StrCommand = 'mv temp/capture2.jpg temp/capture.jpg'
                res = subprocess.check_call(StrCommand, shell=True)
            
            # Convert RAW to TIFF using dcraw
            StrCommand = 'dcraw -T -4 -q 0 temp/capture.jpg'
            try:
                res = subprocess.check_call(StrCommand, shell=True)
            except:
                print ("Conversion Error")
                return
            
            # Log conversion completion time
            dt_now = datetime.datetime.now()
            StrCapture = "TIFF   " + dt_now.strftime('%Y-%m-%d %H:%M:%S') + " Converted"
            print(StrCapture)
            
            fImageReady = 1
        time.sleep(0.1)
    print("Camera Stopped")

# Start the RAW conversion thread
t2 = threading.Thread(target=ConvRaw)
t2.start()

# Set up the main GUI window
window = tk.Tk()  # Create the main window
window.wm_title("PiCamera Viewer")  # Set window title
window.configure(background='#222222')  # Set dark background
gray_default = window.cget("background")  # Get default background color for consistency

# Define colors for UI elements
ColorRED = '#A00'    # Red for warnings/errors
ColorGREEN = '#070'  # Green for success/active states
ColorBLUE = '#00A'   # Blue for normal states
ColorBLUE2 = '#338'  # Alternative blue for toggles

# Create and configure the main image display frame
imageFrame = tk.Frame(window, width=1280, height=720, 
                     bg=gray_default,
                     highlightbackground=gray_default)
imageFrame.grid(row=0, column=0, rowspan=3, padx=2, pady=2)
lmain = tk.Label(imageFrame, bg=gray_default)
lmain.grid(row=0, rowspan=10, column=1, columnspan=3)

# Initialize display image processing
# Resize and clip the initial frame for display
DisplayImage = cv2.resize((FrameImage/16).clip(0,255).astype(np.uint8), 
                         (int(SensorW/5), int(SensorH/5)))

# Create separate color channel arrays for display processing
DisplayR = np.array(DisplayImage, dtype=np.float32)
DisplayR = DisplayR*0  # Initialize to zero
DisplayG = np.array(DisplayR, dtype=np.float32)
DisplayB = np.array(DisplayR, dtype=np.float32)

# Set up color channel masks
DisplayB[:,:,0] = 1.0  # Blue channel
DisplayG[:,:,1] = 1.0  # Green channel
DisplayR[:,:,2] = 1.0  # Red channel

# Convert the initial image for display
RGBImage = cv2.cvtColor(DisplayImage.astype(np.uint8), cv2.COLOR_RGB2BGR)
img = Image.fromarray(RGBImage)
imgtk = ImageTk.PhotoImage(image=img)
lmain.imgtk = imgtk
lmain.configure(image=imgtk)

# Create and configure the zoom window frame
zoomFrame = tk.Frame(window, width=ZoomWindowHW*2, height=ZoomWindowHW*2, 
                    bg=gray_default)
zoomFrame.grid(row=0, column=1, padx=2, pady=2)
zoomImage = tk.Label(zoomFrame, bg=gray_default)
zoomImage.grid(row=0, column=0, columnspan=4)

# Set up initial zoom window display
RGBImage = cv2.cvtColor(ZoomImage, cv2.COLOR_RGB2BGR)
img2 = Image.fromarray(RGBImage)
imgtk2 = ImageTk.PhotoImage(image=img2)
zoomImage.imgtk = imgtk2
zoomImage.configure(image=imgtk2)

# Threshold control setup
fThreshold = 0  # Flag for threshold mode
vThreshold = 128  # Default threshold value

def ChangeThres(vThres):
    """
    Callback function for threshold value changes.
    Updates the threshold value and triggers zoom update if threshold mode is active.
    """
    global vThreshold, fThreshold
    vThreshold = vThres
    if(fThreshold==1):
        global fRunZoomUpdate
        fRunZoomUpdate=1

# Create and configure threshold controls
ThresLabel = tk.Label(zoomFrame, text="Threshold", bg=gray_default, fg='white')
ThresLabel.grid(row=1,column=0,sticky='ew')
ThresScale = tk.Scale(zoomFrame, orient='horizontal', command=ChangeThres,
                      from_=10, to=250, bg=gray_default, fg='white',
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
ThresScale.set(vThreshold)
ThresScale.grid(row=1,column=1,sticky='ew')

# Zoom attenuation control
vZAtten = 16.0  # Default zoom attenuation value

def ChangeZAtten(ZoomAttenuation):
    """
    Callback function for zoom attenuation changes.
    Adjusts the brightness of the zoom window display.
    """
    global vZAtten
    vZAtten = 10 ** float(0.024082*int(ZoomAttenuation))
    UpdateZoom()

# Create and configure zoom attenuation controls
ZAttenLabel = tk.Label(zoomFrame, text="Atten", bg=gray_default, fg='white')
ZAttenLabel.grid(row=1,column=2,sticky='ew')
ZAttenScale = tk.Scale(zoomFrame, orient='horizontal', command=ChangeZAtten,
                      from_=0, to=100, resolution=2, bg=gray_default, fg='white',
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
ZAttenScale.set(50)
ZAttenScale.grid(row=1,column=3,sticky='ew')

def Threshold():
    """
    Toggle function for threshold mode.
    Switches between normal and threshold view in zoom window.
    """
    global fThreshold
    if(fThreshold==1):
        fThreshold = 0
        ThresButton.configure(bg=gray_default)
    else:
        fThreshold =1
        ThresButton.configure(bg=ColorBLUE2)
    global fRunZoomUpdate
    fRunZoomUpdate=1
    UpdateZoom()

# Create threshold toggle button
ThresButton=tk.Button(zoomFrame, text="Threshold", command=Threshold,
                      bg=gray_default, fg='white')
ThresButton.grid(row=2,column=0,columnspan=1,sticky="ew")

# Tracking control setup
fTrack = 0  # Flag for tracking mode
def TrackToggle():
    """
    Toggle function for star tracking mode.
    Initializes tracking when enabled and resets when disabled.
    """
    global fTrack,fBaseSet
    if(fTrack==1):
        fTrack = 0
        fBaseSet = 0
        TrackButton.configure(bg=gray_default)
    else:
        DetectShift()
        fTrack =1
        TrackButton.configure(bg=ColorBLUE2)

# Create tracking toggle button
TrackButton=tk.Button(zoomFrame, text="Track", command=TrackToggle,
                      bg=gray_default, fg='white')
TrackButton.grid(row=2,column=1,columnspan=1, sticky="ew")

# Dark frame subtraction setup
fDark = 0  # Flag for dark frame subtraction
DarkLevel = 255  # Default dark level
DarkFileName = ""  # Path to dark frame file
DarkFileType = [("Dark File", "*.tif")]  # File type filter for dark frames
DarkImage = np.float32(FrameImage*0)  # Initialize dark frame array

def DarkToggle():
    """
    Toggle function for dark frame subtraction.
    Handles loading and processing of dark frame files.
    """
    global fDark, DarkFileName, DarkImage, DarkLevel
    if(fDark==1):
        fDark = 0
        DarkButton.configure(bg=gray_default)
    else:
        # Open file dialog to select dark frame
        DarkFileName = filedialog.askopenfilename(initialdir='~/Pictures', 
                                                filetypes=DarkFileType)
        # Load and process dark frame
        DarkImage = cv2.imread(DarkFileName, -1)  # unchanged
        MinNoise = np.min(DarkImage)
        DarkImage -= MinNoise
        MinNoise = np.min(DarkImage)
        MaxNoise = np.max(DarkImage)
        print("DARK-Array    Min=%f, Max=%f" % (MinNoise,MaxNoise))
        fDark = 1
        DarkButton.configure(bg=ColorBLUE2)

# Create dark frame toggle button
DarkButton=tk.Button(zoomFrame, text="Dark", command=DarkToggle,
                      bg=gray_default, fg='white')
DarkButton.grid(row=2,column=3,columnspan=1, sticky="ew")

# Control frame setup for main settings
sliderFrame = tk.Frame(window, width=400, height=600, bg=gray_default)
sliderFrame.grid(row=1, rowspan=3, column=1, padx=2, pady=2)

# Analog gain control setup
def ChangeAGain(AGain):
    """
    Callback function for analog gain changes.
    Updates the camera's analog gain setting.
    """
    global AnalogGain
    AnalogGain = AGain

# Create and configure analog gain controls
AGainLabel = tk.Label(sliderFrame, text="Analog Gain", bg=gray_default, fg='white')
AGainLabel.grid(row=0,column=0,sticky='ew')
AGainScale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeAGain,
                      from_=1, to=16, bg=gray_default, fg='white',
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
AGainScale.set(AnalogGain)
AGainScale.grid(row=0,column=1,sticky='ew')

# Stack size control setup
MStack = 64  # Default maximum stack size
def ChangeMStack(MaxStack):
    """
    Callback function for maximum stack size changes.
    Controls how many images will be stacked before auto-saving.
    """
    global MStack
    MStack = MaxStack

# Create and configure stack size controls
MStackLabel = tk.Label(sliderFrame, text="MaxStack", bg=gray_default, fg='white')
MStackLabel.grid(row=0,column=2,sticky='ew')
MStackScale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeMStack,
                      from_=16, to=128, bg=gray_default, fg='white',
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
MStackScale.set(MStack)
MStackScale.grid(row=0,column=3,sticky='ew')

# Image update control
fRunImageUpdate = 0  # Flag for continuous image updating
def ExposToggle():
    """
    Toggle function for continuous image capture and processing.
    Controls whether the camera is actively capturing new frames.
    """
    global ExpSec,fRunImageUpdate
    if(fRunImageUpdate==1):
        fRunImageUpdate = 0
        ExposButton.configure(bg=gray_default)
    else:
        fRunImageUpdate = 1
        ExposButton.configure(bg=ColorBLUE2)
        ResetCamera()

# Create capture toggle button
ExposButton = tk.Button(sliderFrame, text="Capture", command=ExposToggle, 
                       bg=gray_default, fg='white')
ExposButton.grid(row=5,column=0, sticky="ew")

# Exposure control setup
ExpSecStr = tk.StringVar()  # String variable for exposure display
PowerOf2 = -2  # Initial power of 2 for exposure time

def ChangeExpos(PowOf2):
    """
    Callback function for exposure time changes.
    Calculates exposure time as a power of 2 and updates display.
    """
    global ExpSec,ExpMicSec
    ExpSec = 2 ** float(PowOf2)  # Calculate exposure time in seconds
    ExpMicSec = int(ExpSec * 1000000)  # Convert to microseconds
    ExpSecStr.set('%1.3f' % ExpSec)  # Update display string
    ExpSecLabel.configure(textvariable=ExpSecStr)

# Create and configure exposure controls
ExposLabel = tk.Label(sliderFrame, text="Exposure[s]", bg=gray_default, fg='white')
ExposLabel.grid(row=5,column=1,sticky='ew')
ExpSecStr.set('%1.3f' % ExpSec)
ExpSecLabel = tk.Label(sliderFrame, textvariable=ExpSecStr, bg=gray_default, fg='white')
ExpSecLabel.grid(row=5,column=2,sticky='ew')
ExposScale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeExpos,
                     from_=-8, to=5, bg=gray_default, fg='white', resolution=0.5,
                     troughcolor=gray_default,highlightbackground=gray_default,
                     length=100, width=20)
ExposScale.set(PowerOf2)
ExposScale.grid(row=5,column=3,sticky='ew')

# Stack counter setup
StackCounter = 0  # Current number of stacked images
CounterStr = tk.StringVar()  # String variable for counter display
CounterStr.set('%s' % StackCounter)
StackLabel = tk.Label(sliderFrame, text="Stack", bg=gray_default, fg='white')
StackLabel.grid(row=7,column=0,sticky='ew')
CountLabel = tk.Label(sliderFrame, textvariable=CounterStr, bg=gray_default, fg='white')
CountLabel.grid(row=7,column=1,sticky='ew')

def ResetStack():
    """
    Function to reset the image stack.
    Clears the stack array and resets counter.
    """
    global StackImage, StackCounter, fStackBusy
    fStackBusy = 1  # Set busy flag to prevent concurrent access
    StackImage = StackImage*0  # Clear stack array
    fStackBusy = 0  # Release busy flag
    StackCounter = 0  # Reset counter
    CounterStr.set('%s' % StackCounter)  # Update display
    CountLabel.configure(textvariable=CounterStr)

# Create reset button
ResetButton = tk.Button(sliderFrame, text="Reset", command=ResetStack, 
                       bg=gray_default, fg='white')
ResetButton.grid(row=7,column=3,sticky="ew")

def SaveStack():
    """
    Function to save the current image stack.
    Processes and saves the stacked image to a file.
    """
    global StackImage, StackCounter, fStackBusy, MStack
    fStackBusy = 1  # Set busy flag
    
    # Generate filename with timestamp
    outfilename = "Pictures/PCIM" + time.strftime("%Y%m%d%H%M%S")
    TIFFilename = outfilename + ".tif"

    # Process stack for saving
    MinValue = np.min(StackImage)
    MaxValue = np.max(StackImage)
    print('Stacked Image Value ranges from %f to %f' % (MinValue,MaxValue))
    
    FloatImage = StackImage - MinValue  # Shift minimum to zero
    if(int(StackCounter)>0):
        StackImage = FloatImage/float(StackCounter)  # Average the stack
        
    MinValue = np.min(StackImage)
    MaxValue = np.max(StackImage)
    print('Stacked Image is averaged')
    print('Stacked Image Value ranges from %f to %f' % (MinValue,MaxValue))
    
    # Save the processed image
    cv2.imwrite(TIFFilename, StackImage)
    print('Stacked Image Saved at ' + time.strftime("%Y%m%d%H%M%S"))
    
    fStackBusy = 0  # Release busy flag
    ResetStack()  # Reset for next stack

# Create save button
SaveButton = tk.Button(sliderFrame, text="Save", command=SaveStack, 
                      bg=gray_default, fg='white')
SaveButton.grid(row=7,column=2, sticky="ew")

# Stack display control
fStackShow = 0  # Flag for showing stack preview
def StackShowToggle():
    """
    Toggle function for stack preview display.
    Switches between showing current frame and stacked image.
    """
    global fStackShow
    if(fStackShow==1):
        fStackShow = 0
        StackShowButton.configure(bg=gray_default)
    else:
        fStackShow = 1
        StackShowButton.configure(bg=ColorBLUE2)
    global fRunDisplayUpdate
    fRunDisplayUpdate = 1

# Create stack preview toggle button
StackShowButton = tk.Button(sliderFrame, text="Show Stack", command=StackShowToggle, 
                          bg=gray_default, fg='white')
StackShowButton.grid(row=9,column=0,sticky="ew")

# Level adjustment controls
fLevel = 0  # Flag for level adjustment mode
def LevelToggle():
    """
    Toggle function for level adjustment mode.
    Enables/disables real-time brightness/contrast adjustments.
    """
    global fLevel
    if(fLevel==1):
        fLevel = 0
        LevelButton.configure(bg=gray_default)
    else:
        fLevel = 1
        LevelButton.configure(bg=ColorBLUE2)
    global fRunDisplayUpdate
    fRunDisplayUpdate = 1

# Create level adjustment toggle button
LevelButton = tk.Button(sliderFrame, text="Level Adjust", command=LevelToggle, 
                       bg=gray_default, fg='white')
LevelButton.grid(row=9,column=1,columnspan=2,sticky="ew")

# Initialize contrast and brightness values
Contrast = 0.0
Brightness = 0.1

def ResetLevel():
    """
    Function to reset brightness and contrast to default values.
    """
    global Brightness, Contrast
    Brightness = 0
    Contrast = 0
    global fRunDisplayUpdate
    fRunDisplayUpdate = 1
    BrightScale.set(Brightness)
    ContrScale.set(Contrast)

# Create reset level button
ResetLevelButton = tk.Button(sliderFrame, text="Reset Level", command=ResetLevel, 
                           bg=gray_default, fg='white')
ResetLevelButton.grid(row=9,column=3,sticky="ew")

# Brightness control setup
def ChangeBrightness(ScaleValue):
    """
    Callback function for brightness adjustment.
    Updates brightness value and triggers display update.
    """
    global Brightness
    Brightness = float(ScaleValue)
    global fRunDisplayUpdate
    fRunDisplayUpdate = 1

# Create and configure brightness controls
BrightLabel = tk.Label(sliderFrame, text="Level", bg=gray_default, fg='white')
BrightLabel.grid(row=10,column=0,sticky='ew')
BrightScale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeBrightness,
                       from_=0, to=0.5, bg=gray_default, fg='white',
                       resolution=0.0005,
                       troughcolor=gray_default,highlightbackground=gray_default,
                       length=100, width=20)
BrightScale.set(Brightness)
BrightScale.grid(row=11,column=0,sticky='ew')

# Color balance controls setup
LevelR = 0.11  # Red channel level
LevelG = 0.1   # Green channel level
LevelB = 0.114 # Blue channel level

def RLevel(Value):
    """
    Callback function for red channel adjustment.
    """
    global LevelR
    LevelR = float(Value)
    global fRunDisplayUpdate
    fRunDisplayUpdate = 1

# Create and configure red channel controls
RLevelLabel = tk.Label(sliderFrame, text="RED", bg=gray_default, fg='white')
RLevelLabel.grid(row=10,column=1,sticky='ew')
RLevelScale = tk.Scale(sliderFrame, orient='horizontal', command=RLevel,
                      from_=0, to=0.2, bg=gray_default, fg='white',
                      resolution=0.0005,
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
RLevelScale.set(LevelR)
RLevelScale.grid(row=11,column=1,sticky='ew')

# Similar controls for Green and Blue channels
[... Green and Blue channel control setup code ...]

# Contrast enhancement using Look-Up Table (LUT)
LUT10bit = np.zeros(1024, dtype=np.uint16)  # LUT for 10-bit input to 8-bit output
Center = float(0.25)   # Center point for contrast curve
Slope0 = float(0.5)    # Initial slope
Slope1 = float(10.0)   # Enhanced slope
MinLUT = np.arctan((Slope1**2)*(0-Center))
MaxLUT = np.arctan((Slope1**2)*(1.0-Center))+Slope0

# Initialize LUT with contrast curve
for i in range(1024):
    LUT10bit[i] = int(255*(np.arctan(float(Slope1**2)*(float(i)/1023.0-float(Center)))+float(i)/1023.0*float(Slope0)-MinLUT)/(MaxLUT-MinLUT))

def UpdateLUT():
    """
    Function to update the Look-Up Table for contrast enhancement.
    Recalculates LUT based on current contrast parameters.
    """
    global LUT10bit, Center, Slope0, Slope1, MinLUT, MaxLUT
    MinLUT = np.arctan(float(Slope1**2)*(0-float(Center)))
    MaxLUT = np.arctan(float(Slope1**2)*(1.0-float(Center)))+float(Slope0)
    for i in range(1024):
        LUT10bit[i] = int(255*(np.arctan(float(Slope1**2)*(float(i)/1023.0-float(Center)))+float(i)/1023.0*float(Slope0)-MinLUT)/(MaxLUT-MinLUT))
    print("LUT updated")

# Display zoom control
DispZoomFactor = 1  # Current zoom level for main display

def ChangeDispZoom(event):
    """
    Mouse wheel callback for zoom level changes.
    Allows cycling through zoom levels using mouse wheel.
    """
    global DispZoomFactor
    print("MouseWheel Event Callback is called")
    if(event.delta < 0):
        if(DispZoomFactor == 5):
            DispZoomFactor = 2
        elif(DispZoomFactor == 2):
            DispZoomFactor = 1
        print("MouseWheel Down")
    elif(event.delta > 0):
        if(DispZoomFactor == 1):
            DispZoomFactor = 2
        elif(DispZoomFactor == 2):
            DispZoomFactor = 5
        print("MouseWheel Up")

# Bind mouse wheel to zoom function
lmain.bind('<MouseWheel>', ChangeDispZoom)

# Alternative zoom control via scale widget
def ChangeDispZoom2(ScaleValue):
    """
    Scale widget callback for zoom level changes.
    Provides an alternative to mouse wheel zoom control.
    """
    global DispZoomFactor
    DispZoomFactor = int(ScaleValue)
    UpdateDisplay()
    UpdateZoom()

# Create and configure zoom scale control
DZScale = tk.Scale(imageFrame, orient='vertical', command=ChangeDispZoom2,
                  from_=1, to=5, bg=gray_default, fg='white', resolution=1,
                  troughcolor=gray_default,highlightbackground=gray_default,
                  length=50, width=20)
DZScale.set(DispZoomFactor)
DZScale.grid(row=0,column=0,sticky='ns')

# Motor control parameters calculation
# These parameters define the stepper motor movement characteristics
StepPerRev = 32       # Steps per revolution of motor
RatioInGear = 64      # Input gear ratio
RatioOutGear = 4      # Output gear ratio
DegWarm = 4           # Degrees per worm gear revolution
# Calculate degrees per step for precise movement
DegPerStep = DegWarm/(RatioOutGear * RatioInGear * StepPerRev)
# Calculate sidereal rate (Earth's rotation)
DegPerSec = 360/(24 * 60 * 60)  # Degrees per second for sidereal tracking
GuideStep = DegPerStep / DegPerSec  # Time step for tracking

# Initialize motor control variables
nPhase1 = 0              # Current phase of RA motor
fForwardDirection1 = 1    # Direction flag for RA motor
fRunThread = 1           # Thread running flag
TimeStep1 = GuideStep    # Time step for RA motor
LoopsToGo2 = 0          # Steps remaining for DEC motor
TimeStep2 = 0.02        # Time step for DEC motor

def PortOutTh1():
    """
    Thread function for Right Ascension (RA) motor control.
    Implements 4-phase stepper motor sequence for smooth movement.
    """
    global fRunThread, nPhase1, fForwardDirection1, TimeStep1
    while (fRunThread == 1):
        if(fForwardDirection1 == 1):  # Positive value goes west
            # Four-phase stepping sequence
            if(nPhase1 == 0):
                GPIO.output(6,  1)
                GPIO.output(13, 0)
                GPIO.output(19, 0)
                GPIO.output(26, 0)
                nPhase1 = 1
            elif(nPhase1 == 1):
                GPIO.output(13, 1)
                GPIO.output(19, 0)
                GPIO.output(26, 0)
                GPIO.output(6,  0)
                nPhase1 = 2
            elif(nPhase1 == 2):
                GPIO.output(19, 1)
                GPIO.output(26, 0)
                GPIO.output(6,  0)
                GPIO.output(13, 0)
                nPhase1 = 3
            elif(nPhase1 == 3):
                GPIO.output(26, 1)
                GPIO.output(6,  0)
                GPIO.output(13, 0)
                GPIO.output(19, 0)
                nPhase1 = 0
        elif(fForwardDirection1 == -1):  # Negative value goes east
            # Reverse four-phase stepping sequence
            if(nPhase1 == 0):
                GPIO.output(6,  1)
                GPIO.output(26, 0)
                GPIO.output(19, 0)
                GPIO.output(13, 0)
                nPhase1 = 3
            elif(nPhase1 == 3):
                GPIO.output(26, 1)
                GPIO.output(19, 0)
                GPIO.output(13, 0)
                GPIO.output(6,  0)
                nPhase1 = 2
            elif(nPhase1 == 2):
                GPIO.output(19, 1)
                GPIO.output(13, 0)
                GPIO.output(6,  0)
                GPIO.output(26, 0)
                nPhase1 = 1
            elif(nPhase1 == 1):
                GPIO.output(13, 1)
                GPIO.output(6,  0)
                GPIO.output(26, 0)
                GPIO.output(19, 0)
                nPhase1 = 0
        time.sleep(TimeStep1)

# Start RA motor control thread
t5 = threading.Thread(target=PortOutTh1)
t5.start()

def PortOutTh2():
    """
    Thread function for Declination (DEC) motor control.
    Implements stepping sequence for north/south movement.
    """
    global fRunThread, nPhase2, TimeStep2, LoopsToGo2, DegPerStep
    while (fRunThread == 1):
        if(LoopsToGo2 < 0):  # Negative value moves south
            # South movement stepping sequence
            if(nPhase2 == 0):
                GPIO.output(12, 1)
                GPIO.output(16, 0)
                GPIO.output(20, 0)
                GPIO.output(21, 0)
                nPhase2 = 1
            elif(nPhase2 == 1):
                GPIO.output(16, 1)
                GPIO.output(20, 0)
                GPIO.output(21, 0)
                GPIO.output(12, 0)
                nPhase2 = 2
            elif(nPhase2 == 2):
                GPIO.output(20, 1)
                GPIO.output(21, 0)
                GPIO.output(12, 0)
                GPIO.output(16, 0)
                nPhase2 = 3
            elif(nPhase2 == 3):
                GPIO.output(21, 1)
                GPIO.output(12, 0)
                GPIO.output(16, 0)
                GPIO.output(20, 0)
                nPhase2 = 0
            LoopsToGo2 += 1
        elif(LoopsToGo2 > 0):  # Positive value moves North
            # North movement stepping sequence
            if(nPhase2 == 0):
                GPIO.output(12, 1)
                GPIO.output(21, 0)
                GPIO.output(20, 0)
                GPIO.output(16, 0)
                nPhase2 = 3
            elif(nPhase2 == 3):
                GPIO.output(21, 1)
                GPIO.output(20, 0)
                GPIO.output(16, 0)
                GPIO.output(12, 0)
                nPhase2 = 2
            elif(nPhase2 == 2):
                GPIO.output(20, 1)
                GPIO.output(16, 0)
                GPIO.output(12, 0)
                GPIO.output(21, 0)
                nPhase2 = 1
            elif(nPhase2 == 1):
                GPIO.output(16, 1)
                GPIO.output(12, 0)
                GPIO.output(21, 0)
                GPIO.output(20, 0)
                nPhase2 = 0
            LoopsToGo2 -= 1
        else:
            # Reset button states when movement complete
            GoN10Button.configure(bg=gray_default)
            GoN05Button.configure(bg=gray_default)
            GoN01Button.configure(bg=gray_default)
            GoS01Button.configure(bg=gray_default)
            GoS05Button.configure(bg=gray_default)
            GoS10Button.configure(bg=gray_default)
        time.sleep(TimeStep2)

# Start DEC motor control thread
t6 = threading.Thread(target=PortOutTh2)
t6.start()

def on_closing():
    """
    Cleanup function called when closing the application.
    Ensures proper shutdown of camera, threads, and GPIO.
    """
    global fRunCamera, fRunImageUpdate, fRunDisplayUpdate
    # Stop all camera operations
    fRunCamera = 0
    fRunImageUpdate = 0
    fRunDisplayUpdate = 0
    # Wait for threads to complete
    t1.join()
    t2.join()
    # Stop motor control
    global fRunThread
    fRunThread = 0
    time.sleep(0.5)
    # Clean up GPIO
    GPIO.cleanup()
    # Destroy window
    window.destroy()
    # Wait for motor threads to complete
    t5.join()
    t6.join()

# Set up window close handler
window.protocol("WM_DELETE_WINDOW", on_closing)

# Start the GUI main loop
window.mainloop()  # Starts GUI and event processing