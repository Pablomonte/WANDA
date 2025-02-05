import time
import numpy as np
import tkinter as tk
from picamera2 import Picamera2
from PIL import Image, ImageTk

# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()

# Create Tkinter Window
root = tk.Tk()
root.title("Live Camera with Adjustable Parameters")

# Set camera resolution
WIDTH, HEIGHT = 640, 480
canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
canvas.pack()

def update_frame():
    """Captures an image from the camera and updates the Tkinter window."""
    frame = picam2.capture_array()
    frame = np.flipud(frame)  # Flip the image if necessary
    img = Image.fromarray(frame)  # Convert NumPy array to PIL Image
    img = img.resize((WIDTH, HEIGHT))  # Resize to fit canvas
    img_tk = ImageTk.PhotoImage(image=img)

    canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
    canvas.image = img_tk  # Keep reference to avoid garbage collection

    root.after(10, update_frame)  # Refresh every 10ms

# Function to update camera settings
def update_settings(value=None):
    picam2.set_controls({
        "AnalogueGain": gain_var.get(),         # Gain
        "ExposureTime": exposure_var.get(),     # Exposure
        "AwbGain": (wb_r_var.get(), wb_b_var.get()),  # White Balance (R, B)
    })

# Sliders for camera settings
gain_var = tk.DoubleVar(value=1.0)
exposure_var = tk.IntVa
