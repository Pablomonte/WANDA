import time
from picamera2 import Picamera2

# Generate timestamp
timestamp = time.strftime("%Y%m%d_%H%M%S")

# Define the filename with timestamp
filename = f"image_{timestamp}.jpg"

# Initialize and start Picamera2
picam2 = Picamera2()
picam2.start()
picam2.capture_file(filename)

print(f"Image saved as {filename}")