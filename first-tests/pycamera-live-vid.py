import time
import cv2
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

# OpenCV window to display the stream
cv2.namedWindow("Live Stream", cv2.WINDOW_AUTOSIZE)

try:
    while True:
        frame = picam2.capture_array()  # Capture a frame
        cv2.imshow("Live Stream", frame)  # Display the frame

        # Press 'q' to exit the preview
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.close()
    cv2.destroyAllWindows()
