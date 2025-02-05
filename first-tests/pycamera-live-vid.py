import time
import pygame
import numpy as np
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

# Initialize Pygame
pygame.init()

# Set the display resolution (match camera resolution)
WIDTH, HEIGHT = 640, 480  # Adjust based on your camera's resolution
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Live Stream")

running = True
try:
    while running:
        frame = picam2.capture_array()  # Capture a frame as an array
        
        # Convert the frame to a format Pygame understands
        frame = np.rot90(frame)  # Rotate if needed
        frame = np.flipud(frame)  # Flip the image if necessary
        frame_surface = pygame.surfarray.make_surface(frame)

        # Display the frame
        screen.blit(frame_surface, (0, 0))
        pygame.display.update()

        # Check for quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

finally:
    picam2.close()
    pygame.quit()
