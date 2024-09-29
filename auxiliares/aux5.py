import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

def main():
    # Open the webcam
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        
        # Check if frame reading was successful
        if not ret:
            break
        
        # Show the frame
        cv2.imshow('frame', frame)
        cv2.waitKey(1)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

# Run the main function
if __name__ == '__main__':
    main()