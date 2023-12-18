import cv2

def main():
    # Open a connection to the camera (use the correct index for your camera)
    cap = cv2.VideoCapture(4)

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    try:
        # Read and display frames from the camera
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            # Display the frame
            cv2.imshow('Camera Feed', frame)

            # Break the loop if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Release the camera and close the window
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()