import tkinter as tk
from PIL import Image, ImageTk
from ps4_controller.msg import PS4
import ps4_controller_publisher
import script
import threading
import time

global last_received_msg

class ImageSliderApp:
    def __init__(self, root, image_paths):
        self.root = root
        self.root.title("Controls")

        # Load images
        self.image_paths = image_paths
        self.current_index = 0
        self.images = [Image.open(path) for path in image_paths]

        # Placeholder image
        self.placeholder_image = ImageTk.PhotoImage(Image.new("RGB", (300, 200), "white"))

        # Create label with placeholder image
        self.label = tk.Label(self.root, image=self.placeholder_image)
        self.label.pack(padx=10, pady=10)

        # Display the first image
        self.display_image()

        # Create forward and backward buttons
        self.btn_backward = tk.Button(root, text="Backward", command=self.show_previous_image)
        self.btn_backward.pack(side=tk.LEFT, padx=10)

        self.btn_forward = tk.Button(root, text="Forward", command=self.show_next_image)
        self.btn_forward.pack(side=tk.RIGHT, padx=10)

    def display_image(self):
        image = ImageTk.PhotoImage(self.images[self.current_index])
        self.label.config(image=image)
        self.label.image = image

    def show_next_image(self):
        self.current_index = (self.current_index + 1) % len(self.images)
        self.display_image()

    def show_previous_image(self):
        self.current_index = (self.current_index - 1) % len(self.images)
        self.display_image()

class MainWindow:

    def __init__(self, root):
        self.root = root
        self.root.title("Scorbot")

        # Set background image
        background_image_path = "scorbot_image.png"
        try:
            background_image = Image.open(background_image_path)
            background_image = ImageTk.PhotoImage(background_image)
            background_label = tk.Label(root, image=background_image)
            background_label.image = background_image  # Store reference
            background_label.place(relwidth=1, relheight=1)
        except FileNotFoundError:
            print(f"Error: Background image not found at {background_image_path}")

        # Set the size of the main window
        #self.root.geometry(f"{background_image.width()}x{background_image.height()}")

        # Create buttons
        self.btn_control_robot = tk.Button(root, text="Control Robot", command=self.control_robot)
        self.btn_control_robot.place(relx=0.5, rely=0.4, anchor=tk.CENTER)

        self.btn_controls = tk.Button(root, text="Controls", command=self.show_controls)
        self.btn_controls.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

    def control_robot(self):
        ps4_controller_ready = threading.Event()
        # Define the function to run ps4_controller_publisher
        def run_ps4_controller_publisher():
            ps4_controller_publisher.main()
            # Signal that ps4_controller_publisher is ready
            ps4_controller_ready.set()

        # Create instances of the threads
        ps4_thread = threading.Thread(target=script.main)
        ps4_publisher_thread = threading.Thread(target=run_ps4_controller_publisher)

        # Start both threads
        ps4_publisher_thread.start()
        ps4_thread.start()

        # Wait for both threads to finish
        ps4_publisher_thread.join()
        ps4_thread.join()
        

    def show_controls(self):
        # Create a new window for the image slider
        image_slider_window = tk.Toplevel(self.root)

        # Provide the image paths to the ImageSliderApp
        #image_paths = ["Ps4_controller_1.png", "Ps4_controller_2.png"]
        #image_slider_app = ImageSliderApp(image_slider_window, image_paths)

if __name__ == "__main__":
    root = tk.Tk()
    main_window = MainWindow(root)
    root.mainloop()