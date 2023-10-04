import tkinter as tk
from tkinter import messagebox, ttk
from ttkthemes import ThemedTk
import subprocess
import glob


def execute_command(command):
    try:
        subprocess.run(command, shell=True, check=True, text=True)
    except subprocess.CalledProcessError as e:
        messagebox.showerror("Error", f"Failed to execute command: {str(e)}")


def view1():
    frame_home.tkraise()


def view2():
    frame_lidar.tkraise()

def view3():
    frame_launchpad.tkraise()


app = ThemedTk(theme="equilux")
app.title("Robot App")
app.geometry("1024x600")

# Configure the style
style = ttk.Style(app)
style.configure("Large.TButton",
                font=("Arial", 20),
                foreground="#D5A247")  # Change font color here

nav_bar = ttk.Frame(app)
nav_bar.place(relwidth=1, height=80)  # Increased height

btn_home = ttk.Button(nav_bar, text="Home", command=view1,
                       style="Large.TButton")
btn_home.pack(side='left', padx=10, ipady=10)  # Use ipady to increase button height

btn_launchpad = ttk.Button(nav_bar, text="LaunchPad", command=view3,
                           style="Large.TButton")
btn_launchpad.pack(side='left', padx=10, ipady=10)

btn_lidar = ttk.Button(nav_bar, text="Lidar", command=view2,
                       style="Large.TButton")
btn_lidar.pack(side='left', padx=10, ipady=10)



''' View Home '''
frame_home = ttk.Frame(app)
frame_home.place(relwidth=1, relheight=1, y=80)

cmd_home_launch = ttk.Button(frame_home, text="Launch",
                          command=lambda: execute_command("echo Hello View 1"),
                          style="Large.TButton")
cmd_home_launch.pack(pady=10, ipady=20)  # Increase ipady value to make the button taller
''' End View Home '''

''' View LaunchPad '''
frame_launchpad = ttk.Frame(app)
frame_launchpad.place(relwidth=1, relheight=1, y=80)

# Loop through all .json files and create a button for each
for json_file in glob.glob("json/*.json"):
    def create_command(filepath):
        return lambda: execute_command(f"echo {filepath}")  # Replace echo command as per your need


    btn = ttk.Button(frame_launchpad, text=json_file.split("/")[-1],
                     command=create_command(json_file),
                     style="Large.TButton")
    btn.pack(pady=5, ipady=10)  # Adjust padding as per need
''' End View LaunchPad '''

''' View Lidar '''
frame_lidar = ttk.Frame(app)
frame_lidar.place(relwidth=1, relheight=1, y=80)

cmd_lidar_start = ttk.Button(frame_lidar, text="Start",
                          command=lambda: execute_command("Start"),
                          style="Large.TButton")
cmd_lidar_stop = ttk.Button(frame_lidar, text="Stop",
                          command=lambda: execute_command("Stop"),
                          style="Large.TButton")
cmd_lidar_start.pack(pady=10, ipady=20)
cmd_lidar_stop.pack(pady=10, ipady=20)
''' End Lidar '''

frame_home.tkraise()
app.mainloop()
