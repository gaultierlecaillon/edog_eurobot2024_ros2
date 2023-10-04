#!/usr/bin/env python3
import tkinter as tk
from tkinter import messagebox, ttk
from ttkthemes import ThemedTk
import subprocess


def execute_command(command):
    try:
        subprocess.run(command, shell=True, check=True, text=True)
    except subprocess.CalledProcessError as e:
        messagebox.showerror("Error", f"Failed to execute command: {str(e)}")


def view1():
    frame_home.tkraise()


def view2():
    frame_lidar.tkraise()


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

btn_lidar = ttk.Button(nav_bar, text="Lidar", command=view2,
                       style="Large.TButton")
btn_lidar.pack(side='left', padx=10, ipady=10)

frame_home = ttk.Frame(app)
frame_home.place(relwidth=1, relheight=1, y=80)

cmd_home_calibrate = ttk.Button(frame_home, text="Command 1",
                          command=lambda: execute_command("echo Hello View 1"),
                          style="Large.TButton")
cmd_home_calibrate.pack(pady=10, ipady=20)  # Increase ipady value to make the button taller

frame_lidar = ttk.Frame(app)
frame_lidar.place(relwidth=1, relheight=1, y=80)

''' View Lidar '''
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
