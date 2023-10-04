import tkinter as tk
import subprocess

def run_command(command):
    subprocess.call(command, shell=True)

root = tk.Tk()
root.geometry('300x200')

button_commands = [
    ('start.json', 'ros2 launch launcher robot.launch.py strat:=strat'),
    ('10forward.json', 'ros2 launch launcher robot.launch.py strat:=10forward'),
]

for i, (button_text, command) in enumerate(button_commands):
    button = tk.Button(root, text=button_text, command=lambda c=command: run_command(c))
    button.pack(fill='x')

root.mainloop()
