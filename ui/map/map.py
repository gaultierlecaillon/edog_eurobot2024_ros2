import tkinter as tk
from PIL import Image, ImageTk

# Image size in pixels
img_size_px = (1200, 800)
# Table size in real life (meters)
table_size_mm = (3000, 2000)
# Image file name
img_file = "table.png"

# Function to convert pixels to mm
def px_to_m(px, img_size_px, table_size_mm):
    return (px[0] * table_size_mm[0] / img_size_px[0], (img_size_px[1] - px[1]) * table_size_mm[1] / img_size_px[1])


# Function to handle click event
# Function to convert pixels to meters
def px_to_m(px, img_size_px, table_size_mm):
    return (px[0] * table_size_mm[0] / img_size_px[0], (img_size_px[1] - px[1]) * table_size_mm[1] / img_size_px[1])

# Function to handle click event
def on_click(event):
    # Position in pixels
    pos_px = (event.x, event.y)
    # Convert to meters
    pos_m = px_to_m(pos_px, img_size_px, table_size_mm)
    print(f"x={pos_m[0]}, y={pos_m[1]} (Clicked at {pos_px[0]} px, {img_size_px[1] - pos_px[1]} px)")

# Function to handle scroll event
def on_zoom(event):
    # Respond to Linux or Windows wheel event
    if event.num == 5 or event.delta == -120:
        img_lbl.zoom_out()
    if event.num == 4 or event.delta == 120:
        img_lbl.zoom_in()

# Create main window
root = tk.Tk()

# Load image
img = Image.open(img_file)
img_tk = ImageTk.PhotoImage(img)

# Create label with image
lbl = tk.Label(root, image=img_tk)
lbl.pack()

# Bind click event
lbl.bind("<Button-1>", on_click)

# Start main loop
root.mainloop()
