import tkinter as tk
from tkinter import ttk, scrolledtext

class RobotUI(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master.title("Python demo V3")
        self.create_widgets()

    def create_widgets(self):
        # --- Robot Connect Frame ---
        self.conn_frame = tk.LabelFrame(self, text="Robot Connect")
        self.conn_frame.pack(fill="x")

        tk.Label(self.conn_frame, text="IP Address:").grid(row=0, column=0)
        self.ip_entry = tk.Entry(self.conn_frame)
        self.ip_entry.grid(row=0, column=1)
        # Add entries for Dashboard Port, Move Port, Feedback Port, etc.

        self.connect_button = tk.Button(self.conn_frame, text="Disconnect")
        self.connect_button.grid(row=0, column=6)

        # --- Dashboard Function Frame ---
        self.dashboard_frame = tk.LabelFrame(self, text="Dashboard Function")
        self.dashboard_frame.pack(fill="x")
        # Add buttons: PowerOn, ResetRobot, etc.

        # --- Move Function Frame ---
        self.move_frame = tk.LabelFrame(self, text="Move Function")
        self.move_frame.pack(fill="x")
        # Add input fields for X, Y, Z, Rx, Ry, Rz, J1-J6

        # --- Feedback Frame ---
        self.feedback_frame = tk.LabelFrame(self, text="Feedback")
        self.feedback_frame.pack(fill="x")
        # Labels for joints, Cartesian positions, speed ratio, etc.

        # --- Error Info and Log Frame ---
        self.log_frame = tk.Frame(self)
        self.log_frame.pack(fill="both", expand=True)

        self.error_info = scrolledtext.ScrolledText(self.log_frame, height=5)
        self.error_info.pack(side="left", fill="both", expand=True)

        self.log_output = scrolledtext.ScrolledText(self.log_frame, height=10)
        self.log_output.pack(side="right", fill="both", expand=True)
