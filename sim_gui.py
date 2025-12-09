# sim_gui.py
import os, sys

import tkinter as tk
from tkinter import ttk, messagebox

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# Import your simulation core
from sim_core import (
    simulate_run,
    plot_FD_curves,
    plot_SD_FD_curves,
    plot_torque_curve,
    plot_tire_curve,
    plot_results,
    FINAL_DRIVE,
    SHIFT_DELAY,
    M_VEHICLE,
    CD,
    A_FRONTAL,
    TARGET_DISTANCE,
    carInfo
)

def setup_graph(ax):
    ax.set_facecolor('#424242')
    ax.set_ylabel('', color='#ffffff')
    ax.set_xlabel('', color='#ffffff')
    ax.tick_params(labelcolor='#ffffff', color='#ffffff', grid_color='#ffffff')
    ax.set_title('', color='#ffffff')

def make_input(ttk, self, parent_frameA, parent_frameB, label, variable):
    ttk.Label(parent_frameA, text=f"{label}").pack(anchor="w", pady=(0, 3.5), fill=tk.BOTH)
    self.var = tk.StringVar(value=f"{variable:.3f}")
    ttk.Entry(parent_frameB, textvariable=self.var, width=10).pack(anchor="w", pady=(0, 4))
    return self.var

class FSAESimApp:
    def __init__(self, master: tk.Tk):
        self.master = master
        master.title("FSAE Acceleration Simulator")

        filename = "black.tcl"
        if hasattr(sys, "_MEIPASS"):                        # running from PyInstaller bundle
            filename = os.path.join(sys._MEIPASS, filename) # type: ignore #_MEIPASS only exists when program is built with pyinstaller
        else:
            filename = os.path.abspath(filename)            # running from source
        
        master.tk.call('source', filename)
        ttk.Style(master).theme_use("black")
        
        # ----- Left: controls -----
        controls = ttk.Frame(master, padding=5)
        controls.pack(side=tk.LEFT, fill=tk.BOTH)

        # Parameters
        ttk.Label(controls, text="Car Parameters", font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(0, 4))

        parameter_labels = ttk.Frame(controls)
        parameter_labels.pack(side=tk.LEFT, fill=tk.BOTH)
        
        parameter_inputs = ttk.Frame(controls)
        parameter_inputs.pack(side=tk.RIGHT, fill=tk.BOTH)

        #self.fd_var = make_input(ttk, self, parameter_labels, parameter_inputs, "Final drive ratio [-]", FINAL_DRIVE)
        ttk.Label(parameter_labels, text="Final drive ratio [-]").pack(anchor="w", pady=(0, 3.5))

        self.big_ratio = tk.StringVar(value=f"{41}")    # Rear sprocket teeth
        self.little_ratio = tk.StringVar(value=f"{11}") # Front sprocket teeth
        fd_inputs = ttk.Frame(parameter_inputs)
        fd_inputs.pack(side=tk.TOP, fill=tk.BOTH)
        ttk.Entry(fd_inputs, textvariable=self.big_ratio, width=10).pack(anchor="e", pady=(0, 4), side=tk.RIGHT, fill=tk.BOTH)
        ttk.Label(fd_inputs, text=":").pack(anchor="e", pady=(0, 3.5), side=tk.RIGHT, fill=tk.BOTH)
        ttk.Entry(fd_inputs, textvariable=self.little_ratio, width=10).pack(anchor="e", pady=(0, 4), side=tk.RIGHT, fill=tk.BOTH)
        
        self.sd_var = make_input(ttk, self, parameter_labels, parameter_inputs, "Shift delay [s]", SHIFT_DELAY)
        self.m_var = make_input(ttk, self, parameter_labels, parameter_inputs, "Car Mass [kg]", M_VEHICLE)
        self.cd_var = make_input(ttk, self, parameter_labels, parameter_inputs, "Drag Coeffiecent [-]", CD)
        self.af_var = make_input(ttk, self, parameter_labels, parameter_inputs, "Frontal Area [m^2]", A_FRONTAL)
        # old friction model
        #self.mup_var = make_input(ttk, self, parameter_labels, parameter_inputs, "Peak MU [-]", MU_PEAK)
        #self.kp_var = make_input(ttk, self, parameter_labels, parameter_inputs, "Peak Slip Ratio [%]", KAPPA_PEAK * 100)
        #self.mus_var = make_input(ttk, self, parameter_labels, parameter_inputs, "MU Slide [-]", MU_SLIDE)

        plot_controls = ttk.Frame(controls)
        plot_controls.pack(side=tk.BOTTOM, fill=tk.BOTH)
        # Plot mode
        ttk.Label(plot_controls, text="Graph type", font=("Segoe UI", 10, "bold")).pack(anchor="center", pady=(8, 2))
        self.plot_mode = tk.StringVar(value="single")

        ttk.Radiobutton(
            plot_controls,
            text="Single run (75 m)",
            value="single",
            variable=self.plot_mode,
        ).pack(anchor="w")
        ttk.Radiobutton(
            plot_controls,
            text="FD sweep (fixed shift delay)",
            value="fd_sweep",
            variable=self.plot_mode,
        ).pack(anchor="w")
        ttk.Radiobutton(
            plot_controls,
            text="FD & shift delay sweep",
            value="fd_sd_sweep",
            variable=self.plot_mode,
        ).pack(anchor="w")
        ttk.Radiobutton(
            plot_controls,
            text="Tourque Curve",
            value="tourque_curve",
            variable=self.plot_mode,
        ).pack(anchor="w")
        ttk.Radiobutton(
            plot_controls,
            text="Grip Curve",
            value="grip_curve",
            variable=self.plot_mode,
        ).pack(anchor="w")

        # Action buttons
        ttk.Button(plot_controls, text="Plot", command=self.run_plot).pack(anchor="center", pady=(12, 4))
        ttk.Button(plot_controls, text="Quit", command=master.destroy).pack(anchor="center", pady=(0, 4))

        # Info label
        self.info_label = ttk.Label(
            plot_controls,
            text=f"Target distance: {TARGET_DISTANCE:.0f} m"
        )
        self.info_label.pack(anchor="center", pady=(10, 0))

        # ----- Right: plot area -----
        plot_frame = ttk.Frame(master, padding=0.5)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = Figure(facecolor="#424242", layout='tight', edgecolor='#ffffff')
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Do an initial plot
        self.run_plot()

    def run_plot(self, initial: bool = False):
        """Read parameters, choose plot mode, and draw the appropriate graph."""

        car = carInfo()

        # Parse parameters
        try:
            car.fd1 = float(self.big_ratio.get()) / float(self.little_ratio.get())
            car.sd = float(self.sd_var.get())
            car.mass = float(self.m_var.get())
            car.cd = float(self.cd_var.get())
            car.af = float(self.af_var.get())
            # Old friction model
            #car.mu_peak = float(self.mup_var.get())
            #car.kappa_peak = float(self.kp_var.get()) / 100
            #car.mu_slide = float(self.mus_var.get())
        except ValueError:
            messagebox.showerror("Input error", "Please enter numeric values for final drive and shift delay.")
            return None

        mode = self.plot_mode.get()

        # Clear figure and create a new Axes
        self.fig.clear()
        ax0 = self.fig.add_subplot(111)
        setup_graph(ax0)
        
        try:
            if mode == "single":
                # Run a single simulation and plot speed vs time
                self.fig.clear()
                ax0 = self.fig.add_subplot(411, sharex=ax0)
                setup_graph(ax0)
                ax1 = self.fig.add_subplot(412, sharex=ax0)
                setup_graph(ax1)
                ax2 = self.fig.add_subplot(413, sharex=ax0)
                setup_graph(ax2)
                ax3 = self.fig.add_subplot(414, sharex=ax0)
                setup_graph(ax3)

                res = simulate_run(car)

                plot_results([ax0, ax1, ax2, ax3], res)

                # Simple text summary in the corner
                summary = (
                    "Final Time = {:.3f} s\nFinal Velocity = {:.1f} km/h"
                    .format(res["t_final"], res["v_final"] * 3.6)
                )

                ax0.text(
                    0.02, 0.98, summary,
                    transform=ax0.transAxes,
                    va="top",
                    ha="left",
                    fontsize=9,
                    bbox=dict(boxstyle="round", facecolor="white", alpha=0.6),
                )
            elif mode == "fd_sweep":
                # Plot FD curves on this Axes for a fixed shift delay
                plot_FD_curves(ax0, car)
                ax0.set_title(
                    "Final drive sweep (shift delay = {:.0f} ms)".format(car.sd * 1000.0)
                )
            elif mode == "fd_sd_sweep":
                # Plot FD & shift delay sweep on this Axes
                plot_SD_FD_curves(ax0, car)
                ax0.set_title("Final drive & shift delay sweep")
            elif mode == "tourque_curve":
                # plot wheel tourque
                plot_torque_curve(ax0)
            elif mode == "grip_curve":
                plot_tire_curve(ax0, car)
            else:
                ax0.text(0.5, 0.5, "Unknown mode", transform=ax0.transAxes, ha="center", va="center")
        except Exception as e:
            if not initial:
                messagebox.showerror("Error", f"Error while plotting:\n{e}")
        self.fig.tight_layout()
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = FSAESimApp(root)
    root.mainloop()
