# sim_gui.py
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
    plot_results,
    FINAL_DRIVE,
    SHIFT_DELAY,
    TARGET_DISTANCE,
    TARGET_SPEED_KMH,
)

def setup_graph(ax):
    ax.set_facecolor('#424242')
    ax.set_ylabel('', color='#ffffff')
    ax.set_xlabel('', color='#ffffff')

class FSAESimApp:
    def __init__(self, master: tk.Tk):
        self.master = master
        master.title("FSAE Acceleration Simulator")

        style = ttk.Style(master)
        root.tk.call('source', 'black.tcl')
        style.theme_use("black")
        
        # ----- Left: controls -----
        controls = ttk.Frame(master, padding=5)
        controls.pack(side=tk.LEFT, fill=tk.Y)

        # Parameters
        ttk.Label(controls, text="Parameters", font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(0, 4))

        # Final drive
        ttk.Label(controls, text="Final drive ratio").pack(anchor="w")
        self.fd_var = tk.StringVar(value=f"{FINAL_DRIVE:.3f}")
        ttk.Entry(controls, textvariable=self.fd_var, width=10).pack(anchor="w", pady=(0, 4))

        # Shift delay
        ttk.Label(controls, text="Shift delay [s]").pack(anchor="w")
        self.sd_var = tk.StringVar(value=f"{SHIFT_DELAY:.3f}")
        ttk.Entry(controls, textvariable=self.sd_var, width=10).pack(anchor="w", pady=(0, 8))

        # Plot mode
        ttk.Label(controls, text="Graph type", font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(8, 2))
        self.plot_mode = tk.StringVar(value="single")

        ttk.Radiobutton(
            controls,
            text="Single run (0 → {:.0f} km/h)".format(TARGET_SPEED_KMH),
            value="single",
            variable=self.plot_mode,
        ).pack(anchor="w")
        ttk.Radiobutton(
            controls,
            text="FD sweep (fixed shift delay)",
            value="fd_sweep",
            variable=self.plot_mode,
        ).pack(anchor="w")
        ttk.Radiobutton(
            controls,
            text="FD & shift delay sweep",
            value="fd_sd_sweep",
            variable=self.plot_mode,
        ).pack(anchor="w")
        ttk.Radiobutton(
            controls,
            text="Tourque Curve",
            value="tourque_curve",
            variable=self.plot_mode,
        ).pack(anchor="w")

        # Action buttons
        ttk.Button(controls, text="Plot", command=self.run_plot).pack(anchor="center", pady=(12, 4))
        ttk.Button(controls, text="Quit", command=master.destroy).pack(anchor="center", pady=(0, 4))

        # Info label
        self.info_label = ttk.Label(
            controls,
            text=f"Target distance: {TARGET_DISTANCE:.0f} m\nTarget speed: {TARGET_SPEED_KMH:.0f} km/h",
            justify="left",
        )
        self.info_label.pack(anchor="w", pady=(10, 0))

        # ----- Right: plot area -----
        plot_frame = ttk.Frame(master, padding=1)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = Figure(facecolor="#424242", figsize=(7, 5), dpi=100, tight_layout=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Do an initial plot
        self.run_plot(initial=True)

    def run_plot(self, initial: bool = False):
        """Read parameters, choose plot mode, and draw the appropriate graph."""
        # Parse parameters
        try:
            fd = float(self.fd_var.get())
            sd = float(self.sd_var.get())
        except ValueError:
            if not initial:
                messagebox.showerror("Input error", "Please enter numeric values for final drive and shift delay.")
            return

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
                
                res = simulate_run(final_drive=fd, shift_delay=sd)

                plot_results([ax0, ax1, ax2, ax3], res)
                
                # Simple text summary in the corner
                summary = (
                    "t_final = {:.3f} s\nv_final = {:.1f} km/h\nx_final = {:.1f} m"
                    .format(res["t_final"], res["v_final"] * 3.6, res["x_final"])
                )
                
                ax0.text(
                    0.02, 0.98, summary,
                    transform=ax0.transAxes,
                    va="top",
                    ha="left",
                    fontsize=9,
                    bbox=dict(boxstyle="round", facecolor="white", alpha=0.7),
                )
            elif mode == "fd_sweep":
                # Plot FD curves on this Axes for a fixed shift delay
                plot_FD_curves(ax0, sd)
                ax0.set_title(
                    "Final drive sweep (shift delay = {:.0f} ms)".format(sd * 1000.0)
                )
            elif mode == "fd_sd_sweep":
                # Plot FD & shift delay sweep on this Axes
                plot_SD_FD_curves(ax0)
                ax0.set_title("Final drive & shift delay sweep")
            elif mode == "tourque_curve":
                # plot wheel tourque
                plot_torque_curve(ax0)
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
