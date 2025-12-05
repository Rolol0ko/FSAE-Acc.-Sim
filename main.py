import numpy as np
import matplotlib.pyplot as plt

# Vehicle & environment
M_VEHICLE = 226.0       # [kg] vehicle + driver mass
G = 9.81                # [m/s^2] gravity
CR = 0.012              # [-] rolling resistance coefficient
R_TIRE = 0.2032         # [m] effective loaded tire radius
D_AIR = 1.225           # [kg/m^3] air density
CD = 0.1                # [-] drag coefficient
A_FRONTAL = 1.0         # [m^2] frontal area
MU_PEAK = 1.4           # peak friction coefficient
KAPPA_PEAK = 0.60       # slip ratio at peak (~10%)
MU_SLIDE = 0.9          # sliding friction at high slip
DRAG_COEFF = 0.5 * D_AIR * CD * A_FRONTAL  # [N/(m/s)^2] coefficient for F_drag = D * v^2
N_DRIVEN = M_VEHICLE * G

# Powertrain
PRIMARY_REDUCTION = 1.974     # [-] engine primary reduction
FINAL_DRIVE = 3.727           # [-] final drive ratio
FINAL_DRIVE_NEW = 3.417       # [-] proposed final drive ratio
GEAR_RATIOS = {
    1: 2.687,
    2: 2.105,
    3: 1.761,
    4: 1.521,
    5: 1.347,
    6: 1.23,
}

ENGINE_REDLINE_RPM = 13000.0  # [rpm]
ENGINE_IDLE_RPM = 6000.0      # [rpm]
ENGINE_LAUNCH_RPM = 9000.0    # [rpm]
LAUNCH_DURATION = 0.3         # [s]

# Torque map
# Linearly interpolated between these points [rpm, lbft]
WHEEL_TORQUE_POINTS = np.array([
    [6800.0, 24.0],
    [7000.0, 26.0],
    [7200.0, 30.0],
    [7500.0, 36.0],
    [7800.0, 38.0],
    [8000.0, 36.0],
    [8600.0, 31.5],
    [8800.0, 30.5],
    [9000.0, 32.0],
    [9300.0, 36.0],
    [9600.0, 39.0],
    [9920.0, 40.0],
    [10500.0, 38.0],
    [11000.0, 37.0],
    [11900.0, 34.0],
    [12000.0, 33.0],
    [12200.0, 24.0],
    [12500.0, 3.0]
])

# Shifting & control
SHIFT_DELAY = 0.15       # [s] duration of no drive force during an upshift
USE_AUTO_SHIFT = True    # if False, no shifting: stay in 1st gear

# Per-gear upshift RPM thresholds
SHIFT_RPM_THRESHOLDS = {
    1: 11000.0,
    2: 12500.0,
    3: 12500.0,
    4: 12700.0,
    5: 13000.0,
    6: None,     # no upshift from 6th
}

# Simulation control
DT = 0.001               # [s] time step
T_MAX = 10.0             # [s] safety time limit

# Distance Based
TARGET_DISTANCE = 75.0   # [m] acceleration event distance (FSAE-style)

# Or Speed Based
TARGET_SPEED_KMH = 100.0           # [km/h] target speed
TARGET_SPEED_MS = TARGET_SPEED_KMH / 3.6  # [m/s]

def wheel_torque_from_rpm(rpm: float) -> float:
    """
    Return wheel torque [N·m] at a given engine speed [rpm],
    using linear interpolation of ENGINE_TORQUE_POINTS_LBFT (in lb·ft).
    """
    rpms = WHEEL_TORQUE_POINTS[:, 0]
    torques_lbft = WHEEL_TORQUE_POINTS[:, 1]

    # Clamp rpm inside range
    if rpm <= rpms[0]:
        tq_lbft = torques_lbft[0]
    elif rpm >= rpms[-1]:
        tq_lbft = torques_lbft[-1]
    else:
        tq_lbft = float(np.interp(rpm, rpms, torques_lbft))

    # Convert to N·m for the rest of the model
    return tq_lbft * 1.35581795

def tire_mu_from_slip(kappa: float) -> float:
    """
    Very simple longitudinal mu(kappa) curve.
    kappa > 0 for traction; curve rises to a peak then falls slightly.
    """
    kappa = max(kappa, 0.0)

    if kappa <= KAPPA_PEAK:
        # Linear build-up to peak
        return MU_PEAK * (kappa / KAPPA_PEAK)
    else:
        # Exponential decay toward slide friction
        decay = np.exp(-(kappa - KAPPA_PEAK) / 0.2)
        return MU_SLIDE + (MU_PEAK - MU_SLIDE) * decay

def traction_limited_force(F_drive_ideal: float, normal_load: float, kappa: float) -> float:
    """
    Traction limit using a simple mu(kappa) curve.
    """
    mu = tire_mu_from_slip(kappa)
    F_max = mu * normal_load
    return float(np.clip(F_drive_ideal, -F_max, F_max))

def simulate_run(final_drive: float = FINAL_DRIVE, shift_delay: float = SHIFT_DELAY):
    deltaT = 0.0
    velocity = 0.0
    deltaX = 0.0
    gear = 2
    shifting = False
    shift_time_left = 0.0
    next_gear = gear

    # Data recording lists
    ts = []
    xs = []
    vs = []
    axs = []
    gears = []
    eng_rpms = []
    drive_forces = []

    while deltaX < TARGET_DISTANCE and deltaT < T_MAX:
    #while velocity < TARGET_SPEED_MS and deltaT < T_MAX:
        # Wheel speed from vehicle speed
        omega_w = velocity / R_TIRE  # [rad/s]
        gear_ratio = GEAR_RATIOS[gear]

        if deltaT < LAUNCH_DURATION:
            # During launch: hold engine at (roughly) fixed high RPM
            engine_rpm = np.clip(ENGINE_LAUNCH_RPM, ENGINE_IDLE_RPM, ENGINE_REDLINE_RPM)
        else:
            # After launch: fully coupled engine–wheel kinematics
            engine_rad_per_s = omega_w * PRIMARY_REDUCTION * gear_ratio * final_drive
            engine_rpm = engine_rad_per_s * 60.0 / (2.0 * np.pi)
            engine_rpm = np.clip(engine_rpm, ENGINE_IDLE_RPM, ENGINE_REDLINE_RPM)

        # Automatic upshift logic (initiate shift)
        if USE_AUTO_SHIFT and not shifting:
            shift_rpm = SHIFT_RPM_THRESHOLDS[gear]
            if shift_rpm is not None and engine_rpm >= shift_rpm and gear < max(GEAR_RATIOS.keys()):
                # Start shift
                shifting = True
                shift_time_left = shift_delay
                next_gear = gear + 1

        # Compute drive force
        if shifting:
            F_drive = 0.0
            shift_time_left -= DT
            if shift_time_left <= 0.0:
                shifting = False
                gear = next_gear
        else:
            # Engine torque at current rpm
            T_e = wheel_torque_from_rpm(engine_rpm)

            # Driveline torque to wheel
            gear_ratio = GEAR_RATIOS[gear]
            T_in = T_e * PRIMARY_REDUCTION * gear_ratio * final_drive

            # Ideal wheel thrust
            F_drive_ideal = T_in / R_TIRE

            # Longitudinal slip ratio (simple definition, avoid div-by-zero)
            if velocity < 0.1:  # near standstill, approximate high slip
                kappa = 1.0
            else:
                kappa = (omega_w - velocity) / max(velocity, 1e-3)

            # Apply simple traction limit
            F_drive = traction_limited_force(F_drive_ideal, N_DRIVEN, kappa)

        # Resistive forces
        F_drag = DRAG_COEFF * velocity**2
        F_roll = CR * M_VEHICLE * G

        # Longitudinal acceleration
        a_x = (F_drive - F_drag - F_roll) / M_VEHICLE

        # Integrate state (simple Euler)
        velocity = velocity + a_x * DT
        if velocity < 0.0:
            velocity = 0.0
        deltaX = deltaX + velocity * DT
        deltaT = deltaT + DT

        # Record data
        ts.append(deltaT)
        xs.append(deltaX)
        vs.append(velocity)
        axs.append(a_x)
        gears.append(gear)
        eng_rpms.append(engine_rpm)
        drive_forces.append(F_drive)

    results = {
        "t": np.array(ts),
        "x": np.array(xs),
        "v": np.array(vs),
        "a": np.array(axs),
        "gear": np.array(gears),
        "engine_rpm": np.array(eng_rpms),
        "F_drive": np.array(drive_forces),
        "t_final": deltaT,
        "x_final": deltaX,
        "v_final": velocity,
        "final_drive": final_drive,
        "shift_delay": shift_delay,
    }
    return results

def print_distance_summary(results):
    t_final = results["t_final"]
    v_final = results["v_final"]
    print(f"Final distance: {TARGET_DISTANCE:.1f} m")
    print(f"Time to {TARGET_DISTANCE:.1f} m: {t_final:.3f} s")
    print(f"Final speed: {v_final * 3.6:.1f} km/h")

def print_time_summary(results):
    t_final = results["t_final"]
    v_final = results["v_final"]
    x_final = results["x_final"]
    print(f"Target: 0 -> {TARGET_SPEED_KMH:.0f} km/h")
    print(f"Time to {TARGET_SPEED_KMH:.0f} km/h: {t_final:.3f} s")
    print(f"Speed at end: {v_final * 3.6:.1f} km/h")
    print(f"Distance covered: {x_final:.1f} m")

def plot_results(results):
    t = results["t"]
    v = results["v"]
    a = results["a"]
    gear = results["gear"]
    rpm = results["engine_rpm"]

    fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)

    # Speed
    axs[0].plot(t, v * 3.6, label="Speed")
    axs[0].set_ylabel("Speed [km/h]")
    axs[0].grid(True)

    # Acceleration in Gs
    axs[1].plot(t, a / G, label="Accel", color="C1")  # G = 9.81 defined at top
    axs[1].set_ylabel("a_x [G]")
    axs[1].grid(True)
    
    # Engine RPM
    axs[2].plot(t, rpm, label="Engine RPM", color="C3")
    axs[2].set_ylabel("RPM")
    axs[2].grid(True)

    # Gear
    axs[3].step(t, gear, where="post", label="Gear", color="C2")
    axs[3].set_ylabel("Gear")
    axs[3].set_xlabel("Time [s]")
    axs[3].grid(True)

    fig.suptitle(
        f"FSAE Accel Simulation\nFD={results['final_drive']:.2f}, "
        f"shift delay={results['shift_delay']*1000:.0f} ms"
    )
    plt.tight_layout()
    plt.show()

def plot_torque_curve():
    """Plot the engine torque curve using engine_torque_from_rpm()."""
    # Base RPM points from the map (for markers)
    rpms_base = WHEEL_TORQUE_POINTS[:, 0]
    tq_lbft_base = WHEEL_TORQUE_POINTS[:, 1]

    # Smooth RPM range
    rpm_smooth = np.linspace(rpms_base[0], rpms_base[-1], 300)

    # Use the sim's interpolation function (returns N·m), then convert to lb·ft
    tq_nm_smooth = np.array([wheel_torque_from_rpm(r) for r in rpm_smooth])
    tq_lbft_smooth = tq_nm_smooth / 1.35581795

    plt.figure()
    plt.xlim(5000, 14000)
    plt.ylim(0, 100)
    plt.plot(rpm_smooth, tq_lbft_smooth, label="Torque (from engine_torque_from_rpm)", color="C0")
    plt.scatter(rpms_base, tq_lbft_base, color="C1", label="Input map points")
    plt.xlabel("Engine speed [rpm]")
    plt.ylabel("Wheel torque [lb·ft]")
    plt.title("Input Wheel Torque Map (via engine_torque_from_rpm)")
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_SD_FD_curves():
    # small sweep over final drive ratios and shift delay times
    plt.figure()

    fds = np.linspace(2, 4.5, 20)
    shift_delays = np.linspace(0.09, 1, 5)

    for sd in shift_delays:
        times = []
        for fd in fds:
            res_fd = simulate_run(final_drive=fd, shift_delay=sd)
            times.append(res_fd['t_final'])
        plt.plot(times, fds, 'k')

    res = simulate_run(FINAL_DRIVE, SHIFT_DELAY)
    plt.plot(res["t"][-1], FINAL_DRIVE, 'or')
    res = simulate_run(FINAL_DRIVE_NEW, SHIFT_DELAY)
    plt.plot(res["t"][-1], FINAL_DRIVE_NEW, 'og')
    res = simulate_run(3.182, SHIFT_DELAY)
    plt.plot(res["t"][-1], 3.182, '*b')
    res = simulate_run(2.917, SHIFT_DELAY)
    plt.plot(res["t"][-1], 2.917, '*c')

    plt.ylabel("Final drive ratio")
    plt.ylim(2, 4.5)
    plt.xlabel(f"Time to {TARGET_DISTANCE:.0f} m [s]")
    plt.xlim(3.5, 6)
    plt.grid(True)
    plt.title(f"Shift Delay Time Effect on Final Drive Ratio")
    plt.show()

def plot_FD_curves():
    fds = np.linspace(2, 4.5, 40)
    times = []
    plt.figure()

    for fd in fds:
        res_fd = simulate_run(fd, SHIFT_DELAY)
        times.append(res_fd['t_final'])
    plt.plot(times, fds, 'k')
    
    res = simulate_run(FINAL_DRIVE, SHIFT_DELAY)
    plt.plot(res["t"][-1], FINAL_DRIVE, 'or')
    res = simulate_run(FINAL_DRIVE_NEW, SHIFT_DELAY)
    plt.plot(res["t"][-1], FINAL_DRIVE_NEW, 'og')
    res = simulate_run(3.182, SHIFT_DELAY)
    plt.plot(res["t"][-1], 3.182, '*b')
    res = simulate_run(2.917, SHIFT_DELAY)
    plt.plot(res["t"][-1], 2.917, '*c')

    plt.ylabel("Final drive ratio")
    plt.ylim(2, 4.5)
    plt.xlabel(f"Time to {TARGET_DISTANCE:.0f} m [s]")
    plt.xlim(3.5, 6)
    plt.grid(True)
    plt.title(f"Optimized Final Drive Ratios for SD of {SHIFT_DELAY*1000} ms")
    plt.show()

if __name__ == "__main__":
    # Single run with current parameters
    res = simulate_run(FINAL_DRIVE, SHIFT_DELAY)
    #print_time_summary(res)
    #print_distance_summary(res)
    #plot_results(res)
    #plot_torque_curve()
    #plot_SD_FD_curves()
    plot_FD_curves()


