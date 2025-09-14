#!/usr/bin/env python3
"""
Vector Fleet Solver
-------------------
A command-line helper for computing a firing solution in the class Battleship-style game.
It asks for:
- Ship (to determine muzzle velocity) or manual v0
- Own ship position (x,y) in meters
- Target initial position (x,y) in meters
- Own ship motion (speed and heading) in m/s and degrees (0° = East, 90° = North)
- Target ship motion (speed and heading)
- Wind (speed and heading) applied to the projectile horizontally
Then it iteratively estimates a time-of-flight and corrects for relative motion + wind,
computing azimuth (in-plane direction) and elevation (angle above horizontal).
Assumptions: flat Earth, no drag except wind as a constant horizontal vector, level launch/target heights.
"""

import sys
from math import atan2, cos, sin, sqrt, asin, radians, degrees, isfinite

G = 9.81  # m/s^2

# Ship data (muzzle velocities in m/s). Extend as desired.
SHIPS = {
    "Midway": {"muzzle_velocity": 762.0},
    "Alabama": {"muzzle_velocity": 701.0},
    "Atlanta": {"muzzle_velocity": 790.0},
    "Dolphin": {"muzzle_velocity": 18.0},     # Torpedo (horizontal), but treat as low-velocity launcher
    "Farragut": {"muzzle_velocity": 790.0},
}

def heading_speed_to_vxvy(speed_mps: float, heading_deg: float):
    """
    Convert speed (m/s) and heading (deg) to Cartesian components.
    Heading convention: 0° = +x (East), 90° = +y (North), increases CCW.
    """
    th = radians(heading_deg)
    vx = speed_mps * cos(th)
    vy = speed_mps * sin(th)
    return vx, vy

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def angle_for_range(v0: float, R: float, g: float = G):
    """
    Given muzzle speed v0 and horizontal ground range R, return the two possible elevation angles (low, high) in radians.
    Uses: R = (v0^2/g) * sin(2θ). If R exceeds max range, returns None.
    """
    arg = g * R / (v0**2)
    if arg < -1 or arg > 1:
        return None
    two_theta = asin(clamp(arg, -1, 1))
    low = 0.5 * two_theta
    high = 0.5 * (3.141592653589793 - two_theta)  # π/2 - low + π/2
    return low, high

def norm(dx, dy):
    return sqrt(dx*dx + dy*dy)

def unit(dx, dy):
    n = norm(dx, dy)
    if n == 0:
        return 0.0, 0.0
    return dx/n, dy/n

def fmt(v, digits=3):
    return f"{v:.{digits}f}"

def get_float(prompt, default=None):
    while True:
        s = input(f"{prompt}" + (f" [{default}]" if default is not None else "") + ": ").strip()
        if s == "" and default is not None:
            return float(default)
        try:
            return float(s)
        except ValueError:
            print("  Please enter a number.")

def choose_ship():
    print("\nShips available:")
    names = list(SHIPS.keys())
    for i, name in enumerate(names, 1):
        print(f"  {i}) {name} (v0 = {SHIPS[name]['muzzle_velocity']} m/s)")
    print(f"  {len(names)+1}) Custom (enter muzzle velocity manually)")
    while True:
        idx = input("Choose a ship by number: ").strip()
        if idx.isdigit():
            idx = int(idx)
            if 1 <= idx <= len(names):
                name = names[idx-1]
                return name, SHIPS[name]["muzzle_velocity"]
            elif idx == len(names)+1:
                v0 = get_float("Enter muzzle velocity (m/s)")
                return "Custom", v0
        print("  Invalid choice.")

def solve_fire(
    own_pos, target_pos,
    own_vel, target_vel,
    wind_vel,
    v0,
    prefer_high_arc=False,
    max_iters=12,
    tol_time=1e-3
):
    """
    Iteratively solve for azimuth and elevation to hit a moving target with wind.
    Steps:
      - Initialize t assuming 45° launch (vx ≈ v0/√2).
      - Predict target position after t.
      - Compensate for own ship displacement and wind drift over t.
      - Compute displacement vector d and range R = |d|.
      - Compute elevation angle θ from R using range formula (no vertical offsets).
      - Compute vx = v0 * cos θ, update t = R / vx.
      - Repeat until t converges.
    Returns: dict with keys (theta_deg, azimuth_deg, time_s, impact_xy, predicted_target_xy, iterations, low_high_deg)
    """
    x0, y0 = own_pos
    xt0, yt0 = target_pos
    vox, voy = own_vel
    vtx, vty = target_vel
    vwx, vwy = wind_vel

    # Initial guess for time: assume 45° and straight-line to current target
    dx0 = (xt0 - x0)
    dy0 = (yt0 - y0)
    R0 = norm(dx0, dy0)
    if v0 <= 0:
        raise ValueError("Muzzle velocity must be positive.")
    vx_guess = v0 / 2**0.5
    t = R0 / max(vx_guess, 1e-6)

    theta_rad = None
    azimuth_rad = None

    for it in range(1, max_iters+1):
        # Predict target after t
        xt = xt0 + vtx * t
        yt = yt0 + vty * t
        # Compensate for own motion and wind during flight
        xc = xt - vox * t - vwx * t
        yc = yt - voy * t - vwy * t
        # Displacement from launch point to compensated target
        dx = xc - x0
        dy = yc - y0
        R = norm(dx, dy)

        # Angle solutions for this range
        roots = angle_for_range(v0, R, G)
        if roots is None:
            # Cannot reach: R > v0^2/g. Return best-effort azimuth + None elevation
            azimuth_rad = atan2(dy, dx)
            return {
                "reachable": False,
                "theta_deg": None,
                "azimuth_deg": degrees(azimuth_rad),
                "time_s": None,
                "impact_xy": None,
                "predicted_target_xy": (xt, yt),
                "iterations": it,
                "low_high_deg": None,
                "max_range_m": (v0**2)/G
            }
        low, high = roots
        theta_rad = high if prefer_high_arc else low
        azimuth_rad = atan2(dy, dx)

        vx = v0 * cos(theta_rad)
        if abs(vx) < 1e-6:
            vx = 1e-6
        t_new = R / vx

        if abs(t_new - t) < tol_time:
            t = t_new
            # Compute impact world coordinates to report
            ux, uy = unit(dx, dy)
            # Projectile horizontal travel relative to ship, then add back ship and wind drift
            impact_x = x0 + ux * R + (vox + vwx) * t
            impact_y = y0 + uy * R + (voy + vwy) * t
            return {
                "reachable": True,
                "theta_deg": degrees(theta_rad),
                "azimuth_deg": degrees(azimuth_rad),
                "time_s": t_new,
                "impact_xy": (impact_x, impact_y),
                "predicted_target_xy": (xt, yt),
                "iterations": it,
                "low_high_deg": (degrees(low), degrees(high))
            }
        t = t_new

    # If not converged, still return last estimate
    ux, uy = unit(dx, dy) if 'dx' in locals() else (1.0, 0.0)
    impact_x = x0 + ux * R + (vox + vwx) * t if 'R' in locals() else None
    impact_y = y0 + uy * R + (voy + vwy) * t if 'R' in locals() else None
    return {
        "reachable": True,
        "theta_deg": degrees(theta_rad) if theta_rad is not None else None,
        "azimuth_deg": degrees(azimuth_rad) if azimuth_rad is not None else None,
        "time_s": t,
        "impact_xy": (impact_x, impact_y) if impact_x is not None else None,
        "predicted_target_xy": (xt, yt) if 'xt' in locals() else None,
        "iterations": max_iters,
        "low_high_deg": None
    }

def prompt_run():
    print("\n=== Vector Fleet Solver ===")
    name, v0 = choose_ship()
    print(f"Selected: {name} (v0 = {v0} m/s)")

    print("\n-- Positions (meters) --")
    x0 = get_float("Own ship x0 (m)", 0)
    y0 = get_float("Own ship y0 (m)", 0)
    xt0 = get_float("Target x0 (m)")
    yt0 = get_float("Target y0 (m)")

    print("\n-- Motions (ship speeds in m/s; headings in deg, 0°=East, 90°=North) --")
    v_own = get_float("Own speed (m/s)", 0)
    h_own = get_float("Own heading (deg)", 0)
    v_target = get_float("Target speed (m/s)", 0)
    h_target = get_float("Target heading (deg)", 0)

    print("\n-- Wind (affects projectile horizontally) --")
    v_wind = get_float("Wind speed (m/s)", 0)
    h_wind = get_float("Wind heading (deg, direction wind blows TOWARD)", 0)

    prefer_high = input("Prefer high arc? (y/N): ").strip().lower().startswith("y")

    vox, voy = heading_speed_to_vxvy(v_own, h_own)
    vtx, vty = heading_speed_to_vxvy(v_target, h_target)
    vwx, vwy = heading_speed_to_vxvy(v_wind, h_wind)

    result = solve_fire(
        own_pos=(x0, y0),
        target_pos=(xt0, yt0),
        own_vel=(vox, voy),
        target_vel=(vtx, vty),
        wind_vel=(vwx, vwy),
        v0=v0,
        prefer_high_arc=prefer_high
    )

    print("\n--- Solution ---")
    if not result["reachable"]:
        print("Target is out of range for this muzzle velocity.")
        print(f"Max ideal range: {result['max_range_m']:.1f} m")
        print(f"Suggested azimuth (bearing): {result['azimuth_deg']:.2f}°")
        print(f"Predicted target after flight: ({result['predicted_target_xy'][0]:.2f}, {result['predicted_target_xy'][1]:.2f}) m")
        print(f"Iterations: {result['iterations']}")
        return

    th = result["theta_deg"]
    az = result["azimuth_deg"]
    t = result["time_s"]
    ix, iy = result["impact_xy"]
    tx, ty = result["predicted_target_xy"]
    lh = result["low_high_deg"]

    print(f"Elevation (θ): {th:.2f}°" if th is not None and isfinite(th) else "Elevation (θ): N/A")
    if lh:
        print(f"Low/High angle options: {lh[0]:.2f}° / {lh[1]:.2f}°")
    print(f"Azimuth (bearing in-plane): {az:.2f}°")
    print(f"Time of flight: {t:.2f} s")
    print(f"Impact point (world coords): ({ix:.2f}, {iy:.2f}) m")
    print(f"Predicted target at impact:  ({tx:.2f}, {ty:.2f}) m")
    print(f"Iterations: {result['iterations']}")
    print("\nNOTE: This tool ignores vertical height differences and complex drag; treat wind as constant horizontal drift.")
    print("For torpedoes (very low v0), the range equation may not be appropriate; consider 0° elevation and pure intercept timing.")

if __name__ == "__main__":
    try:
        prompt_run()
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(0)
