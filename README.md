# Vector Fleet ⚓️

A command-line helper program for computing firing solutions in the **Vector Fleet classroom game** — a physics-based activity for learning **vector addition** and **projectile motion**.

Students use this tool to simulate naval battles, inputting ship stats, positions, velocities, and wind to calculate azimuth, elevation, and time-of-flight for their shots.

---

## 🚀 Quick Start

1. **Clone the repository**
   ```bash
   git clone https://github.com/<your-username>/vector-fleet.git
   cd vector-fleet
   ```

2. **Create a virtual environment**

   - **Windows (PowerShell)**
     ```powershell
     py -3 -m venv .venv
     .venv\Scripts\Activate.ps1
     ```

   - **macOS/Linux**
     ```bash
     python3 -m venv .venv
     source .venv/bin/activate
     ```

3. **Open in VS Code**
   ```bash
   code .
   ```
   - Install the recommended Python extension (prompted by VS Code).
   - Ensure `.venv` is selected as your interpreter.

4. **Run the program**
   - Press **F5** (or Run → Start Debugging).
   - Follow the prompts in the integrated terminal.

---

## 🧮 Inputs

- **Ship** (choose from Midway, Alabama, Atlanta, Dolphin, Farragut, or custom muzzle velocity)
- **Own ship position** (x, y in meters)
- **Target position** (x, y in meters)
- **Own & target speeds/headings** (m/s, degrees where 0° = East, 90° = North)
- **Wind speed/heading** (m/s, direction wind blows *toward*)

---

## 📊 Outputs

- Elevation angle(s) (θ) — low/high solutions
- Azimuth (bearing to fire)
- Time of flight (s)
- Impact coordinates
- Predicted target coordinates at impact

---

## 🏫 Classroom Use

- Designed for **Physics Honors** courses to practice projectile motion and vector addition.
- Students run calculations individually or in teams, then compare results.
- Teacher role shifts to facilitator/referee while the tool checks math.

---

## 🔧 Development

- Python 3.10+
- No external dependencies (standard library only)
- Includes VS Code launch configuration for easy classroom use

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).  
Copyright © 2025 Jason Olson

---

## ✨ Future Enhancements

- Batch mode for CSV submissions (auto-check an entire class in one run)
- Scenario files (`.json`) with preset maps, winds, and target motions
- Simplified torpedo/intercept mode for `USS Dolphin`