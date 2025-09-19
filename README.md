# MultiTrip4TechniciansDrones

**MILP model for multi-trip scheduling of technicians and drones**

This repository implements a mixed-integer linear programming (MILP) model for a **medical sampling service** in which human technicians and drones cooperate to visit a set of customers over **multiple trips**. Technicians have **time-dependent speeds** (to model congestion); drones have **payload and energy limits**. The goal is to **minimize makespan** (time when the last resource returns to the depot) while observing cold-chain (waiting-time) constraints.

> Model details are summarized from the PDF in this repo:
> *Parallel_Truck_Drone_MILP_Section.pdf — “Medical Sampling Service System with Variable Technician Speed and Drone Energy Consumption Problem – Model Formulation Section”, Pham Cong Hoang (Sept 17, 2025).*

---

## ✨ Key Features

- **Integrated technician–drone routing & scheduling**
- **Multi-trip** routing for each technician and each drone
- **Technician speeds vary by time interval** (congestion discretization)
- **Drone energy and payload constraints** (takeoff/cruise/landing velocities, energy coefficients β and γ)
- **Cold-chain waiting-time limits** per sample
- **Objective:** minimize **makespan** (C_max)

---

## 🧠 Problem Overview (short)

- Customers are split into:
  - `CT`: must be served by a technician
  - `CA`: can be served by either technician or drone
- Each technician/drone may perform **multiple trips** from/to the depot.
- Technician travel times depend on departure time (via time marks T_l and factors θ_l).
- Drone flights use fixed speeds by phase and accumulate energy; each trip is bounded by payload and energy.
- **Decision variables** (high level):
  - Technician/drone arc selections for each trip
  - Service start/departure/return times
  - Interpolation variables linking times to the time-mark grid
  - Payload and energy accumulators for drones
- **Objective:** minimize C_max (makespan).

---

## 📦 Repository Structure

```
.
├── config.json                         # Speeds/energy/payload/time-mark settings
├── mul_trip.py                         # Main (Docplex) model and run script
├── Parallel_Truck_Drone_MILP_Section.pdf  # Model formulation (paper section)
└── instance/                           # (Put your instance .txt files here)
```

---

## ⚙️ Requirements & Installation

- Python 3.9+
- IBM **CPLEX** (Community or commercial) and **docplex**
- Other standard libs: `math`, `json`, `time`

```bash
# Create an environment (optional)
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate

# Install docplex
pip install docplex
```

> You will also need a local CPLEX installation or access to CPLEX cloud.
> Set environment variables if needed so docplex can find CPLEX.

---

## 🔧 Configuration (`config.json`)

The default config in this repo looks like:

```json
{
  "technician": {
    "V_max (m/s)": 15.557,
    "M_t (kg)": 400,
    "T (hour)": {
      "0-1": 0.7, "1-2": 0.4, "2-3": 0.6, "3-4": 0.7,
      "4-5": 0.8, "5-6": 0.9, "6-7": 1.0, "7-8": 0.7,
      "8-9": 0.6, "9-10": 0.5, "10-11": 0.7, "11-12": 0.8
    }
  },
  "drone": {
    "takeoffSpeed [m/s]": 20,
    "cruiseSpeed [m/s]": 30,
    "landingSpeed [m/s]": 20,
    "cruiseAlt [m]": 50,
    "capacity [kg]": 2.27,
    "batteryPower [Joule]": 562990,
    "speed_type": "low",
    "range": "high",
    "beta(w/kg)": 210.8,
    "gama(w)": 181.2,
    "FixedTime (s)": 1400,
    "FixedDistance (m)": 19312.128
  }
}
```

---

## 🗂️ Instance File Format

Example:

```
TECHNICIANS 2
DRONES 1
CUSTOMERS 3

0.0  1.0  0.50  1  120   0    3600
3.2  2.3  1.10  0  180  150   5400
5.0  4.0  0.80  0  240  180
```

---

## ▶️ Running

```bash
python mul_trip.py
```

Edit `mul_trip.py` to set paths for `config.json`, instance file, and solver time limit.

---

## 📄 Citation

If you use this code or formulation, please cite the accompanying formulation section:

> Pham Cong Hoang (2025). *Medical Sampling Service System with Variable Technician Speed and Drone Energy Consumption Problem – Model Formulation Section.* (PDF in repo).

---

## 📝 License

MIT (or update as needed).
