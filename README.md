# EV Powertrain Dynamics and Energy Analysis: UDDS Simulation

## Abstract
This project presents a deterministic longitudinal simulation of a Battery Electric Vehicle (BEV) to quantify energy consumption and range under the EPA Urban Dynamometer Driving Schedule (UDDS). The model integrates road-load physics with hardware-specific constraints, including a dual-region motor torque-speed envelope and non-linear efficiency mapping. By accounting for parasitic accessory loads and regenerative braking effectiveness, the simulation provides a high-fidelity estimation of State of Charge (SOC) depletion and net energy flow.

---

## Contents
1. [Introduction](#1-introduction)
2. [Methodology](#2-methodology)
3. [Results of Task](#3-results-of-task)
4. [Conclusion](#4-conclusion)

---

## 1. Introduction
### 1.1 Objective
The objective is to evaluate the energy demand of an 1800 kg EV by simulating real-world urban driving conditions. The project aims to bridge the gap between theoretical mechanical work and actual battery discharge by modeling the electrical and mechanical losses inherent in the powertrain.

### 1.2 Tools and Libraries Used
* **MATLAB:** Core environment for vector-based numerical differentiation and integration.
* **UDDS Dataset:** Time-series velocity profile (1369 seconds) used as the kinematic input.
* **Numerical Methods:** Implementation of the Trapezoidal rule (`trapz`) for energy accumulation and `diff` for instantaneous acceleration.

### 1.3 Key Features
* **Equivalent Mass Modeling:** Integration of a rotational inertia factor ($\delta = 0.08$) to account for the angular momentum of rotating components.
* **Propulsion Constraints:** Implementation of a 250 Nm constant-torque region and an 80 kW constant-power field-weakening region.
* **Non-Linear Efficiency Map:** Quadratic loss modeling based on normalized speed ($\omega_n$) and torque ($T_n$).
* **Energy Recovery Logic:** Regen braking capped at 120 Nm with a 1.5 m/s ($5.4$ km/h) low-speed cut-off to reflect back-EMF limitations.

### 1.4 Flowchart
```text
       Start
         ↓
Import UDDS Drive Cycle Data
         ↓
   Convert Speed Profile
         ↓
   Compute Acceleration
         ↓
Calculate Vehicle Forces
         ↓
  Compute Tractive Power
         ↓
 Calculate Battery Power
         ↓
Evaluate Regenerative Braking
         ↓
 Compute Energy Consumption
         ↓
Update Battery State of Charge
         ↓
Generate Plots & Performance Metrics
         ↓
        End
```
---

## 2. Methodology
The simulation utilizes a "backward-facing" approach where the velocity trace dictates the required torque at the wheels. 

1.  **Force Balance:** The total tractive effort ($F_{total}$) is calculated by summing Aerodynamic Drag, Rolling Resistance, and Inertial Forces ($m_{eq} \cdot a$). 
2.  **Wheel-to-Motor Translation:** Wheel speed and torque are translated to the motor shaft via a fixed gear ratio, respecting the maximum torque (250 Nm) and power (80 kW) limits.
3.  **Efficiency Losses:** During drive phases, battery power is calculated as $P_{mech} / \eta_{drive}$. During braking, recovered power is $P_{mech} \cdot \eta_{regen}$. Both efficiency values vary dynamically based on the motor's operating point.
4.  **Energy Accounting:** The battery model tracks energy in Watt-hours (Wh). It includes a constant 900W accessory load to represent HVAC and low-voltage systems.

---

## 3. Results of Task

### 3.1 Vehicle & Drive Cycle Metrics
* **Vehicle Mass:** 1800 kg
* **Total Distance Travelled:** 11.990 km
* **Average Speed:** 31.51 km/h
* **Drive Cycle Duration:** 1369 s

### 3.2 Energy Consumption Analysis
* **Wheel Mechanical Energy:** 815.62 Wh
* **Battery Energy Drawn:** 2222.15 Wh
* **Battery Energy Regenerated:** 644.54 Wh
* **Net Battery Energy Used:** 1577.61 Wh
* **Specific Energy Consumption:** 131.57 Wh/km

### 3.3 Power Performance
* **Maximum Wheel Power:** 43.32 kW
* **Maximum Battery Drive Power:** 50.08 kW
* **Maximum Regen Power:** 26.51 kW

### 3.4 Regenerative Braking Effectiveness
* **Mechanical Braking Energy:** 994.54 Wh
* **Regen Energy Recovery:** 64.81 %

### 3.5 Battery State of Charge (SOC)
* **Initial SOC:** 80.00 %
* **Final SOC:** 76.16 %
* **Net SOC Change:** 3.840 %

### 3.6 Energy Flow Analysis
* **Drive Energy From Battery:** 2222.15 Wh
* **Recovered Regen Energy:** 644.54 Wh
* **Accessory Consumption:** 342.25 Wh
* **Net Battery Consumption:** 1577.61 Wh

### 3.7 Estimated EV Range
* **Estimated Range (UDDS Based):** 228.0 km

---

## 4. Conclusion

### 4.1 Evaluation of Results
The simulation reveals that the net energy consumption (**131.57 Wh/km**) is significantly influenced by conversion efficiencies and auxiliary loads. While the vehicle only required **815.62 Wh** of mechanical energy to complete the distance, the battery supplied **2222.15 Wh**. This "Efficiency Gap" is partially mitigated by the regenerative braking system, which recovered **644.54 Wh** (**64.81%** of available braking energy). The accessory load accounted for **21.7%** of the net energy used, proving that auxiliary systems are a major factor in urban range depletion.

### 4.2 Learning Outcomes
The development of this simulation provided a practical framework for transitioning from theoretical mechanical design to integrated mechatronic systems engineering. Key technical competencies gained include:

* **Discretization and Numerical Integration:** Learned to convert continuous velocity traces into discrete acceleration and power data points using the Trapezoidal rule.
* **System-Level Modeling:** Gained experience in "backward-facing" logic, where requirements at the wheel dictate the electrical load on the battery.
* **Efficiency Map Implementation:** Developed a method to integrate non-linear loss models, moving beyond ideal 1:1 conversion ratios to capture the quadratic nature of motor/inverter losses.
* **Constraint-Based Logic:** Implemented hardware-specific limits, such as field-weakening regions and low-speed regen cut-offs, to reflect physical hardware limitations.
* **Energy Management:** Understood the impact of parasitic auxiliary loads (900W) on net vehicle range, especially in low-speed urban cycles like the UDDS.

---
