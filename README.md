# BUFD-Q: Boltzmann-Guided Q-Learning for Optimized 3D UAV Deployment in Flood-Affected Wireless Networks

> **Paper:** *BUFD-Q: Boltzmann-Guided Q-Learning for Optimized 3D UAV Deployment in Flood-Affected Wireless Networks*  
> **Authors:** Swateya Gupta, Meenu Rani Dey  
> **Affiliation:** Indian Institute of Technology Guwahati, Assam, India  
> **Contact:** swateya.gupta@iitg.ac.in · rmeenu@iitg.ac.in

---

## Overview

Floods frequently disable terrestrial base stations, creating critical communication blackouts precisely when emergency coordination is most needed. **BUFD-Q** is a reinforcement learning framework that deploys a fleet of UAVs (Unmanned Aerial Vehicles) as aerial 5G base stations in such post-disaster environments.

The key idea is simple: instead of placing UAVs statically (as GA, PSO, or K-Means do), BUFD-Q lets UAVs *learn* optimal 3D positions through interaction with the environment, guided by a Boltzmann exploration strategy that balances exploration and exploitation in a principled, value-aware manner.

### Key Results (vs. PSO, GA, K-Means, Greedy, ε-greedy Q-learning baselines)

| Metric | BUFD-Q Improvement |
|---|---|
| Energy Consumption | **15.73% lower** |
| Overlapping Users | **11.35% fewer** |
| Coverage | Within **0.73%** of best baseline |

---

## Repository Structure

```
BUFD-Q/
├── README.md
├── environments/
│   ├── MultiUAVEnv.m          # Main RL environment (4 UAVs, 300 users, 500×500m)
│   └── Final_code_env.m       # Training-ready environment wrapper
├── training/
│   └── Final_code_training.m  # BUFD-Q algorithm: Boltzmann Q-learning + logging
├── setup/
│   ├── User_distribution.m    # Clustered user spatial distribution generator
│   ├── User_Request_data.m    # Dynamic user request simulation (per timestep)
│   └── calculateMaxCoverageRadius.m  # Derives 136m coverage radius from path loss
└── analysis/
    ├── COV_RANGE.m            # Coverage radius vs. altitude plots (4 environments)
    └── Parameters.m           # UAV deployment parameter reference script
```

---

## System Model

- **Area:** 500 × 500 m²
- **Users (N):** 300, distributed in 4 spatial clusters + 10% random spread
- **UAVs (U):** 4, acting as mobile 5G aerial base stations
- **Altitude range:** 15 m – 100 m
- **Coverage radius:** 136 m (derived from path loss threshold of 125 dB at 2 GHz)
- **Episode length:** 30 timesteps (1 minute each)
- **Energy model:** Trajectory-cost based (0.2 J/meter)

### Channel Model

The air-to-ground path loss combines LoS and NLoS components:

```
L_avg = p_LoS · L_LoS + p_NLoS · L_NLoS

L_LoS  = FSPL + L_rain + L_duct
L_NLoS = L_LoS + L_reflection + L_shadow
```

SINR-based coverage determines whether a user is truly served, not just within range.

---

## BUFD-Q Algorithm

BUFD-Q uses a **centralized Q-table** with **Boltzmann (softmax) exploration**. Unlike ε-greedy, Boltzmann selects actions proportionally to their Q-values, making exploration more informed and stable in large action spaces.

```
π(a|s) = exp(Q(s,a) / τ) / Σ exp(Q(s,a') / τ)
```

Temperature τ decays exponentially across episodes, shifting the policy from exploration to exploitation.

### Reward Function

The multi-objective reward for each UAV integrates four components:

```
R(t) = w1·R_Coverage + w2·R_Overlap + w3·R_Energy + w4·R_Proximity
```

- **Coverage reward:** Fraction of users served
- **Overlap penalty:** Penalizes UAVs covering the same users (interference control)
- **Energy penalty:** Discourages unnecessary movement
- **Proximity reward:** Encourages staying within backhaul range of the ground base station

---

## Getting Started

### Requirements

- MATLAB R2021a or later
- MATLAB Reinforcement Learning Toolbox (for `rl.env.MATLABEnvironment`)
- No additional toolboxes required for standalone scripts

### Quick Start

**Step 1 — Generate user locations and requests:**
```matlab
run('setup/User_distribution.m')      % Creates userLocations.mat
run('setup/User_Request_data.m')      % Creates userRequests.mat
```

**Step 2 — Verify coverage radius:**
```matlab
run('setup/calculateMaxCoverageRadius.m')
% Expected output: Maximum coverage radius for UAV: 136.00 meters
```

**Step 3 — Run BUFD-Q training:**
```matlab
env = Final_code_env;                  % Initialize environment
Final_code_training(env);             % Run training (100 episodes by default)
% Saves TrainedQTables.mat and generates 4 training plots
```

**Step 4 — (Optional) Analyse coverage vs. altitude:**
```matlab
run('analysis/COV_RANGE.m')
```

### Training Configuration

The optimal hyperparameters reported in the paper are set by default in `Final_code_training.m`:

| Parameter | Value |
|---|---|
| Learning rate (α) | 0.07 |
| Discount factor (γ) | 0.95 |
| Training episodes | 100–1000 |
| Max steps per episode | 30 |
| Initial temperature (τ₀) | 1.0 |
| Min temperature (τ_min) | 0.1 |

---

## Output

After training, `Final_code_training.m` automatically generates:

1. **Reward vs. Episodes** — Convergence of the cumulative reward
2. **Coverage vs. Episodes** — User coverage improvement over training
3. **Overlapped Users vs. Episodes** — Interference reduction over training
4. **Energy Consumed vs. Episodes** — Energy efficiency over training
5. **`TrainedQTables.mat`** — Saved Q-table for deployment or further evaluation

---

## File Descriptions

| File | Role |
|---|---|
| `MultiUAVEnv.m` | Core RL environment: 4 UAVs, 300 users, path loss, SINR, energy, reward |
| `Final_code_env.m` | Thin wrapper around `MultiUAVEnv` used during training runs |
| `Final_code_training.m` | BUFD-Q training loop: Boltzmann policy, Q-table updates, metric logging, plot generation |
| `User_distribution.m` | Generates 300-user positions across 4 clusters in a 500×500m area |
| `User_Request_data.m` | Simulates dynamic user requests (220–300 active users per minute, 30 minutes) |
| `calculateMaxCoverageRadius.m` | Iteratively solves for the maximum radius where average path loss ≤ 125 dB |
| `COV_RANGE.m` | Plots coverage radius vs. altitude for Suburban, Urban, DenseUrban, HighriseUrban |
| `Parameters.m` | Reference script computing per-UAV area, radius, and user density |

---

## Citation

If you use this code in your research, please cite:

```bibtex (to be updated)
@inproceedings{gupta2025bufdq,
  title     = {BUFD-Q: Boltzmann-Guided Q-Learning for Optimized 3D UAV Deployment in Flood-Affected Wireless Networks},
  author    = {Gupta, Swateya and Dey, Meenu Rani},
  institution = {Indian Institute of Technology Guwahati},
  year      = {2026}
}
```

---


## Acknowledgements

This work was carried out at the Indian Institute of Technology Guwahati. The channel model follows the air-to-ground path loss framework established in prior literature on UAV communications (Al-Hourani et al., IEEE WCL 2014; Yan et al., IEEE Access 2019).
