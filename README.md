# EdgeImpulse-MATLAB-Simulink-Toolbox

A **Universal 2-Way Edge Impulse $\longleftrightarrow$ MATLAB & Simulink Plugin / Toolbox** for end-to-end TinyML & Edge AI engineering across **Automotive, Robotics, Industrial Predictive Maintenance, Aerospace, and Biomedical** systems.

---

## Architecture: Two-Way Closed-Loop TinyML

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                  EDGE IMPULSE <-> MATLAB & SIMULINK TOOLBOX                 │
├──────────────────────────────────────┬──────────────────────────────────────┤
│  PLUGIN 1: EI_Data_Logger            │  PLUGIN 2: EI_Classifier             │
│  (Simulink / MATLAB -> Cloud)        │  (Edge Impulse C++ .ZIP -> Simulink) │
│                                      │                                      │
│ • Drag & drop onto ANY Simulink wire │ • 1-Click .ZIP browse & C++20 build  │
│ • Automatically inherits sample rate │   directly inside Simulink block UI  │
│ • Windowed chunking (e.g. every 5s)  │ • Compiles full 59-file DSP + TFLite │
│ • Safety guards (Skip All-Zero,      │   Micro operator kernel suite        │
│   Max Windows limit, Enable/Pause)   │ • Live Prediction Monitor HUD window │
│ • Direct HTTPS Cloud Ingestion via   │   showing predicted TEXT LABEL,      │
│   Edge Impulse API Key (x-api-key)   │   confidence %, & probability bars!  │
│ • Supports Time-Series, IMU, Temp,   │ • Solver-adaptive across ode45,      │
│   CAN bus, Images (.png), Audio(.wav)│   ode23t (Simscape), & Fixed-Step    │
└──────────────────────────────────────┴──────────────────────────────────────┘
```

---

## Repository Structure

```
EdgeImpulse-MATLAB-Simulink-Toolbox/
│
├── toolbox/                              # Core Universal 2-Way Plugin & Simulink Library
│   ├── EdgeImpulse_Library.slx           # Drag-and-drop Simulink Library (EI_Data_Logger & EI_Classifier)
│   ├── slblocks.m                        # Registers "Edge Impulse TinyML" in Simulink Library Browser
│   ├── create_ei_simulink_library.m      # Programmatic builder for EdgeImpulse_Library.slx
│   ├── EdgeImpulseUploader.m             # Plugin 1 class: Time-series, Images (.png), Audio (.wav) -> Cloud
│   ├── ei_logger_msfun.m                 # Plugin 1 S-Function: Solver-adaptive windowed Simulink logger
│   ├── ei_import_model.m                 # Plugin 2 compiler: Auto-imports & compiles ANY Edge Impulse C++ .ZIP
│   ├── ei_universal_mex.cpp              # Plugin 2 C++ MEX gateway: Dynamic channels, windows & classes
│   ├── ei_classifier_msfun.m             # Plugin 2 S-Function: Live Simulink classifier + Prediction Monitor HUD
│   ├── ei_browse_and_import_callback.m   # Simulink Mask callback for 1-click .ZIP selection & build
│   └── test_step_a_universal_inputs.m    # Multi-modality verification suite (Temp, 6-Axis IMU, Images, Audio)
│
├── motor_bearing_test/                   # Example 1: Industrial Motor Bearing Predictive Maintenance (Simscape)
│   ├── motor_bearing_sim.slx             # Flat Simscape physical model (Motor, Bearing, Unbalanced Load, Accel)
│   ├── create_library_bearing_simulink.m # Programmatic generator for flat Simscape bearing model
│   ├── verify_library_bearing_simulink.m # 4-state verification & FFT spectrum generator
│   └── simscape_library_verification.png # Verified 4-state accelerometer waveforms & FFT spectra
│
├── coffee_machine_test/                  # Example 2: 3-Axis Vibration Stage Classification
│   ├── coffee_machine_sim.slx            # Streaming 3-axis Simulink classifier model
│   ├── coffee_machine_interactive.slx    # Interactive toggle-switch Simulink model
│   ├── coffee_machine_gui.m              # Standalone MATLAB live dashboard GUI
│   └── SCENARIO_COFFEE_MACHINE.md        # Detailed scenario documentation
│
├── .gitignore
└── README.md
```

---

## Quick Start Guide

### 1. Add the Toolbox to MATLAB
```matlab
addpath('C:\path\to\EdgeImpulse-MATLAB-Simulink-Toolbox\toolbox');
open_system('EdgeImpulse_Library');
```

### 2. Plugin 1 — Collect & Upload Training Data from Simulink (`EI_Data_Logger`)
1. Drag **`EI_Data_Logger`** from `EdgeImpulse_Library` into any Simulink model and connect an output wire from any sensor or signal block into it.
2. Double-click **`EI_Data_Logger`** to configure:
   * **Label Name:** e.g. `Sine_Wave`, `Square_Wave`, `outer_race_fault`, `pothole`
   * **Sampling Window Size in Seconds:** e.g. `5` or `10` (uploads one clean window every $N$ seconds)
   * **Max Windows to Upload per Run:** e.g. `10` (`0` = unlimited)
   * **Edge Impulse API Key:** Paste your project's `ei_...` API key (from *Edge Impulse Studio $\rightarrow$ Dashboard $\rightarrow$ Keys*) for automatic live cloud upload, or leave blank to export `.csv` and `.json` files locally.
3. Click **Run (▶)** in Simulink—each completed window uploads directly to your Edge Impulse project!

### 3. Plugin 2 — Import & Run Trained Edge Impulse Models in Simulink (`EI_Classifier`)
1. In Edge Impulse Studio, train your impulse, go to **Deployment $\rightarrow$ C++ library $\rightarrow$ Build**, and download the `.zip` file.
2. Drag **`EI_Classifier`** from `EdgeImpulse_Library` into your Simulink model.
3. Double-click **`EI_Classifier`** and click **`[ Browse & Build Edge Impulse .ZIP Model... ]`** (or run `ei_import_model('path/to/model.zip')` in MATLAB).
   * Automatically extracts the `.zip`, detects the input channels, window size, sampling frequency ($Hz$), and class names from `model_metadata.h`, and compiles the native MEX binary (`ei_universal_mex`).
4. Connect your live signal wire into **`Signal_In`** and click **Run (▶)**:
   * The **Live Prediction Monitor HUD Window** opens automatically, displaying the **predicted text label** (e.g. `>> SINE_WAVE <<`), confidence percentage, rolling input window, and live class probability bars!
   * Output ports **`Class_ID`**, **`Confidence`**, and **`Scores`** also drive standard Simulink `Scope` and `Display` blocks.
