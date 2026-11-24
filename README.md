# EdgeImpulse-MATLAB-Simulink-Toolbox

A **Universal 2-Way Edge Impulse $\longleftrightarrow$ MATLAB & Simulink Plugin / Toolbox** for end-to-end TinyML & Edge AI engineering across **Automotive, Robotics, Industrial Predictive Maintenance, Aerospace, and Biomedical** systems.

---

## Architecture: Two-Way Closed-Loop TinyML

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                  EDGE IMPULSE <-> MATLAB & SIMULINK TOOLBOX                 │
├──────────────────────────────────────┬──────────────────────────────────────┤
│  PLUGIN 1: EI_Data_Logger            │  PLUGIN 2: EI_Model                  │
│  (Simulink / MATLAB -> Cloud)        │  (Edge Impulse C++ .ZIP -> Simulink) │
│                                      │                                      │
│ • Drag & drop onto ANY Simulink wire │ • 1-Click .ZIP browse & C++20 build  │
│ • Automatically inherits sample rate │   directly inside Simulink block UI  │
│ • Windowed chunking (e.g. every 3s)  │ • Compiles full 59-file DSP + TFLite │
│ • Safety guards (Skip All-Zero,      │   Micro operator kernel suite        │
│   Max Windows limit, Enable/Pause)   │ • Supports Classification, Regression│
│ • Direct HTTPS Cloud Ingestion via   │   Anomaly Detection (K-means/GMM), & │
│   training / testing / split (80/20) │   Object Detection (FOMO/YOLO/SSD)   │
│ • Supports Time-Series, IMU, Temp,   │ • Live Prediction Monitor HUD window │
│   CAN bus, Images (.png), Audio(.wav)│ • Auto RGB888 Image packing in MEX   │
└──────────────────────────────────────┴──────────────────────────────────────┘
```

---

## Repository Structure

```
EdgeImpulse-MATLAB-Simulink-Toolbox/
│
├── toolbox/                              # Core Universal 2-Way Plugin & Simulink Library
│   ├── EdgeImpulse_Library.slx           # Drag-and-drop Simulink Library (EI_Data_Logger & EI_Model)
│   ├── slblocks.m                        # Registers "Edge Impulse TinyML" in Simulink Library Browser
│   ├── create_ei_simulink_library.m      # Programmatic builder for EdgeImpulse_Library.slx
│   ├── EdgeImpulseUploader.m             # Plugin 1 class: Time-series, Images (.png), Audio (.wav), auto-split
│   ├── ei_logger_msfun.m                 # Plugin 1 S-Function: Solver-adaptive windowed Simulink logger
│   ├── ei_import_model.m                 # Plugin 2 compiler: Auto-imports & compiles ANY Edge Impulse C++ .ZIP
│   ├── ei_universal_mex.cpp              # Plugin 2 C++ MEX gateway: Classification, Regression, Anomaly, OD, Images
│   ├── ei_classifier_msfun.m             # Plugin 2 S-Function: Live Simulink model block + Prediction Monitor HUD
│   ├── ei_browse_and_import_callback.m   # Simulink Mask callback for 1-click .ZIP selection & build
│   └── test_step_a_universal_inputs.m    # Multi-modality verification suite (Temp, 6-Axis IMU, Images, Audio)
│
├── ev_battery_test/                      # Example 1: Physical Simscape EV Battery BMS Fault Detection
│   ├── ev_battery_sim.slx                # Flat Simscape 27 Ah NMC cell + liquid cooling + Arrhenius runaway
│   ├── create_ev_battery_simulink.m      # Programmatic generator for flat Simscape EV battery model
│   ├── verify_ev_battery_simulink.m      # 4-mode 60s verification (Drive, Fast Charge, Short, Runaway)
│   └── ev_battery_verification.png       # Verified 4-mode Voltage, Current & Temperature curves
│
├── motor_bearing_test/                   # Example 2: Industrial Motor Bearing Predictive Maintenance (Simscape)
│   ├── motor_bearing_sim.slx             # Flat Simscape physical model (Motor, Bearing, Unbalanced Load, Accel)
│   ├── create_library_bearing_simulink.m # Programmatic generator for flat Simscape bearing model
│   ├── verify_library_bearing_simulink.m # 4-state verification & FFT spectrum generator
│   └── simscape_library_verification.png # Verified 4-state accelerometer waveforms & FFT spectra
│
├── coffee_machine_test/                  # Example 3: 3-Axis Vibration Stage Classification
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
   * **Label Name:** e.g. `Normal_Drive`, `Fast_Charging`, `Short_Circuit`, `Thermal_Runaway`
   * **Sampling Window Size in Seconds:** e.g. `3` or `5` (uploads one clean window every $N$ seconds)
   * **Max Windows to Upload per Run:** e.g. `10` (`0` = unlimited)
   * **Dataset Split:** `training`, `testing`, or `split` (automatic 80/20 split via `/api/split/files`)
   * **Edge Impulse API Key:** Paste your project's `ei_...` API key (from *Edge Impulse Studio $\rightarrow$ Dashboard $\rightarrow$ Keys*) for automatic live cloud upload, or leave blank to export `.csv` and `.json` files locally.
3. Click **Run (▶)** in Simulink—each completed window uploads directly to your Edge Impulse project!

### 3. Plugin 2 — Import & Run Trained Edge Impulse Models in Simulink (`EI_Model`)
1. In Edge Impulse Studio, train your impulse (Classification, Regression, Anomaly Detection, or Object Detection), go to **Deployment $\rightarrow$ C++ library $\rightarrow$ Build**, and download the `.zip` file.
2. Drag **`EI_Model`** from `EdgeImpulse_Library` into your Simulink model.
3. Double-click **`EI_Model`** and click **`[ Browse & Build Edge Impulse .ZIP Model... ]`** (or run `ei_import_model('path/to/model.zip')` in MATLAB).
   * Automatically extracts the `.zip`, applies MSVC C++20 portability patches (including K-means `anomaly_metadata.h` support), detects input channels/window/frequency/labels from `model_metadata.h`, and compiles the native MEX binary (`ei_universal_mex`).
4. Connect your live signal wire into **`Signal_In`** and click **Run (▶)**:
   * The **Live Prediction Monitor HUD Window** opens automatically, displaying the **predicted text label** (e.g. `>> THERMAL_RUNAWAY <<`), confidence percentage, rolling input window, and live class probability bars!
   * Output ports **`Class_ID`**, **`Confidence`**, and **`Scores`** drive standard Simulink `Scope` and `Display` blocks, and direct MATLAB calls (`res = ei_universal_mex(x)`) also expose `.anomaly` (K-means/GMM) and `.bounding_boxes` (FOMO/YOLO/SSD).
