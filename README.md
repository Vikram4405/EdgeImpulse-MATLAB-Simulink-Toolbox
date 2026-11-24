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
│ • Windowed chunking (e.g. 3s/5s/10s) │ • Compiles full DSP + TFLite Micro   │
│ • Auto 80/20 split (/api/split/files)│   + K-Means / Object Detection SDK   │
│ • Safety guards (Skip All-Zero,      │ • Classification, Regression,        │
│   Max Windows limit, Enable/Pause)   │   Anomaly Detection & Object Detect  │
│ • Direct HTTPS Cloud Ingestion via   │ • Auto-packs MATLAB uint8 RGB images │
│   Edge Impulse API Key (x-api-key)   │   & returns bounding_boxes structs   │
│ • Supports Time-Series, IMU, Temp,   │ • Live Prediction Monitor HUD window │
│   CAN bus, Images (.png), Audio(.wav)│ • Solver-adaptive (ode45/ode23t/fix) │
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
│   ├── EdgeImpulseUploader.m             # Plugin 1 class: Time-series, Images (.png), Audio (.wav) -> Cloud (training/testing/split)
│   ├── ei_logger_msfun.m                 # Plugin 1 S-Function: Solver-adaptive windowed Simulink logger
│   ├── ei_import_model.m                 # Plugin 2 compiler: Auto-imports & compiles ANY Edge Impulse C++ .ZIP (with MSVC patches)
│   ├── ei_universal_mex.cpp              # Plugin 2 C++20 MEX gateway: Classification, Regression, Anomaly, Object Detection & RGB packing
│   ├── ei_classifier_msfun.m             # Plugin 2 S-Function: Live Simulink inference + Prediction Monitor HUD
│   ├── ei_browse_and_import_callback.m   # Simulink Mask callback for 1-click .ZIP selection & build
│   └── test_step_a_universal_inputs.m    # Multi-modality verification suite (Temp, 6-Axis IMU, Images, Audio)
│
├── ev_battery_test/                      # Featured Example: Real-Time EdgeAI EV BMS (Simscape Electrical + Thermal)
│   ├── ev_battery_sim.slx                # 4-state EV NMC battery + Arrhenius thermal runaway model
│   ├── create_ev_battery_simulink.m      # Programmatic Simscape EV BMS builder
│   ├── verify_ev_battery_simulink.m      # 4-state verification & waveform generator
│   └── ev_battery_verification.png       # Verified 4-state V/I/T waveforms
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

## Featured Standalone Showcase

* **[EdgeAI-EV-BMS-Simulink](https://github.com/Vikram4405/EdgeAI-EV-BMS-Simulink)** — Full zero-hardware EV Battery Management System using this toolbox to simulate `Normal_Drive`, `Fast_Charging`, `Short_Circuit`, and `Thermal_Runaway` in Simscape, stream 24 minutes of 3-channel telemetry to Edge Impulse, and run live 4-class + K-Means anomaly inference back inside Simulink.

---

## Quick Start Guide

### 1. Add the Toolbox to MATLAB
```matlab
addpath('C:\path\to\EdgeImpulse-MATLAB-Simulink-Toolbox\toolbox');
open_system('EdgeImpulse_Library');
```

### 2. Plugin 1 — Collect & Upload Training Data (`EI_Data_Logger`)
1. Drag **`EI_Data_Logger`** from `EdgeImpulse_Library` into any Simulink model and connect any sensor or signal wire into it.
2. Double-click **`EI_Data_Logger`** to configure:
   * **Label Name:** e.g. `Normal_Drive`, `Thermal_Runaway`, `outer_race_fault`
   * **Sampling Window Size (s):** e.g. `3`, `5`, or `10`
   * **Dataset Split:** `training`, `testing`, or `split` *(automatic 80/20 split via `/api/split/files`)*
   * **Edge Impulse API Key:** Paste your project's `ei_...` API key (*Edge Impulse Studio $\rightarrow$ Dashboard $\rightarrow$ Keys*) for live cloud upload, or leave blank for local `.csv`/`.json` export.
3. Click **Run (▶)** in Simulink—each completed window uploads directly to your Edge Impulse project!

### 3. Plugin 2 — Import & Run Trained Edge Impulse Models (`EI_Model`)
1. In Edge Impulse Studio, export your impulse via **Deployment $\rightarrow$ C++ library $\rightarrow$ Build** and download the `.zip`.
2. Drag **`EI_Model`** from `EdgeImpulse_Library` into your Simulink model.
3. Double-click **`EI_Model`** and click **`Browse & Build Edge Impulse .ZIP Model...`** (or run `ei_import_model('path/to/model.zip')` in MATLAB).
   * Automatically extracts the `.zip`, parses `model_metadata.h`, patches MSVC C++20 compatibility, and compiles `ei_universal_mex`.
4. Connect your live signal wire into **`Signal_In`** and click **Run (▶)**:
   * The **Live Prediction Monitor HUD** opens automatically, showing the predicted text label, confidence %, rolling input window, and live class probability bars.
   * Output ports **`Class_ID`**, **`Confidence`**, and **`Scores`** drive standard Simulink scopes, displays, or closed-loop controllers.

### 4. Object Detection (FOMO / YOLO / SSD) & Anomaly Detection in MATLAB
Calling the compiled MEX directly from MATLAB returns the full rich result struct, including **`anomaly`** scores and dynamic **`bounding_boxes`**:

```matlab
% Pass a standard MATLAB uint8 RGB image (H x W x 3) directly — auto-packed to 0xRRGGBB!
img = imresize(imread('test_frame.jpg'), [96, 96]);
res = ei_universal_mex(img);

% Draw every detected bounding box (x, y, width, height, label, value)
imshow(img); hold on;
for k = 1:numel(res.bounding_boxes)
    bb = res.bounding_boxes(k);
    rectangle('Position', [bb.x, bb.y, bb.width, bb.height], 'EdgeColor', 'g', 'LineWidth', 2);
    text(bb.x, bb.y - 6, sprintf('%s (%.0f%%)', bb.label, bb.value*100), 'Color', 'y', 'FontWeight', 'bold');
end
```
