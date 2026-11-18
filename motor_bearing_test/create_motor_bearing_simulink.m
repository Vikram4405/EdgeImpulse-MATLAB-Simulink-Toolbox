% create_motor_bearing_simulink.m
% Creates 'motor_bearing_sim.slx' on a single flat canvas (NO SUBSYSTEMS).
% Includes interactive Manual Switch toggles, Motor RPM & Fault Severity controls,
% the 3-Axis MEMS Accelerometer C-MEX S-Function, and live Scopes.

fprintf('=== Creating Flat Simulink Model: motor_bearing_sim.slx (No Subsystems) ===\n');

baseDir = fileparts(mfilename('fullpath'));
if isempty(baseDir)
    baseDir = pwd;
end
addpath(baseDir);

modelName = 'motor_bearing_sim';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);

% Configure Fixed-Step Discrete Solver at 1000 Hz (0.001 s)
set_param(modelName, 'SolverType', 'Fixed-step');
set_param(modelName, 'Solver', 'FixedStepDiscrete');
set_param(modelName, 'FixedStep', '0.001');
set_param(modelName, 'StopTime', '10.0');

% -------------------------------------------------------------------------
% 1. Fault Mode Sources & Interactive Manual Switches (Top-Level Flat Canvas)
% -------------------------------------------------------------------------
% 4 Constant blocks representing the 4 physical bearing states:
add_block('simulink/Sources/Constant', [modelName '/Mode_1_HEALTHY'], ...
    'Value', '1', 'Position', [40, 40, 110, 70]);

add_block('simulink/Sources/Constant', [modelName '/Mode_2_OUTER_RACE'], ...
    'Value', '2', 'Position', [40, 105, 110, 135]);

add_block('simulink/Sources/Constant', [modelName '/Mode_3_INNER_RACE'], ...
    'Value', '3', 'Position', [40, 170, 110, 200]);

add_block('simulink/Sources/Constant', [modelName '/Mode_4_UNBALANCE'], ...
    'Value', '4', 'Position', [40, 235, 110, 265]);

% Switch C: Inner Race (3) vs Unbalance (4)
add_block('simulink/Signal Routing/Manual Switch', [modelName '/Toggle_Inner_vs_Unbalance'], ...
    'Position', [170, 185, 210, 245], 'sw', '1');
add_line(modelName, 'Mode_3_INNER_RACE/1', 'Toggle_Inner_vs_Unbalance/1');
add_line(modelName, 'Mode_4_UNBALANCE/1',  'Toggle_Inner_vs_Unbalance/2');

% Switch B: Outer Race (2) vs (Inner/Unbalance)
add_block('simulink/Signal Routing/Manual Switch', [modelName '/Toggle_Outer_vs_Others'], ...
    'Position', [265, 120, 305, 180], 'sw', '1');
add_line(modelName, 'Mode_2_OUTER_RACE/1',         'Toggle_Outer_vs_Others/1');
add_line(modelName, 'Toggle_Inner_vs_Unbalance/1', 'Toggle_Outer_vs_Others/2');

% Switch A (Master): Healthy (1) vs Faulty (2/3/4)
add_block('simulink/Signal Routing/Manual Switch', [modelName '/Toggle_HEALTHY_vs_FAULT'], ...
    'Position', [360, 65, 400, 125], 'sw', '1');
add_line(modelName, 'Mode_1_HEALTHY/1',        'Toggle_HEALTHY_vs_FAULT/1');
add_line(modelName, 'Toggle_Outer_vs_Others/1', 'Toggle_HEALTHY_vs_FAULT/2');

% -------------------------------------------------------------------------
% 2. Motor Speed (RPM) & Fault Severity Inputs
% -------------------------------------------------------------------------
add_block('simulink/Sources/Constant', [modelName '/Motor_Speed_RPM'], ...
    'Value', '1800', 'Position', [250, 230, 330, 265]);

add_block('simulink/Sources/Constant', [modelName '/Fault_Severity'], ...
    'Value', '1.0', 'Position', [250, 300, 330, 335]);

% -------------------------------------------------------------------------
% 3. Core Motor Bearing & 3-Axis MEMS Accelerometer S-Function Block
% -------------------------------------------------------------------------
add_block('simulink/User-Defined Functions/S-Function', ...
    [modelName '/Motor_Bearing_3Axis_Accelerometer'], ...
    'Position', [470, 110, 680, 270], ...
    'FunctionName', 'motor_bearing_accel_sfun', ...
    'Parameters', '0.001');

add_line(modelName, 'Toggle_HEALTHY_vs_FAULT/1', 'Motor_Bearing_3Axis_Accelerometer/1');
add_line(modelName, 'Motor_Speed_RPM/1',         'Motor_Bearing_3Axis_Accelerometer/2');
add_line(modelName, 'Fault_Severity/1',          'Motor_Bearing_3Axis_Accelerometer/3');

% -------------------------------------------------------------------------
% 4. Scopes, Digital Readouts & Logged Outports
% -------------------------------------------------------------------------
% Scope 1: 3-Axis Accelerometer [accX, accY, accZ] (m/s^2)
add_block('simulink/Sinks/Scope', [modelName '/Scope_3Axis_Accelerometer'], ...
    'Position', [770, 95, 820, 145]);
add_line(modelName, 'Motor_Bearing_3Axis_Accelerometer/1', 'Scope_3Axis_Accelerometer/1');

% Scope 2: Bearing Defect Shock Force (Newtons)
add_block('simulink/Sinks/Scope', [modelName '/Scope_Defect_Impact_Force_N'], ...
    'Position', [770, 165, 820, 215]);
add_line(modelName, 'Motor_Bearing_3Axis_Accelerometer/2', 'Scope_Defect_Impact_Force_N/1');

% Digital Display: Live BPFO & BPFI Characteristic Fault Frequencies (Hz)
add_block('simulink/Sinks/Display', [modelName '/Live_BPFO_and_BPFI_Hz'], ...
    'Position', [770, 235, 865, 285]);
add_line(modelName, 'Motor_Bearing_3Axis_Accelerometer/3', 'Live_BPFO_and_BPFI_Hz/1');

% Scope 3: Active Fault Mode (1..4)
add_block('simulink/Sinks/Scope', [modelName '/Scope_Active_Fault_Mode'], ...
    'Position', [470, 35, 515, 75]);
add_line(modelName, 'Toggle_HEALTHY_vs_FAULT/1', 'Scope_Active_Fault_Mode/1');

% Outports for automated MATLAB verification & Edge Impulse Dataset Export
add_block('simulink/Sinks/Out1', [modelName '/out_accel_xyz'], ...
    'Position', [860, 110, 890, 130]);
add_line(modelName, 'Motor_Bearing_3Axis_Accelerometer/1', 'out_accel_xyz/1');

add_block('simulink/Sinks/Out1', [modelName '/out_impact_force'], ...
    'Position', [860, 180, 890, 200]);
add_line(modelName, 'Motor_Bearing_3Axis_Accelerometer/2', 'out_impact_force/1');

add_block('simulink/Sinks/Out1', [modelName '/out_fault_mode'], ...
    'Position', [550, 45, 580, 65]);
add_line(modelName, 'Toggle_HEALTHY_vs_FAULT/1', 'out_fault_mode/1');

% Save flat model
slxPath = fullfile(baseDir, [modelName '.slx']);
save_system(modelName, slxPath);
close_system(modelName);

fprintf('SUCCESS: Created flat Simulink model at %s\n\n', slxPath);
