% create_library_bearing_simulink.m
% Builds 'motor_bearing_sim.slx' using 100% built-in Simscape & Simulink
% library blocks on a single flat canvas (NO SUBSYSTEMS, NO MEX):
%   - ee_lib/Electromechanical/Motor & Drive (System Level)
%   - fl_lib/Mechanical/Rotational Elements/Rotational Spring & Damper (Flexible Shaft Coupling)
%   - sdl_lib/Couplings & Drives/Bearing
%   - sdl_lib/Inertias & Loads/Unbalanced Load
%   - fl_lib/Mechanical/Mechanisms/Wheel and Axle (Rotational -> Translational bridge)
%   - ee_lib/Sensors & Transducers/Accelerometer
%   - fl_lib/Electrical/Electrical Sensors/Voltage Sensor
%   - fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source
%   - Standard Simulink Pulse Generators, Sine Waves & Manual Switches

fprintf('=== Building Pure Library Simscape Motor Bearing + Accelerometer Model ===\n');

baseDir = fileparts(mfilename('fullpath'));
if isempty(baseDir)
    baseDir = pwd;
end
addpath(baseDir);
addpath(fullfile(fileparts(baseDir), 'toolbox'));

load_system('simulink');
load_system('sdl_lib');
load_system('ee_lib');
load_system('fl_lib');
load_system('nesl_utility');
if exist(fullfile(fileparts(baseDir), 'toolbox', 'EdgeImpulse_Library.slx'), 'file') == 4
    load_system('EdgeImpulse_Library');
end

modelName = 'motor_bearing_sim';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);

% Use ode23t (trapezoidal stiff solver - standard for Simscape physical networks)
set_param(modelName, 'SolverType', 'Variable-step');
set_param(modelName, 'Solver', 'ode23t');
set_param(modelName, 'MaxStep', '0.0005');
set_param(modelName, 'StopTime', '1.0');

% =========================================================================
% PART 1: SIMSCAPE PHYSICAL NETWORK (Motor, Flexible Shaft, Bearing, Unbalanced Load, Accelerometer)
% =========================================================================

% 1. Solver Configuration & Mechanical Rotational Reference
add_block('nesl_utility/Solver Configuration', [modelName '/Solver_Config'], ...
    'Position', [400, 490, 450, 520]);
add_block('fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference', ...
    [modelName '/Rotational_Ground'], 'Position', [520, 520, 550, 550]);

% 2. DC Power Supply for Motor & Drive (+ and - Electrical ports)
add_block('fl_lib/Electrical/Electrical Sources/DC Voltage Source', ...
    [modelName '/Motor_DC_Supply_48V'], 'v0', '48', 'Position', [400, 320, 440, 360]);
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', ...
    [modelName '/Electrical_Ground_Motor'], 'Position', [400, 410, 430, 440]);

% 3. Motor & Drive (System Level) from Simscape Electrical
add_block('ee_lib/Electromechanical/Motor & Drive (System Level)', ...
    [modelName '/Motor_and_Drive'], 'Position', [490, 300, 590, 420]);

% Motor Torque Command (Constant 3.5 N*m -> Simulink-PS Converter -> Motor Tr port)
add_block('simulink/Sources/Constant', [modelName '/Motor_Drive_Torque_Cmd'], ...
    'Value', '3.5', 'Position', [300, 250, 350, 280]);
add_block('nesl_utility/Simulink-PS Converter', [modelName '/Sim_to_PS_MotorCmd'], ...
    'Position', [390, 250, 430, 280]);
add_line(modelName, 'Motor_Drive_Torque_Cmd/1', 'Sim_to_PS_MotorCmd/1');

% Wire DC Supply (+ to Motor LConn1, - to Motor RConn1 & Electrical Ground)
add_line(modelName, 'Motor_DC_Supply_48V/LConn1', 'Motor_and_Drive/LConn1');
add_line(modelName, 'Motor_DC_Supply_48V/RConn1', 'Motor_and_Drive/RConn1');
add_line(modelName, 'Motor_DC_Supply_48V/RConn1', 'Electrical_Ground_Motor/LConn1');

% Wire Motor Torque Command (Sim_to_PS_MotorCmd RConn1 -> Motor LConn2 [Tr])
add_line(modelName, 'Sim_to_PS_MotorCmd/RConn1', 'Motor_and_Drive/LConn2');

% Wire Motor Casing (RConn3 [C]) to Rotational Ground & Solver Config
add_line(modelName, 'Motor_and_Drive/RConn3', 'Rotational_Ground/LConn1');
add_line(modelName, 'Solver_Config/RConn1',   'Rotational_Ground/LConn1');

% 4. Flexible Shaft Coupling (Rotational Spring + Damper decoupling Motor Rotor & Load Inertias)
add_block('fl_lib/Mechanical/Rotational Elements/Rotational Spring', ...
    [modelName '/Shaft_Stiffness'], 'spr_rate', '1500', 'Position', [630, 285, 680, 315]);
add_block('fl_lib/Mechanical/Rotational Elements/Rotational Damper', ...
    [modelName '/Shaft_Damping'], 'D', '1.2', 'Position', [630, 330, 680, 360]);

add_line(modelName, 'Motor_and_Drive/LConn3', 'Shaft_Stiffness/LConn1');
add_line(modelName, 'Motor_and_Drive/LConn3', 'Shaft_Damping/LConn1');
add_line(modelName, 'Shaft_Stiffness/RConn1', 'Shaft_Damping/RConn1');

% 5. Bearing (from Simscape Driveline) connected to Load Shaft Node (Shaft_Stiffness/RConn1)
add_block('sdl_lib/Couplings & Drives/Bearing', [modelName '/Simscape_Bearing'], ...
    'Position', [730, 210, 780, 250]);
add_line(modelName, 'Shaft_Stiffness/RConn1', 'Simscape_Bearing/LConn1');

% 6. Unbalanced Load (from Simscape Driveline) connected to Load Shaft Node
add_block('sdl_lib/Inertias & Loads/Unbalanced Load', [modelName '/Simscape_Unbalanced_Load'], ...
    'inertia', '0.02', 'mass', '0.15', 'r_axis', '0.02', 'r_rotation', '0.02', ...
    'init_angle', '1.0', ...
    'Position', [730, 130, 780, 170]);
add_line(modelName, 'Shaft_Stiffness/RConn1', 'Simscape_Unbalanced_Load/LConn1');

% 7. Ideal Torque Source (Injects Bearing Fault Vibrations onto the Bearing/Load Shaft)
add_block('fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source', ...
    [modelName '/Bearing_Fault_Torque_Injector'], 'Position', [610, 140, 660, 190]);
add_line(modelName, 'Bearing_Fault_Torque_Injector/LConn1', 'Shaft_Stiffness/RConn1');
add_line(modelName, 'Bearing_Fault_Torque_Injector/RConn2', 'Rotational_Ground/LConn1');

% 8. Wheel and Axle (Bridges Light-Green Rotational Shaft -> Dark-Green Translational Housing)
add_block('fl_lib/Mechanical/Mechanisms/Wheel and Axle', ...
    [modelName '/Rotational_to_Translational_Coupler'], ...
    'Position', [760, 290, 810, 330]);
add_line(modelName, 'Shaft_Stiffness/RConn1', 'Rotational_to_Translational_Coupler/LConn1');

% 8b. Compliant Motor Housing Mount (Translational Spring + Damper + Housing Mass)
%     Provides well-posed Newton F=ma state for the Accelerometer
add_block('fl_lib/Mechanical/Translational Elements/Translational Spring', ...
    [modelName '/Housing_Spring'], 'spr_rate', '5000', 'Position', [840, 275, 885, 305]);
add_block('fl_lib/Mechanical/Translational Elements/Translational Damper', ...
    [modelName '/Housing_Damper'], 'D', '25', 'Position', [840, 320, 885, 350]);
add_block('fl_lib/Mechanical/Translational Elements/Mass', ...
    [modelName '/Housing_Mass'], 'mass', '0.5', 'Position', [915, 200, 955, 240]);

add_line(modelName, 'Rotational_to_Translational_Coupler/RConn1', 'Housing_Spring/LConn1');
add_line(modelName, 'Rotational_to_Translational_Coupler/RConn1', 'Housing_Damper/LConn1');
add_line(modelName, 'Housing_Spring/RConn1', 'Housing_Damper/RConn1');
add_line(modelName, 'Housing_Spring/RConn1', 'Housing_Mass/LConn1');

% 9. Simscape Accelerometer (ee_lib/Sensors & Transducers/Accelerometer)
add_block('ee_lib/Sensors & Transducers/Accelerometer', ...
    [modelName '/Simscape_Accelerometer'], ...
    'v_min', '-50', 'v_max', '50', ...
    'Position', [925, 280, 995, 340]);
add_line(modelName, 'Housing_Spring/RConn1', 'Simscape_Accelerometer/LConn1');

% 10. Voltage Sensor + Electrical Reference to read Accelerometer Voltage Output (+ and -)
add_block('fl_lib/Electrical/Electrical Sensors/Voltage Sensor', ...
    [modelName '/Accelerometer_Voltage_Reader'], 'Position', [1025, 280, 1075, 340]);
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', ...
    [modelName '/Electrical_Ground_Accel'], 'Position', [1000, 375, 1030, 405]);

add_line(modelName, 'Simscape_Accelerometer/RConn1', 'Accelerometer_Voltage_Reader/LConn1');
add_line(modelName, 'Simscape_Accelerometer/RConn2', 'Accelerometer_Voltage_Reader/RConn2');
add_line(modelName, 'Simscape_Accelerometer/RConn2', 'Electrical_Ground_Accel/LConn1');

% 11. PS-Simulink Converters for Accelerometer & Shaft Speed
add_block('nesl_utility/PS-Simulink Converter', [modelName '/PS_to_Sim_Accelerometer'], ...
    'Position', [1110, 285, 1150, 315]);
add_line(modelName, 'Accelerometer_Voltage_Reader/RConn1', 'PS_to_Sim_Accelerometer/LConn1');

add_block('nesl_utility/PS-Simulink Converter', [modelName '/PS_to_Sim_Shaft_Speed'], ...
    'Position', [635, 395, 675, 425]);
add_line(modelName, 'Motor_and_Drive/RConn2', 'PS_to_Sim_Shaft_Speed/LConn1');

% =========================================================================
% PART 2: FLAT SIMULINK FAULT EXCITATIONS & MANUAL SWITCHES (NO SUBSYSTEMS)
% =========================================================================

% Mode 1: HEALTHY (Zero defect impulse torque)
add_block('simulink/Sources/Constant', [modelName '/Fault1_Healthy_Zero'], ...
    'Value', '0', 'Position', [30, 30, 80, 60]);

% Mode 2: OUTER RACE FAULT (BPFO = 107.5 Hz Sine Shock Wave Harmonics)
add_block('simulink/Sources/Sine Wave', [modelName '/Fault2_OuterRace_BPFO_108Hz'], ...
    'Amplitude', '4.5', 'Frequency', '2*pi*107.5', ...
    'Position', [30, 85, 80, 115]);

% Mode 3: INNER RACE FAULT (BPFI = 162.5 Hz Wave * 30 Hz Load Zone Modulation)
add_block('simulink/Sources/Sine Wave', [modelName '/Fault3_InnerRace_BPFI_162Hz'], ...
    'Amplitude', '6.0', 'Frequency', '2*pi*162.5', ...
    'Position', [30, 140, 80, 170]);
add_block('simulink/Sources/Sine Wave', [modelName '/Fault3_LoadZone_Mod_30Hz'], ...
    'Amplitude', '0.6', 'Bias', '1.0', 'Frequency', '2*pi*30', ...
    'Position', [30, 190, 80, 220]);
add_block('simulink/Math Operations/Product', [modelName '/Fault3_Modulated_Product'], ...
    'Position', [115, 150, 145, 190]);
add_line(modelName, 'Fault3_InnerRace_BPFI_162Hz/1', 'Fault3_Modulated_Product/1');
add_line(modelName, 'Fault3_LoadZone_Mod_30Hz/1',     'Fault3_Modulated_Product/2');

% Mode 4: ROTOR UNBALANCE (Heavy 1X = 30 Hz & 2X = 60 Hz Sinusoidal Shaking)
add_block('simulink/Sources/Sine Wave', [modelName '/Fault4_Unbalance_1X_30Hz'], ...
    'Amplitude', '7.5', 'Frequency', '2*pi*30', 'Position', [30, 245, 80, 275]);
add_block('simulink/Sources/Sine Wave', [modelName '/Fault4_Unbalance_2X_60Hz'], ...
    'Amplitude', '3.0', 'Frequency', '2*pi*60', 'Position', [30, 295, 80, 325]);
add_block('simulink/Math Operations/Sum', [modelName '/Fault4_Unbalance_Sum'], ...
    'Inputs', '++', 'Position', [115, 260, 145, 295]);
add_line(modelName, 'Fault4_Unbalance_1X_30Hz/1', 'Fault4_Unbalance_Sum/1');
add_line(modelName, 'Fault4_Unbalance_2X_60Hz/1', 'Fault4_Unbalance_Sum/2');

% Interactive Manual Switches (Double-click on canvas to flip!)
add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_Inner_vs_Unbalance'], ...
    'Position', [185, 180, 225, 230], 'sw', '1');
add_line(modelName, 'Fault3_Modulated_Product/1', 'Switch_Inner_vs_Unbalance/1');
add_line(modelName, 'Fault4_Unbalance_Sum/1',     'Switch_Inner_vs_Unbalance/2');

add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_Outer_vs_Others'], ...
    'Position', [255, 110, 295, 160], 'sw', '1');
add_line(modelName, 'Fault2_OuterRace_BPFO_108Hz/1', 'Switch_Outer_vs_Others/1');
add_line(modelName, 'Switch_Inner_vs_Unbalance/1',    'Switch_Outer_vs_Others/2');

add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_HEALTHY_vs_FAULT'], ...
    'Position', [325, 55, 365, 105], 'sw', '1');
add_line(modelName, 'Fault1_Healthy_Zero/1',    'Switch_HEALTHY_vs_FAULT/1');
add_line(modelName, 'Switch_Outer_vs_Others/1', 'Switch_HEALTHY_vs_FAULT/2');

% Feed selected fault signal into Simscape Ideal Torque Source
add_block('nesl_utility/Simulink-PS Converter', [modelName '/Sim_to_PS_FaultTorque'], ...
    'Position', [420, 65, 460, 95]);
add_line(modelName, 'Switch_HEALTHY_vs_FAULT/1',   'Sim_to_PS_FaultTorque/1');
add_line(modelName, 'Sim_to_PS_FaultTorque/RConn1', 'Bearing_Fault_Torque_Injector/RConn1');

% =========================================================================
% PART 3: SCOPES & OUTPORTS
% =========================================================================
add_block('simulink/Sinks/Scope', [modelName '/Scope_Simscape_Accelerometer'], ...
    'Position', [1195, 275, 1245, 325]);
add_line(modelName, 'PS_to_Sim_Accelerometer/1', 'Scope_Simscape_Accelerometer/1');

add_block('simulink/Sinks/Scope', [modelName '/Scope_Shaft_Speed_rad_s'], ...
    'Position', [730, 385, 780, 435]);
add_line(modelName, 'PS_to_Sim_Shaft_Speed/1', 'Scope_Shaft_Speed_rad_s/1');

add_block('simulink/Sinks/Scope', [modelName '/Scope_Injected_Fault_Torque'], ...
    'Position', [420, 15, 460, 50]);
add_line(modelName, 'Switch_HEALTHY_vs_FAULT/1', 'Scope_Injected_Fault_Torque/1');

% Logged Outports for verification
add_block('simulink/Sinks/Out1', [modelName '/out_simscape_accel'], ...
    'Position', [1200, 345, 1230, 365]);
add_line(modelName, 'PS_to_Sim_Accelerometer/1', 'out_simscape_accel/1');

add_block('simulink/Sinks/Out1', [modelName '/out_shaft_speed'], ...
    'Position', [735, 455, 765, 475]);
add_line(modelName, 'PS_to_Sim_Shaft_Speed/1', 'out_shaft_speed/1');

% Drag-and-Drop Edge Impulse Data Logger Block (Plugin 1)
if bdIsLoaded('EdgeImpulse_Library')
    add_block('EdgeImpulse_Library/EI_Data_Logger', [modelName '/EI_Data_Logger'], ...
        'Position', [1195, 390, 1365, 460], ...
        'ei_label', 'simscape_bearing_accel', ...
        'ei_sensors', 'accel_V', ...
        'ei_units', 'V');
    add_line(modelName, 'PS_to_Sim_Accelerometer/1', 'EI_Data_Logger/1');
end

% Save model
slxPath = fullfile(baseDir, [modelName '.slx']);
save_system(modelName, slxPath);
close_system(modelName);

fprintf('SUCCESS: Saved pure library Simscape model to %s\n', slxPath);
