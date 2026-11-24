% create_ev_battery_simulink.m
% Builds a clean, flat (NO SUBSYSTEMS) physical Simscape EV Battery & Thermal
% Fault Simulation model: 'ev_battery_sim.slx'
%
% Physical Domains:
%   - Electrical (Blue): Battery (Table-Based), Voltage Sensor, Current Sensor,
%                        Controlled Current Source, Electrical Reference
%   - Thermal (Orange) : Battery Thermal Port (H), Thermal Mass, Convective Cooling,
%                        Controlled Heat Flow Source, Temperature Sensor, Ambient 25C
%   - Simulink (Black) : 3 Manual Switch toggles for 4 EV modes, Mux(3),
%                        Scope_BMS_3Sensors, and EI_Data_Logger (Plugin 1)

fprintf('=== Creating Physical Simscape EV Battery BMS Model (Flat Canvas) ===\n');

baseDir = fileparts(mfilename('fullpath'));
if isempty(baseDir)
    baseDir = pwd;
end
addpath(baseDir);
tbxDir = fullfile(fileparts(baseDir), 'toolbox');
addpath(tbxDir);

load_system('simulink');
load_system('ee_lib');
load_system('fl_lib');
load_system('nesl_utility');
if exist(fullfile(tbxDir, 'EdgeImpulse_Library.slx'), 'file') == 4
    load_system('EdgeImpulse_Library');
end

modelName = 'ev_battery_sim';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);

% Simscape stiff solver, 20 Hz max step (0.05s) for clean BMS telemetry
set_param(modelName, 'SolverType', 'Variable-step');
set_param(modelName, 'Solver', 'ode23t');
set_param(modelName, 'MaxStep', '0.05');
set_param(modelName, 'StopTime', '60.0');

% =========================================================================
% 1. TOP-LEFT: 4 EV BATTERY MODES & INTERACTIVE MANUAL SWITCHES
%    Each mode outputs [Load_Current_A, Runaway_Enable (0 or 1)]
%    Literature-grounded 27 Ah EV NMC cell (SoC = 0.5, R0 ~ 8.5 mOhm):
%      - Mode 1 Normal_Drive   : +20 A (~0.74C), pure I^2*R0 ohmic heat
%      - Mode 2 Fast_Charging  : -50 A (~1.85C DC fast charge, V ~ 4.15V), pure I^2*R0 heat
%      - Mode 3 Short_Circuit  : +220 A (~8.1C external short, V < 2.0V), pure I^2*R0 Joule heat
%      - Mode 4 Thermal_Runaway: +45 A internal-fault load + Arrhenius exothermic cascade
% =========================================================================
add_block('simulink/Sources/Constant', [modelName '/Mode_1_Normal_Drive'], ...
    'Value', '[20, 0]', 'Position', [30, 40, 125, 70]);

add_block('simulink/Sources/Constant', [modelName '/Mode_2_Fast_Charging'], ...
    'Value', '[-50, 0]', 'Position', [30, 105, 125, 135]);

add_block('simulink/Sources/Constant', [modelName '/Mode_3_Short_Circuit'], ...
    'Value', '[220, 0]', 'Position', [30, 170, 125, 200]);

add_block('simulink/Sources/Constant', [modelName '/Mode_4_Thermal_Runaway'], ...
    'Value', '[45, 1]', 'Position', [30, 235, 125, 265]);

% Cascaded Manual Switches (double-click any switch on the canvas to flip!)
add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_Short_vs_Runaway'], ...
    'Position', [185, 190, 225, 240], 'sw', '1');
add_line(modelName, 'Mode_3_Short_Circuit/1',   'Switch_Short_vs_Runaway/1');
add_line(modelName, 'Mode_4_Thermal_Runaway/1', 'Switch_Short_vs_Runaway/2');

add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_Charge_vs_Faults'], ...
    'Position', [265, 120, 305, 170], 'sw', '1');
add_line(modelName, 'Mode_2_Fast_Charging/1',    'Switch_Charge_vs_Faults/1');
add_line(modelName, 'Switch_Short_vs_Runaway/1', 'Switch_Charge_vs_Faults/2');

add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_Drive_vs_Others'], ...
    'Position', [345, 65, 385, 115], 'sw', '1');
add_line(modelName, 'Mode_1_Normal_Drive/1',     'Switch_Drive_vs_Others/1');
add_line(modelName, 'Switch_Charge_vs_Faults/1', 'Switch_Drive_vs_Others/2');

% Add subtle realistic driving current ripple & thermal jitter
add_block('simulink/Sources/Sine Wave', [modelName '/Drive_Current_Ripple'], ...
    'Amplitude', '[2.5, 0]', 'Frequency', '2*pi*0.5', 'SampleTime', '0.05', ...
    'Position', [345, 145, 385, 175]);
add_block('simulink/Math Operations/Sum', [modelName '/Add_Ripple'], ...
    'Inputs', '++', 'Position', [420, 80, 445, 115]);
add_line(modelName, 'Switch_Drive_vs_Others/1', 'Add_Ripple/1');
add_line(modelName, 'Drive_Current_Ripple/1',   'Add_Ripple/2');

% Split [Load_Current_A, Exothermic_Heat_W]
add_block('simulink/Signal Routing/Demux', [modelName '/Split_Cmd'], ...
    'Outputs', '2', 'Position', [475, 70, 480, 125]);
add_line(modelName, 'Add_Ripple/1', 'Split_Cmd/1');

% =========================================================================
% 2. MIDDLE: REAL PHYSICAL SIMSCAPE BATTERY + ELECTRICAL CIRCUIT (BLUE)
% =========================================================================
add_block('nesl_utility/Solver Configuration', [modelName '/Solver_Config'], ...
    'Position', [520, 455, 570, 485]);

add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', ...
    [modelName '/Electrical_Ground'], 'Position', [620, 455, 650, 485]);

% Simscape Battery (Table-Based) starting at 50% SoC (27 Ah NMC cell, R0 ~ 8.5 mOhm)
% with Thermal Port enabled (internal thermal mass = 100 J/K)
batBlk = add_block('ee_lib/Sources/Battery (Table-Based)', ...
    [modelName '/Simscape_LiIon_Battery'], ...
    'stateOfCharge_specify', 'on', 'stateOfCharge', '0.5', ...
    'Position', [600, 220, 670, 310]);
try
    set_param(batBlk, 'thermal_port', '1');
catch
end

% Current Sensor (Series on + terminal) & Voltage Sensor (Parallel across + and -)
add_block('fl_lib/Electrical/Electrical Sensors/Current Sensor', ...
    [modelName '/Battery_Current_Sensor'], 'Position', [740, 200, 790, 240]);

add_block('fl_lib/Electrical/Electrical Sensors/Voltage Sensor', ...
    [modelName '/Battery_Voltage_Sensor'], 'Position', [695, 330, 745, 370]);

% Controlled Current Source (EV Motor / Charger / Short-Circuit Load)
add_block('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ...
    [modelName '/EV_Load_Current_Source'], 'Position', [845, 290, 885, 340]);

% Simulink-PS Converter for Load Current Command
add_block('nesl_utility/Simulink-PS Converter', [modelName '/Sim_to_PS_Current'], ...
    'Position', [530, 65, 570, 95]);
add_line(modelName, 'Split_Cmd/1', 'Sim_to_PS_Current/1');
add_line(modelName, 'Sim_to_PS_Current/RConn1', 'EV_Load_Current_Source/RConn1');

% Wire Electrical Loop:
%   Battery(+) [LConn1] -> Current Sensor(+) [LConn1]
%   Current Sensor(-) [RConn2] -> Load(+) [LConn1]
%   Load(-) [RConn2] -> Battery(-) [RConn1] -> Electrical Ground & Solver Config
add_line(modelName, 'Simscape_LiIon_Battery/LConn1', 'Battery_Current_Sensor/LConn1');
add_line(modelName, 'Battery_Current_Sensor/RConn2', 'EV_Load_Current_Source/RConn2');
add_line(modelName, 'EV_Load_Current_Source/LConn1', 'Simscape_LiIon_Battery/RConn1');
add_line(modelName, 'Simscape_LiIon_Battery/RConn1', 'Electrical_Ground/LConn1');
add_line(modelName, 'Solver_Config/RConn1',          'Electrical_Ground/LConn1');

% Wire Voltage Sensor across Battery(+) and Battery(-)
add_line(modelName, 'Simscape_LiIon_Battery/LConn1', 'Battery_Voltage_Sensor/LConn1');
add_line(modelName, 'Simscape_LiIon_Battery/RConn1', 'Battery_Voltage_Sensor/RConn2');

% =========================================================================
% 3. BOTTOM-MIDDLE: REAL PHYSICAL SIMSCAPE THERMAL CIRCUIT (ORANGE)
%    - Total EV 27 Ah cell thermal mass: 100 J/K (internal) + 400 J/K (external) = 500 J/K
%    - Active EV liquid cooling plate: h = 200 W/(m^2*K), A = 0.02 m^2 => hA = 4.0 W/K
%    - Temperature-dependent Arrhenius exothermic heat (only active in Mode 4)
% =========================================================================
add_block('fl_lib/Thermal/Thermal Elements/Thermal Reference', ...
    [modelName '/Thermal_Ground'], 'Position', [620, 650, 650, 680]);

add_block('fl_lib/Thermal/Thermal Sources/Temperature Source', ...
    [modelName '/Coolant_Temp_25C'], 'temperature', '298.15', ...
    'Position', [520, 585, 570, 620]);

add_block('fl_lib/Thermal/Thermal Elements/Convective Heat Transfer', ...
    [modelName '/Liquid_Cooling_Plate'], 'area', '0.02', 'heat_tr_coeff', '200', ...
    'Position', [610, 580, 665, 620]);

add_block('fl_lib/Thermal/Thermal Elements/Thermal Mass', ...
    [modelName '/Cell_Thermal_Mass'], 'mass', '0.40', 'sp_heat', '1000', ...
    'T_specify', 'on', 'T', '298.15', ...
    'Position', [715, 515, 765, 555]);

add_block('fl_lib/Thermal/Thermal Sources/Controlled Heat Flow Rate Source', ...
    [modelName '/Exothermic_Fault_Heat'], 'Position', [520, 515, 570, 555]);

add_block('fl_lib/Thermal/Thermal Sensors/Temperature Sensor', ...
    [modelName '/Cell_Temp_Sensor'], 'Position', [795, 575, 845, 615]);

% Arrhenius Exothermic Law: Q_rxn = Enable * (1200 + 400*exp(-7060*(1/T_K - 1/353.15)))
% Active ONLY when Runaway_Enable == 1; strictly 0 W for Normal/FastCharge/ShortCircuit!
% Saturated cleanly at [0, 4000 W] max exothermic runaway power.
add_block('simulink/Signal Routing/Mux', [modelName '/Arrhenius_Inputs'], ...
    'Inputs', '2', 'Position', [205, 510, 212, 555]);
add_block('simulink/User-Defined Functions/Fcn', [modelName '/Arrhenius_Exothermic_Law'], ...
    'Expr', 'u(1) * (1200 + 400 * exp(-7060 * (1/u(2) - 1/353.15)))', ...
    'Position', [245, 515, 355, 550]);
add_block('simulink/Discontinuities/Saturation', [modelName '/Max_Runaway_Power_4kW'], ...
    'LowerLimit', '0', 'UpperLimit', '4000', ...
    'Position', [385, 518, 415, 548]);

% Simulink-PS Converter for Exothermic Heat Command
add_block('nesl_utility/Simulink-PS Converter', [modelName '/Sim_to_PS_Heat'], ...
    'Position', [445, 520, 485, 550]);
add_line(modelName, 'Split_Cmd/2',                'Arrhenius_Inputs/1');
add_line(modelName, 'Arrhenius_Inputs/1',         'Arrhenius_Exothermic_Law/1');
add_line(modelName, 'Arrhenius_Exothermic_Law/1', 'Max_Runaway_Power_4kW/1');
add_line(modelName, 'Max_Runaway_Power_4kW/1',    'Sim_to_PS_Heat/1');
add_line(modelName, 'Sim_to_PS_Heat/RConn1',      'Exothermic_Fault_Heat/RConn1');

% Wire Thermal Loop:
%   Thermal Node = Cell_Thermal_Mass/LConn1
%   Connect Battery Thermal Port (LConn2 if present) + Heat Source + Liquid Cooling + Temp Sensor
batPH = get_param(batBlk, 'PortHandles');
if numel(batPH.LConn) >= 2
    add_line(modelName, 'Simscape_LiIon_Battery/LConn2', 'Cell_Thermal_Mass/LConn1');
elseif numel(batPH.RConn) >= 2
    add_line(modelName, 'Simscape_LiIon_Battery/RConn2', 'Cell_Thermal_Mass/LConn1');
end

add_line(modelName, 'Exothermic_Fault_Heat/RConn2', 'Thermal_Ground/LConn1');
add_line(modelName, 'Exothermic_Fault_Heat/LConn1', 'Cell_Thermal_Mass/LConn1');

add_line(modelName, 'Coolant_Temp_25C/LConn1',      'Liquid_Cooling_Plate/LConn1');
add_line(modelName, 'Liquid_Cooling_Plate/RConn1',  'Cell_Thermal_Mass/LConn1');

add_line(modelName, 'Cell_Temp_Sensor/LConn1', 'Cell_Thermal_Mass/LConn1');
add_line(modelName, 'Cell_Temp_Sensor/RConn1', 'Thermal_Ground/LConn1');

% =========================================================================
% 4. RIGHT SIDE: PS-SIMULINK CONVERTERS -> MUX(3) -> SCOPE & EI_DATA_LOGGER
%    Mux_3Sensors strictly takes ONLY 3 inputs: [Voltage_V, Current_A, Temp_C]
% =========================================================================
add_block('nesl_utility/PS-Simulink Converter', [modelName '/PS_to_Sim_Voltage_V'], ...
    'Position', [895, 330, 935, 360]);
add_line(modelName, 'Battery_Voltage_Sensor/RConn1', 'PS_to_Sim_Voltage_V/LConn1');

add_block('nesl_utility/PS-Simulink Converter', [modelName '/PS_to_Sim_Current_A'], ...
    'Position', [895, 200, 935, 230]);
add_line(modelName, 'Battery_Current_Sensor/RConn1', 'PS_to_Sim_Current_A/LConn1');

add_block('nesl_utility/PS-Simulink Converter', [modelName '/PS_to_Sim_Temp_K'], ...
    'Position', [895, 575, 935, 605]);
add_line(modelName, 'Cell_Temp_Sensor/RConn2', 'PS_to_Sim_Temp_K/LConn1');

% Feed real-time Cell Temperature (K) back into Arrhenius Exothermic Law
% (Memory block with X0 = 298.15 K cleanly breaks the Simscape-Simulink algebraic loop)
add_block('simulink/Discrete/Memory', [modelName '/Temp_Feedback_Delay'], ...
    'X0', '298.15', 'Position', [520, 650, 550, 680]);
add_line(modelName, 'PS_to_Sim_Temp_K/1',    'Temp_Feedback_Delay/1');
add_line(modelName, 'Temp_Feedback_Delay/1', 'Arrhenius_Inputs/2');

% Convert Kelvin -> Celsius (Temp_C = Temp_K - 273.15)
add_block('simulink/Sources/Constant', [modelName '/Kelvin_Offset'], ...
    'Value', '273.15', 'Position', [895, 615, 945, 640]);
add_block('simulink/Math Operations/Sum', [modelName '/Kelvin_to_Celsius'], ...
    'Inputs', '+-', 'Position', [975, 560, 1000, 600]);
add_line(modelName, 'PS_to_Sim_Temp_K/1', 'Kelvin_to_Celsius/1');
add_line(modelName, 'Kelvin_Offset/1',    'Kelvin_to_Celsius/2');

% Fuse all 3 sensors into [Voltage_V, Current_A, Temp_C]
add_block('simulink/Signal Routing/Mux', [modelName '/Mux_3Sensors'], ...
    'Inputs', '3', 'Position', [1055, 280, 1065, 410]);
add_line(modelName, 'PS_to_Sim_Voltage_V/1', 'Mux_3Sensors/1');
add_line(modelName, 'PS_to_Sim_Current_A/1', 'Mux_3Sensors/2');
add_line(modelName, 'Kelvin_to_Celsius/1',   'Mux_3Sensors/3');

% Scope + Outport
add_block('simulink/Sinks/Scope', [modelName '/Scope_BMS_3Sensors'], ...
    'Position', [1145, 275, 1195, 325]);
add_line(modelName, 'Mux_3Sensors/1', 'Scope_BMS_3Sensors/1');

add_block('simulink/Sinks/Out1', [modelName '/out_bms_3sensors'], ...
    'Position', [1150, 465, 1180, 485]);
add_line(modelName, 'Mux_3Sensors/1', 'out_bms_3sensors/1');

% Drag-and-Drop Edge Impulse Data Logger Block (Plugin 1)
if bdIsLoaded('EdgeImpulse_Library')
    add_block('EdgeImpulse_Library/EI_Data_Logger', [modelName '/EI_Data_Logger'], ...
        'Position', [1145, 360, 1320, 435], ...
        'ei_label', 'Normal_Drive', ...
        'ei_window_sec', '3', ...
        'ei_max_chunks', '0', ...
        'ei_sensors', 'Voltage_V,Current_A,Temp_C', ...
        'ei_units', 'V,A,Cel');
    add_line(modelName, 'Mux_3Sensors/1', 'EI_Data_Logger/1');
end

% Save model
slxPath = fullfile(baseDir, [modelName '.slx']);
save_system(modelName, slxPath);
close_system(modelName);

fprintf('SUCCESS: Saved physical Simscape EV Battery model to %s\n', slxPath);
