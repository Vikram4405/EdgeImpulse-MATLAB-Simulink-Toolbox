% verify_ev_battery_simulink.m
% Builds and verifies the physical Simscape EV Battery BMS model (ev_battery_sim.slx)
% across all 4 operating & fault states, printing real Voltage, Current & Temperature
% readings and generating a 4-state verification plot.

fprintf('===========================================================================\n');
fprintf('  Physical Simscape EV Battery BMS Simulation & Verification               \n');
fprintf('===========================================================================\n\n');

baseDir = fileparts(mfilename('fullpath'));
if isempty(baseDir)
    baseDir = pwd;
end
addpath(baseDir);
addpath(fullfile(fileparts(baseDir), 'toolbox'));

% Build the flat Simscape model
run(fullfile(baseDir, 'create_ev_battery_simulink.m'));

modelName = 'ev_battery_sim';
load_system(modelName);

% Temporarily pause EI_Data_Logger during automated verification so we don't spam disk
set_param([modelName '/EI_Data_Logger'], 'ei_enabled', 'off');

modes = {
    1, 'Normal_Drive',    '1', '1', '1', 'Mode 1: NORMAL DRIVE (+20A, 0.74C)';
    2, 'Fast_Charging',   '0', '1', '1', 'Mode 2: FAST CHARGING (-50A, 1.85C)';
    3, 'Short_Circuit',   '0', '0', '1', 'Mode 3: SHORT CIRCUIT (+220A, Pure I^2R)';
    4, 'Thermal_Runaway', '0', '0', '0', 'Mode 4: THERMAL RUNAWAY (+45A + Arrhenius)'
};

fprintf('----------------------------------------------------------------------------------------\n');
fprintf(' Mode | Battery State    | Voltage (V) Mean | Current (A) Mean | Temp (C) Start -> End \n');
fprintf('----------------------------------------------------------------------------------------\n');

res = struct();

for m = 1:size(modes, 1)
    modeId = modes{m, 1};
    name   = modes{m, 2};
    swA    = modes{m, 3};
    swB    = modes{m, 4};
    swC    = modes{m, 5};
    desc   = modes{m, 6};

    set_param([modelName '/Switch_Drive_vs_Others'],   'sw', swA);
    set_param([modelName '/Switch_Charge_vs_Faults'],  'sw', swB);
    set_param([modelName '/Switch_Short_vs_Runaway'],  'sw', swC);

    simOut = sim(modelName, 'StopTime', '60.0', 'ReturnWorkspaceOutputs', 'on');

    sig = simOut.yout{1};
    t   = sig.Values.Time;
    dat = squeeze(sig.Values.Data);
    if size(dat, 1) ~= length(t) && size(dat, 2) == length(t)
        dat = dat';
    end

    assert(size(dat, 2) == 3, 'Expected strictly 3 output channels [V, A, C], got %d', size(dat, 2));
    assert(all(isfinite(dat(:))), 'Non-finite (NaN/Inf) value detected in mode %s!', name);

    v_mean = mean(dat(:, 1));
    i_mean = mean(dat(:, 2));
    t_start = dat(1, 3);
    t_end   = dat(end, 3);

    fprintf('  %d   | %-16s | %11.3f V    | %11.2f A    | %6.1f C -> %6.1f C\n', ...
        modeId, name, v_mean, i_mean, t_start, t_end);

    res(m).t = t;
    res(m).V = dat(:, 1);
    res(m).I = dat(:, 2);
    res(m).T = dat(:, 3);
    res(m).desc = desc;
end
fprintf('----------------------------------------------------------------------------------------\n\n');

% Restore default Normal_Drive state & re-enable EI_Data_Logger
set_param([modelName '/Switch_Drive_vs_Others'],  'sw', '1');
set_param([modelName '/Switch_Charge_vs_Faults'], 'sw', '1');
set_param([modelName '/Switch_Short_vs_Runaway'], 'sw', '1');
set_param([modelName '/EI_Data_Logger'], 'ei_enabled', 'on');
save_system(modelName);
close_system(modelName);

% Plot 4-State Verification Figure
fig = figure('Name', 'Simscape EV Battery BMS 4-State Verification', ...
             'Position', [100, 80, 1250, 820], 'Visible', 'off');

for m = 1:4
    subplot(4, 3, (m-1)*3 + 1);
    plot(res(m).t, res(m).V, 'b', 'LineWidth', 1.4); grid on;
    title([res(m).desc ' - Voltage'], 'FontSize', 9, 'FontWeight', 'bold');
    ylabel('Voltage (V)'); xlabel('Time (s)');

    subplot(4, 3, (m-1)*3 + 2);
    plot(res(m).t, res(m).I, 'r', 'LineWidth', 1.4); grid on;
    title('Current (A)', 'FontSize', 9, 'FontWeight', 'bold');
    ylabel('Current (A)'); xlabel('Time (s)');

    subplot(4, 3, (m-1)*3 + 3);
    plot(res(m).t, res(m).T, 'Color', [0.9, 0.45, 0.1], 'LineWidth', 1.6); grid on;
    title('Cell Temperature (C)', 'FontSize', 9, 'FontWeight', 'bold');
    ylabel('Temp (C)'); xlabel('Time (s)');
end

outPng = fullfile(baseDir, 'ev_battery_verification.png');
saveas(fig, outPng);
delete(fig);
fprintf('Saved EV Battery verification plot to: %s\n', outPng);
