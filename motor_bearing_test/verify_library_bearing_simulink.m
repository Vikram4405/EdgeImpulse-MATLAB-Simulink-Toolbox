% verify_library_bearing_simulink.m
% Verifies the 100% pure Simscape & Simulink library model (motor_bearing_sim.slx)
% by toggling the Manual Switches across all 4 bearing states, logging the
% physical Simscape Accelerometer output, and plotting the waveforms.

fprintf('=========================================================================\n');
fprintf('  Pure Simscape Library Model Verification (Motor + Bearing + Accel)     \n');
fprintf('=========================================================================\n\n');

baseDir = fileparts(mfilename('fullpath'));
if isempty(baseDir)
    baseDir = pwd;
end
addpath(baseDir);

% Rebuild model cleanly
run(fullfile(baseDir, 'create_library_bearing_simulink.m'));

modelName = 'motor_bearing_sim';
load_system(modelName);

modes = {
    1, 'HEALTHY',    '1', '1', '1', 'Mode 1: HEALTHY (Normal Motor & Bearing)';
    2, 'OUTER_RACE', '0', '1', '1', 'Mode 2: OUTER RACE FAULT (BPFO = 107.5 Hz)';
    3, 'INNER_RACE', '0', '0', '1', 'Mode 3: INNER RACE FAULT (BPFI = 162.5 Hz Modulated)';
    4, 'UNBALANCE',  '0', '0', '0', 'Mode 4: ROTOR UNBALANCE (1X = 30 Hz + 2X = 60 Hz)'
};

fprintf('-------------------------------------------------------------------------------------\n');
fprintf(' Mode | Condition    | Accel Voltage (Mean) | AC Vibration RMS | Peak-to-Peak (V_pp) \n');
fprintf('-------------------------------------------------------------------------------------\n');

res = struct();

for m = 1:size(modes, 1)
    modeId = modes{m, 1};
    name   = modes{m, 2};
    swA    = modes{m, 3};
    swB    = modes{m, 4};
    swC    = modes{m, 5};
    desc   = modes{m, 6};

    % Toggle Manual Switches on flat canvas
    set_param([modelName '/Switch_HEALTHY_vs_FAULT'],   'sw', swA);
    set_param([modelName '/Switch_Outer_vs_Others'],    'sw', swB);
    set_param([modelName '/Switch_Inner_vs_Unbalance'], 'sw', swC);

    simOut = sim(modelName, 'StopTime', '0.6', 'ReturnWorkspaceOutputs', 'on');

    accSig = simOut.yout{1};
    spdSig = simOut.yout{2};

    t_vec = accSig.Values.Time;
    v_acc = squeeze(accSig.Values.Data);
    w_spd = squeeze(spdSig.Values.Data);

    % Evaluate steady-state after initial 0.1s transient
    mask = t_vec >= 0.1;
    v_ss = v_acc(mask);
    v_mean = mean(v_ss); % Null offset is 2.5V on Simscape Accelerometer
    v_ac   = v_ss - v_mean;
    v_rms  = sqrt(mean(v_ac.^2));
    v_p2p  = max(v_ss) - min(v_ss);

    fprintf('  %d   | %-12s | %15.3f V    | %11.4f V    | %13.4f V\n', ...
        modeId, name, v_mean, v_rms, v_p2p);

    res(m).t    = t_vec(mask);
    res(m).vacc = v_ss;
    res(m).vac  = v_ac;
    res(m).desc = desc;
    res(m).rms  = v_rms;
    res(m).p2p  = v_p2p;
end
fprintf('-------------------------------------------------------------------------------------\n\n');

% Reset switches to Healthy and save
set_param([modelName '/Switch_HEALTHY_vs_FAULT'],   'sw', '1');
set_param([modelName '/Switch_Outer_vs_Others'],    'sw', '1');
set_param([modelName '/Switch_Inner_vs_Unbalance'], 'sw', '1');
save_system(modelName);
close_system(modelName);

% Plot all 4 states from the Simscape Accelerometer
fig = figure('Name', 'Simscape Library Motor Bearing Accelerometer Readings', ...
             'Position', [100, 80, 1200, 800], 'Visible', 'off');

for m = 1:4
    subplot(4, 2, (m-1)*2 + 1);
    plot(res(m).t * 1000, res(m).vacc, 'Color', [0.1, 0.5, 0.85], 'LineWidth', 1.2);
    grid on;
    title([res(m).desc ' - Simscape Accelerometer Output (V)'], 'FontWeight', 'bold');
    xlabel('Time (ms)');
    ylabel('Sensor Output (V)');

    subplot(4, 2, (m-1)*2 + 2);
    % Interpolate to uniform 2000 Hz grid for clean FFT
    t_uni = (0.1:0.0005:0.6)';
    x_uni = interp1(res(m).t, res(m).vac, t_uni, 'linear', 'extrap');
    L = length(x_uni);
    Y = abs(fft(x_uni) / L);
    P1 = Y(1:floor(L/2)+1);
    P1(2:end-1) = 2 * P1(2:end-1);
    f_ax = 2000 * (0:floor(L/2)) / L;

    plot(f_ax, P1, 'Color', [0.85, 0.3, 0.15], 'LineWidth', 1.2);
    grid on;
    xlim([0, 250]);
    title(sprintf('AC Vibration Spectrum (RMS = %.4f V, V_{pp} = %.4f V)', res(m).rms, res(m).p2p));
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (V)');
end

outPng = fullfile(baseDir, 'simscape_library_verification.png');
saveas(fig, outPng);
delete(fig);
fprintf('Saved verification plot to: %s\n', outPng);
