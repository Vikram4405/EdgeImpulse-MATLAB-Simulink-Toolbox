% verify_and_collect_readings.m
% Compiles the Motor Bearing 3-Axis Accelerometer S-Function, builds the flat
% Simulink model (motor_bearing_sim.slx), simulates all 4 bearing states,
% verifies the accelerometer readings, plots time & FFT spectra, and exports
% Edge Impulse-ready training CSV files.

fprintf('======================================================================\n');
fprintf('  Motor Bearing Predictive Maintenance: Simulink & Accelerometer Test \n');
fprintf('======================================================================\n\n');

baseDir = fileparts(mfilename('fullpath'));
if isempty(baseDir)
    baseDir = pwd;
end
addpath(baseDir);

% 1. Compile S-Function if needed
sfunBin = fullfile(baseDir, ['motor_bearing_accel_sfun.' mexext]);
if exist(sfunBin, 'file') ~= 3
    run(fullfile(baseDir, 'build_bearing_sfun.m'));
end

% 2. Create flat Simulink model
run(fullfile(baseDir, 'create_motor_bearing_simulink.m'));

modelName = 'motor_bearing_sim';
load_system(modelName);

% Prepare dataset output directory for Edge Impulse
datasetDir = fullfile(baseDir, 'edge_impulse_dataset');
if ~exist(datasetDir, 'dir')
    mkdir(datasetDir);
end

% -------------------------------------------------------------------------
% 3. Simulate All 4 Bearing States by Flipping the Simulink Toggle Switches
% -------------------------------------------------------------------------
% Switch positions to select each mode:
%   Mode 1 (HEALTHY)    : Toggle_HEALTHY_vs_FAULT = '1'
%   Mode 2 (OUTER_RACE) : Toggle_HEALTHY_vs_FAULT = '0', Toggle_Outer_vs_Others = '1'
%   Mode 3 (INNER_RACE) : Toggle_HEALTHY_vs_FAULT = '0', Toggle_Outer_vs_Others = '0', Toggle_Inner_vs_Unbalance = '1'
%   Mode 4 (UNBALANCE)  : Toggle_HEALTHY_vs_FAULT = '0', Toggle_Outer_vs_Others = '0', Toggle_Inner_vs_Unbalance = '0'

modes = {
    1, 'healthy',    '1', '1', '1', 'HEALTHY (Normal Operation)';
    2, 'outer_race', '0', '1', '1', 'OUTER RACE FAULT (BPFO = 107.5 Hz)';
    3, 'inner_race', '0', '0', '1', 'INNER RACE FAULT (BPFI = 162.5 Hz)';
    4, 'unbalance',  '0', '0', '0', 'ROTOR UNBALANCE (1X = 30 Hz, 2X = 60 Hz)'
};

Fs = 1000; % 1000 Hz sampling rate
simDuration = 2.0; % 2.0 seconds per test run (2000 samples)

results = struct();

fprintf('------------------------------------------------------------------------------------------\n');
fprintf(' Mode | Bearing Condition        | Accel X RMS | Peak-to-Peak | Kurtosis | Max Shock (N)\n');
fprintf('------------------------------------------------------------------------------------------\n');

for m = 1:size(modes, 1)
    modeId   = modes{m, 1};
    labelStr = modes{m, 2};
    swA      = modes{m, 3};
    swB      = modes{m, 4};
    swC      = modes{m, 5};
    descStr  = modes{m, 6};

    % Flip the Manual Switches in Simulink
    set_param([modelName '/Toggle_HEALTHY_vs_FAULT'],   'sw', swA);
    set_param([modelName '/Toggle_Outer_vs_Others'],    'sw', swB);
    set_param([modelName '/Toggle_Inner_vs_Unbalance'], 'sw', swC);

    % Run Simulink model for 2.0 seconds
    simOut = sim(modelName, 'StopTime', num2str(simDuration), 'ReturnWorkspaceOutputs', 'on');

    % Extract logged Outports from yout dataset
    accelSig = simOut.yout{1};
    forceSig = simOut.yout{2};

    t_vec = accelSig.Values.Time;
    accData = squeeze(accelSig.Values.Data);
    if size(accData, 1) ~= length(t_vec) && size(accData, 2) == length(t_vec)
        accData = accData';
    end
    forceData = squeeze(forceSig.Values.Data);
    forceData = forceData(:);

    % Compute diagnostic metrics on steady-state window (after first 0.2s)
    idx = t_vec >= 0.2;
    ax = accData(idx, 1);
    ay = accData(idx, 2) - 9.81; % Remove static gravity for AC vibration analysis
    az = accData(idx, 3);

    rms_x  = sqrt(mean(ax.^2));
    p2p_x  = max(ax) - min(ax);
    kurt_x = mean((ax - mean(ax)).^4) / (std(ax)^4);
    max_F  = max(forceData(idx));

    fprintf('  %d   | %-24s | %7.2f m/s2 | %8.2f m/s2 | %8.2f | %8.1f N\n', ...
        modeId, upper(labelStr), rms_x, p2p_x, kurt_x, max_F);

    % Save into results struct for plotting
    results(m).t = t_vec;
    results(m).acc = accData;
    results(m).force = forceData;
    results(m).desc = descStr;
    results(m).label = labelStr;
    results(m).rms_x = rms_x;
    results(m).kurt_x = kurt_x;

    % Export 5 training segments (1.0s = 1000 samples each) in Edge Impulse CSV format
    for sampleIdx = 1:5
        % Vary RPM slightly (1770 to 1830 RPM) & severity (0.85 to 1.15) for realistic ML training variety
        rpm_var = 1800 + (sampleIdx - 3) * 15;
        sev_var = 1.0 + (sampleIdx - 3) * 0.08;
        set_param([modelName '/Motor_Speed_RPM'], 'Value', num2str(rpm_var));
        set_param([modelName '/Fault_Severity'],  'Value', num2str(sev_var));

        simBatch = sim(modelName, 'StopTime', '1.0', 'ReturnWorkspaceOutputs', 'on');
        bAcc = squeeze(simBatch.yout{1}.Values.Data);
        bTime = simBatch.yout{1}.Values.Time;
        if size(bAcc, 1) ~= length(bTime) && size(bAcc, 2) == length(bTime)
            bAcc = bAcc';
        end

        % Edge Impulse CSV format: timestamp (ms), accX, accY, accZ
        timestamp_ms = round(bTime * 1000);
        csvFilename = fullfile(datasetDir, sprintf('%s.sample_%02d.csv', labelStr, sampleIdx));
        fid = fopen(csvFilename, 'w');
        fprintf(fid, 'timestamp,accX,accY,accZ\n');
        for row = 1:length(timestamp_ms)
            fprintf(fid, '%d,%.4f,%.4f,%.4f\n', ...
                timestamp_ms(row), bAcc(row, 1), bAcc(row, 2), bAcc(row, 3));
        end
        fclose(fid);
    end

    % Restore nominal 1800 RPM & 1.0 severity
    set_param([modelName '/Motor_Speed_RPM'], 'Value', '1800');
    set_param([modelName '/Fault_Severity'],  'Value', '1.0');
end
fprintf('------------------------------------------------------------------------------------------\n\n');

% Reset switches to HEALTHY (Mode 1) and save model
set_param([modelName '/Toggle_HEALTHY_vs_FAULT'],   'sw', '1');
set_param([modelName '/Toggle_Outer_vs_Others'],    'sw', '1');
set_param([modelName '/Toggle_Inner_vs_Unbalance'], 'sw', '1');
save_system(modelName);
close_system(modelName);

% -------------------------------------------------------------------------
% 4. Generate Verification Plot (Time Waveforms + FFT Spectra)
% -------------------------------------------------------------------------
fig = figure('Name', 'Motor Bearing 3-Axis Accelerometer Verification', ...
             'Position', [80, 60, 1250, 850], 'Visible', 'off');

for m = 1:4
    t_vec = results(m).t;
    acc   = results(m).acc;

    % Left Column: Zoomed Time-Domain 3-Axis Accelerometer (0.2s to 0.45s = 250 ms window)
    subplot(4, 2, (m-1)*2 + 1);
    zoom_mask = (t_vec >= 0.2) & (t_vec <= 0.45);
    plot(t_vec(zoom_mask)*1000, acc(zoom_mask, 1), 'r', 'LineWidth', 1.1, 'DisplayName', 'accX (Radial H)'); hold on;
    plot(t_vec(zoom_mask)*1000, acc(zoom_mask, 2), 'g', 'LineWidth', 1.1, 'DisplayName', 'accY (Radial V + g)');
    plot(t_vec(zoom_mask)*1000, acc(zoom_mask, 3), 'b', 'LineWidth', 1.0, 'DisplayName', 'accZ (Axial)');
    grid on;
    title(sprintf('%s - 3-Axis Accelerometer', results(m).desc), 'FontWeight', 'bold');
    xlabel('Time (ms)');
    ylabel('Accel (m/s^2)');
    if m == 1
        legend('Location', 'northeast');
    end

    % Right Column: Single-Sided FFT Amplitude Spectrum of Radial Vibration (accX)
    subplot(4, 2, (m-1)*2 + 2);
    sig_ac = acc(t_vec >= 0.2, 1) - mean(acc(t_vec >= 0.2, 1));
    L = length(sig_ac);
    Y = fft(sig_ac);
    P2 = abs(Y / L);
    P1 = P2(1:floor(L/2)+1);
    P1(2:end-1) = 2 * P1(2:end-1);
    f_axis = Fs * (0:floor(L/2)) / L;

    plot(f_axis, P1, 'Color', [0.15, 0.45, 0.8], 'LineWidth', 1.2); hold on;
    xline(30,    '--k', '1X (30Hz)',    'FontSize', 8);
    xline(107.5, '--r', 'BPFO (108Hz)', 'FontSize', 8);
    xline(162.5, '--m', 'BPFI (162Hz)', 'FontSize', 8);
    xline(310,   ':k',  'Resonance',    'FontSize', 8);
    grid on;
    xlim([0, 450]);
    title(sprintf('FFT Spectrum (RMS = %.2f m/s^2, Kurtosis = %.1f)', results(m).rms_x, results(m).kurt_x));
    xlabel('Frequency (Hz)');
    ylabel('Magnitude');
end

plotPath = fullfile(baseDir, 'motor_bearing_verification.png');
saveas(fig, plotPath);
delete(fig);

fprintf('SUCCESS:\n');
fprintf('  1. Verified Simulink Model : %s\n', fullfile(baseDir, 'motor_bearing_sim.slx'));
fprintf('  2. Saved Verification Plot : %s\n', plotPath);
fprintf('  3. Exported 20 EI CSV files: %s\n', datasetDir);
