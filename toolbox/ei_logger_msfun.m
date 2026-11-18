function ei_logger_msfun(block)
    % ei_logger_msfun
    % Universal, Solver-Adaptive Level-2 MATLAB S-Function for 'EI_Data_Logger'.
    %
    % Adapts automatically to ANY Simulink solver configuration:
    %   - Variable-step continuous (ode45, ode15s, ode23t for Simscape, Auto)
    %   - Variable-step discrete
    %   - Fixed-step continuous/discrete (ode3, ode4, FixedStepDiscrete)
    %     including mismatched FixedStep sizes (auto-falls back to inherited
    %     sample time and resamples cleanly to the target Sample Rate).

    setup(block);
end

function setup(block)
    block.NumDialogPrms  = 0;
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 0;

    block.SetPreCompInpPortInfoToDynamic;
    block.InputPort(1).DirectFeedthrough = true;

    % Automatically inherit sample rate from the connected input signal / solver
    block.SampleTimes = [-1, 0];

    block.InputPort(1).Complexity = 'Real';
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
    block.RegBlockMethod('SetInputPortDataType',   @SetInpPortDType);
    block.RegBlockMethod('Start',                  @Start);
    block.RegBlockMethod('Outputs',                @Outputs);
    block.RegBlockMethod('Terminate',              @Terminate);
end

function SetInpPortDims(block, idx, di)
    block.InputPort(idx).Dimensions = di;
end

function SetInpPortDType(block, idx, dt)
    block.InputPort(idx).DatatypeID = dt;
end

function logMap = getLogMap()
    global g_ei_simulink_logs;
    if isempty(g_ei_simulink_logs) || ~isa(g_ei_simulink_logs, 'containers.Map')
        g_ei_simulink_logs = containers.Map('KeyType', 'double', 'ValueType', 'any');
    end
    logMap = g_ei_simulink_logs;
end

function val = parseNumericParam(blkH, paramName, defaultVal, minVal)
    val = defaultVal;
    try
        rawStr = strtrim(char(get_param(blkH, paramName)));
        if isempty(rawStr)
            return;
        end
        parsed = str2double(rawStr);
        if isnan(parsed)
            % Strip common unit suffixes (e.g., '5s', '5 sec', '100 Hz')
            cleanStr = regexprep(rawStr, '\s*(sec(onds?)?|s|hz)\s*$', '', 'ignorecase');
            parsed = str2double(cleanStr);
        end
        if isnan(parsed)
            % Allow model/mask workspace variables via slResolve, then base workspace
            try
                evalVal = slResolve(rawStr, blkH);
                if isnumeric(evalVal) && isscalar(evalVal) && isreal(evalVal)
                    parsed = double(evalVal);
                end
            catch
                try
                    evalVal = evalin('base', rawStr);
                    if isnumeric(evalVal) && isscalar(evalVal) && isreal(evalVal)
                        parsed = double(evalVal);
                    end
                catch
                end
            end
        end
        if ~isnan(parsed) && isfinite(parsed) && parsed >= minVal
            val = parsed;
        else
            fprintf('[EI_Data_Logger Warning] Invalid %s=''%s''. Using default %.4g.\n', ...
                paramName, rawStr, defaultVal);
        end
    catch ME
        fprintf('[EI_Data_Logger Warning] Could not read %s (%s). Using default %.4g.\n', ...
            paramName, ME.message, defaultVal);
    end
end

function Start(block)
    logMap = getLogMap();
    blkH = block.BlockHandle;

    isEnabled = true;
    try
        enStr = strtrim(char(get_param(blkH, 'ei_enabled')));
        if strcmpi(enStr, 'off') || strcmpi(enStr, '0') || strcmpi(enStr, 'false')
            isEnabled = false;
        end
    catch
    end

    winSec    = parseNumericParam(blkH, 'ei_window_sec', 10.0, 1e-3);
    maxChunks = floor(parseNumericParam(blkH, 'ei_max_chunks', 10.0, 0.0));
    fs        = 0.0; % Always inherited from the connected input signal / solver

    labelStr = 'simulink_signal';
    catStr   = 'training';
    blkPath  = '';
    try
        blkPath = getfullname(blkH);
        lRaw = strtrim(char(get_param(blkH, 'ei_label')));
        lRaw = strrep(strrep(lRaw, '''', ''), '"', '');
        if ~isempty(lRaw), labelStr = lRaw; end
        catStr = lower(strtrim(char(get_param(blkH, 'ei_category'))));
    catch
    end

    if ~isEnabled
        fprintf('[EI_Data_Logger] PAUSED (ei_enabled=off): Block=''%s'', Label=''%s''\n', ...
            blkPath, labelStr);
        logMap(blkH) = struct('enabled', false);
        return;
    end

    if maxChunks == 0
        maxStr = 'unlimited';
    else
        maxStr = sprintf('%d', maxChunks);
    end
    if fs == 0
        fsStr = 'inherit';
    else
        fsStr = sprintf('%.4g Hz', fs);
    end
    fprintf('[EI_Data_Logger] Started: Block=''%s'', Label=''%s'', Window=%.4gs, Fs=%s, MaxWindows=%s, Split=''%s''\n', ...
        blkPath, labelStr, winSec, fsStr, maxStr, catStr);

    logMap(blkH) = struct( ...
        'enabled', true, ...
        't', [], ...
        'u', [], ...
        'chunk_start_t', 0.0, ...
        'window_sec', winSec, ...
        'max_chunks', maxChunks, ...
        'fs', fs, ...
        'chunks_uploaded', 0 ...
    );
end

function Outputs(block)
    % Ignore continuous solver minor integration sub-steps (ode45, ode23t, etc.)
    if ~block.IsMajorTimeStep
        return;
    end

    logMap = getLogMap();
    blkH = block.BlockHandle;
    if ~isKey(logMap, blkH)
        Start(block);
    end

    st = logMap(blkH);

    if ~isfield(st, 'enabled') || ~st.enabled
        return;
    end

    if st.max_chunks > 0 && st.chunks_uploaded >= st.max_chunks
        return;
    end

    t_now = block.CurrentTime;
    u_now = double(block.InputPort(1).Data(:)');

    % Reset buffer if simulation time moved backwards (e.g. solver reset)
    if ~isempty(st.t) && t_now < (st.t(end) - 1e-12)
        st.t = [];
        st.u = [];
        st.chunk_start_t = t_now;
        st.chunks_uploaded = 0;
    end

    % Flush completed window BEFORE appending the first sample of the next window
    if ~isempty(st.t) && (t_now - st.chunk_start_t) >= (st.window_sec - 1e-9)
        st.chunks_uploaded = st.chunks_uploaded + 1;
        fprintf('[EI_Data_Logger] Window #%d complete (%.2fs to %.2fs, %d samples). Uploading...\n', ...
            st.chunks_uploaded, st.chunk_start_t, t_now, length(st.t));

        flushChunkToEdgeImpulse(blkH, st.t, st.u, st.fs);

        st.t = [];
        st.u = [];
        st.chunk_start_t = t_now;

        if st.max_chunks > 0 && st.chunks_uploaded >= st.max_chunks
            logMap(blkH) = st;
            return;
        end
    end

    if isempty(st.t)
        st.chunk_start_t = t_now;
    end

    if isempty(st.t) || (t_now > st.t(end) + 1e-12)
        st.t(end+1, 1) = t_now;
        st.u(end+1, :) = u_now;
    end

    logMap(blkH) = st;
end

function Terminate(block)
    logMap = getLogMap();
    blkH = block.BlockHandle;
    if ~isKey(logMap, blkH)
        return;
    end

    st = logMap(blkH);

    if ~isfield(st, 'enabled') || ~st.enabled
        remove(logMap, blkH);
        return;
    end

    effFs = st.fs;
    if effFs <= 0 && length(st.t) >= 2
        effFs = 1.0 / max(median(diff(st.t)), 1e-4);
    end
    if effFs <= 0, effFs = 100; end

    % Flush final window if simulation ended right on a window boundary
    if ~isempty(st.t) && (st.t(end) - st.chunk_start_t + (1.0 / effFs)) >= (st.window_sec - 1e-6)
        if st.max_chunks == 0 || st.chunks_uploaded < st.max_chunks
            st.chunks_uploaded = st.chunks_uploaded + 1;
            fprintf('[EI_Data_Logger] Window #%d complete (%.2fs to %.2fs, %d samples). Uploading...\n', ...
                st.chunks_uploaded, st.chunk_start_t, st.t(end), length(st.t));
            flushChunkToEdgeImpulse(blkH, st.t, st.u, st.fs);
            st.t = [];
        end
    end

    if st.chunks_uploaded == 0 && length(st.t) >= 2
        elapsed = st.t(end) - st.t(1);
        fprintf('[EI_Data_Logger] Simulation ended at %.2fs (< %.4gs window). Saving partial window...\n', ...
            elapsed, st.window_sec);
        flushChunkToEdgeImpulse(blkH, st.t, st.u, st.fs);
    elseif st.chunks_uploaded > 0
        fprintf('[EI_Data_Logger] Finished! Total full %.4gs windows uploaded: %d.\n', ...
            st.window_sec, st.chunks_uploaded);
    end

    remove(logMap, blkH);
end

function flushChunkToEdgeImpulse(blkH, t_vec, u_mat, fs_cfg)
    if fs_cfg > 0
        fs = fs_cfg;
    elseif length(t_vec) >= 2
        dt_median = median(diff(t_vec));
        if dt_median <= 0, dt_median = 0.01; end
        fs = 1.0 / dt_median;
        if abs(fs - round(fs)) < 1e-6 && round(fs) >= 1
            fs = round(fs);
        end
    else
        fs = 100;
    end

    % Resample to uniform grid if solver used variable steps or a different FixedStep
    if length(t_vec) >= 2 && max(abs(diff(t_vec) - (1.0/fs))) > (0.05 / fs)
        if any(diff(t_vec) <= 0)
            [t_vec, uidx] = unique(t_vec, 'stable');
            u_mat = u_mat(uidx, :);
        end
        t_uni = (t_vec(1) : (1.0/fs) : t_vec(end))';
        if length(t_uni) >= 2 && length(t_vec) >= 2
            u_mat = interp1(t_vec, u_mat, t_uni, 'previous', 'extrap');
        end
    end

    try
        labelStr    = strtrim(get_param(blkH, 'ei_label'));
        categoryStr = strtrim(get_param(blkH, 'ei_category'));
        sensorsRaw  = strtrim(get_param(blkH, 'ei_sensors'));
        unitsRaw    = strtrim(get_param(blkH, 'ei_units'));
        apiKeyStr   = strtrim(get_param(blkH, 'ei_apikey'));
        outDirStr   = strtrim(get_param(blkH, 'ei_outdir'));
    catch
        labelStr    = 'simulink_signal';
        categoryStr = 'training';
        sensorsRaw  = 'auto';
        unitsRaw    = 'auto';
        apiKeyStr   = '';
        outDirStr   = 'default';
    end

    labelStr  = strrep(strrep(labelStr, '''', ''), '"', '');
    apiKeyStr = strrep(strrep(apiKeyStr, '''', ''), '"', '');
    outDirStr = strrep(strrep(outDirStr, '''', ''), '"', '');
    if isempty(labelStr), labelStr = 'simulink_signal'; end

    % Guard against uploading empty/unconnected/grounded all-zero signals
    skipZero = true;
    try
        szStr = strtrim(char(get_param(blkH, 'ei_skip_zero')));
        if strcmpi(szStr, 'off') || strcmpi(szStr, '0') || strcmpi(szStr, 'false')
            skipZero = false;
        end
    catch
    end

    if skipZero && all(u_mat(:) == 0)
        blkName = '';
        try blkName = getfullname(blkH); catch, end
        fprintf('[EI_Data_Logger Warning] Skipped upload for ''%s'' (Label=''%s''): all %d samples are 0.00000 (check if wire is unconnected, grounded, or switched off).\n', ...
            blkName, labelStr, size(u_mat, 1));
        return;
    end

    if ~isempty(sensorsRaw) && ~strcmpi(sensorsRaw, 'auto')
        sensorNames = strtrim(strsplit(sensorsRaw, ','));
    else
        sensorNames = {};
    end

    if ~isempty(unitsRaw) && ~strcmpi(unitsRaw, 'auto')
        sensorUnits = strtrim(strsplit(unitsRaw, ','));
    else
        sensorUnits = {};
    end

    tbxDir = fileparts(mfilename('fullpath'));
    addpath(tbxDir);

    if isempty(outDirStr) || strcmpi(outDirStr, 'default')
        outDirStr = fullfile(tbxDir, 'universal_dataset');
    end

    uploader = EdgeImpulseUploader(apiKeyStr, outDirStr);
    uploader.sendTimeSeries(u_mat, labelStr, ...
        'Fs', fs, ...
        'Category', categoryStr, ...
        'SensorNames', sensorNames, ...
        'Units', sensorUnits);
end
