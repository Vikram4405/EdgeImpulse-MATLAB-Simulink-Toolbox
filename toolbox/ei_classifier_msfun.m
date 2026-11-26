function ei_classifier_msfun(block)
    % ei_classifier_msfun
    % Universal, Solver-Adaptive Level-2 MATLAB S-Function for 'EI_Model' (Plugin 2).
    %
    % Features:
    %   - Auto-imports & compiles any Edge Impulse C++ .ZIP specified on the block
    %   - Supports Time-Series, Audio, Images (Single-Frame), Regression, Object
    %     Detection, and Anomaly Detection (K-means / GMM)
    %   - Live Visual Prediction Monitor HUD showing the predicted TEXT LABEL,
    %     confidence/value, anomaly score, live input window, and bar chart
    %   - Drives 3 Simulink output ports: Class_ID, Confidence, and Class_Scores

    setup(block);
end

function maybeAutoImportZip(blkH)
    tbxDir = fileparts(mfilename('fullpath'));
    addpath(tbxDir);
    zipStr = strtrim(char(get_param(blkH, 'ei_zip_path')));
    zipStr = strrep(strrep(zipStr, '''', ''), '"', '');
    if ~isempty(zipStr) && (isfile(zipStr) || isfolder(zipStr))
        curInfo = getActiveModelInfo();
        alreadyLoaded = (exist('ei_universal_mex', 'file') == 3) && ...
            isfield(curInfo, 'zip_path') && strcmpi(curInfo.zip_path, zipStr);
        if ~alreadyLoaded
            fprintf('[EI_Model] Auto-importing & compiling ZIP from block parameter: %s\n', zipStr);
            ei_import_model(zipStr);
        end
    end
end

function info = getActiveModelInfo()
    tbxDir = fileparts(mfilename('fullpath'));
    addpath(tbxDir);
    matFile = fullfile(tbxDir, 'active_model_info.mat');
    if isfile(matFile)
        try
            s = load(matFile, 'modelInfo');
            if isfield(s, 'modelInfo')
                info = s.modelInfo;
                return;
            end
        catch
        end
    end
    if exist('ei_universal_mex', 'file') == 3
        try
            info = ei_universal_mex('info');
            return;
        catch
        end
    end
    info = struct( ...
        'project_name', 'No Model Imported Yet', ...
        'project_id', 0, ...
        'num_channels', 1, ...
        'window_samples', 100, ...
        'frame_size', 100, ...
        'frequency_hz', 100, ...
        'num_classes', 4, ...
        'labels', {{'class_1', 'class_2', 'class_3', 'class_4'}}, ...
        'has_anomaly', 0, ...
        'is_object_detection', 0, ...
        'zip_path', '' ...
    );
end

function setup(block)
    try maybeAutoImportZip(block.BlockHandle); catch, end
    info = getActiveModelInfo();

    block.NumDialogPrms  = 0;
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 3;

    block.SetPreCompInpPortInfoToDynamic;
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).Complexity = 'Real';

    % Output Port 1: Predicted Class ID (1 .. N)
    block.OutputPort(1).Dimensions   = 1;
    block.OutputPort(1).DatatypeID   = 0;
    block.OutputPort(1).Complexity   = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';

    % Output Port 2: Confidence Score (0.0 .. 1.0)
    block.OutputPort(2).Dimensions   = 1;
    block.OutputPort(2).DatatypeID   = 0;
    block.OutputPort(2).Complexity   = 'Real';
    block.OutputPort(2).SamplingMode = 'Sample';

    % Output Port 3: Class Probabilities Vector (1 x num_classes)
    numClasses = max(1, double(info.num_classes));
    block.OutputPort(3).Dimensions   = numClasses;
    block.OutputPort(3).DatatypeID   = 0;
    block.OutputPort(3).Complexity   = 'Real';
    block.OutputPort(3).SamplingMode = 'Sample';

    block.SampleTimes = [-1, 0];
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('SetInputPortDimensions',   @SetInpPortDims);
    block.RegBlockMethod('SetInputPortDataType',     @SetInpPortDType);
    block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortSampMode);
    block.RegBlockMethod('Start',                    @Start);
    block.RegBlockMethod('Outputs',                  @Outputs);
    block.RegBlockMethod('Terminate',                @Terminate);
end

function SetInpPortDims(block, idx, di)
    block.InputPort(idx).Dimensions = di;
end

function SetInpPortDType(block, idx, dt)
    block.InputPort(idx).DatatypeID = dt;
end

function SetInpPortSampMode(block, idx, sm)
    block.InputPort(idx).SamplingMode = sm;
    block.OutputPort(1).SamplingMode  = sm;
    block.OutputPort(2).SamplingMode  = sm;
    block.OutputPort(3).SamplingMode  = sm;
end

function stateMap = getClassifierMap()
    global g_ei_classifier_states;
    if isempty(g_ei_classifier_states) || ~isa(g_ei_classifier_states, 'containers.Map')
        g_ei_classifier_states = containers.Map('KeyType', 'double', 'ValueType', 'any');
    end
    stateMap = getClassifierMapHelper();
end

function stateMap = getClassifierMapHelper()
    global g_ei_classifier_states;
    stateMap = g_ei_classifier_states;
end

function Start(block)
    stateMap = getClassifierMap();
    blkH = block.BlockHandle;

    tbxDir = fileparts(mfilename('fullpath'));
    addpath(tbxDir);

    % Check if user specified a .ZIP path on the block mask and auto-compile if new
    try
        maybeAutoImportZip(blkH);
    catch ME
        fprintf('[EI_Model Warning] Auto-import check failed: %s\n', ME.message);
    end

    if exist('ei_universal_mex', 'file') ~= 3
        error(['[EI_Model] No compiled Edge Impulse model found! ' ...
               'Double-click the EI_Model block and click "Browse & Build .ZIP Model" ' ...
               'or paste your .zip file path into the block dialog.']);
    end

    info = ei_universal_mex('info');

    stepSec = 0.25;
    verbose = true;
    showHud = true;
    try
        sStr = strtrim(char(get_param(blkH, 'ei_step_sec')));
        valS = str2double(sStr);
        if ~isnan(valS) && valS > 0, stepSec = valS; end

        vStr = strtrim(char(get_param(blkH, 'ei_verbose')));
        if strcmpi(vStr, 'off') || strcmpi(vStr, '0'), verbose = false; end

        hStr = strtrim(char(get_param(blkH, 'ei_show_hud')));
        if strcmpi(hStr, 'off') || strcmpi(hStr, '0'), showHud = false; end
    catch
    end

    labelList = '';
    for k = 1:numel(info.labels)
        if k > 1, labelList = [labelList ', ']; end
        labelList = [labelList sprintf('%d=%s', k, info.labels{k})];
    end

    if info.frequency_hz > 0
        fprintf('[EI_Model] Started: Model=''%s'', Window=%d samples (%.2fs @ %g Hz), Classes: [%s]\n', ...
            info.project_name, info.window_samples, info.window_samples / info.frequency_hz, ...
            info.frequency_hz, labelList);
    else
        fprintf('[EI_Model] Started: Model=''%s'', Frame=%d values (Single-Frame/Image), Classes: [%s]\n', ...
            info.project_name, info.frame_size, labelList);
    end

    hudHandles = struct('fig', [], 'axWave', [], 'axBar', [], 'hBar', [], 'hTitle', []);
    if showHud
        hudHandles = initLiveHud(info);
    end

    stateMap(blkH) = struct( ...
        'info', info, ...
        't_hist', [], ...
        'u_hist', [], ...
        'last_infer_t', -1e9, ...
        'step_sec', stepSec, ...
        'verbose', verbose, ...
        'show_hud', showHud, ...
        'hud', hudHandles, ...
        'pred_id', 0, ...
        'pred_label', 'Warming up buffer...', ...
        'pred_conf', 0.0, ...
        'pred_anom', NaN, ...
        'pred_scores', zeros(1, info.num_classes) ...
    );
end

function hud = initLiveHud(info)
    hud = struct('fig', [], 'axWave', [], 'axBar', [], 'hBar', [], 'hTitle', []);
    try
        figTag = 'EI_Simulink_Live_Prediction_HUD';
        fig = findall(0, 'Type', 'figure', 'Tag', figTag);
        if isempty(fig)
            fig = figure('Name', sprintf('Edge Impulse Live Model — %s', info.project_name), ...
                         'Tag', figTag, ...
                         'NumberTitle', 'off', ...
                         'Color', [0.10, 0.12, 0.16], ...
                         'Position', [150, 120, 720, 520]);
        else
            fig = fig(1);
            clf(fig);
            set(fig, 'Name', sprintf('Edge Impulse Live Model — %s', info.project_name));
        end

        % Top subplot: Rolling Input Waveform Window
        axWave = subplot(2, 1, 1, 'Parent', fig);
        set(axWave, 'Color', [0.06, 0.08, 0.11], 'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8], ...
            'GridColor', [0.3 0.35 0.4], 'FontSize', 10);
        grid(axWave, 'on');
        hTitle = title(axWave, 'PREDICTED LABEL: Warming up window buffer...', ...
            'Color', [0.3, 0.95, 0.5], 'FontSize', 15, 'FontWeight', 'bold', 'Interpreter', 'none');
        xlabel(axWave, 'Window Time (sec)', 'Color', [0.8 0.8 0.8]);
        ylabel(axWave, 'Signal Amplitude', 'Color', [0.8 0.8 0.8]);

        % Bottom subplot: Labeled Class Probability Bar Chart
        axBar = subplot(2, 1, 2, 'Parent', fig);
        set(axBar, 'Color', [0.06, 0.08, 0.11], 'XColor', [0.9 0.9 0.9], 'YColor', [0.8 0.8 0.8], ...
            'GridColor', [0.3 0.35 0.4], 'FontSize', 11);
        hBar = bar(axBar, zeros(1, info.num_classes), 'FaceColor', [0.2, 0.65, 0.95]);
        set(axBar, 'XTick', 1:info.num_classes, 'XTickLabel', info.labels, ...
            'TickLabelInterpreter', 'none', 'Color', [0.06, 0.08, 0.11], ...
            'XColor', [0.95 0.95 0.95], 'YColor', [0.85 0.85 0.85]);
        ylim(axBar, [0, 1.05]);
        ylabel(axBar, 'Probability (0 - 1)', 'Color', [0.85 0.85 0.85]);
        title(axBar, 'Live Class Probabilities', 'Color', [1 1 1], 'FontSize', 12);
        grid(axBar, 'on');

        hud.fig    = fig;
        hud.axWave = axWave;
        hud.axBar  = axBar;
        hud.hBar   = hBar;
        hud.hTitle = hTitle;
        drawnow limitrate;
    catch
    end
end

function Outputs(block)
    if ~block.IsMajorTimeStep
        return;
    end

    stateMap = getClassifierMap();
    blkH = block.BlockHandle;
    if ~isKey(stateMap, blkH)
        Start(block);
    end

    st = stateMap(blkH);
    info = st.info;

    t_now = block.CurrentTime;
    rawIn = block.InputPort(1).Data;

    if ~isempty(st.t_hist) && t_now < (st.t_hist(end) - 1e-12)
        st.t_hist = [];
        st.u_hist = [];
        st.last_infer_t = -1e9;
    end

    didInfer = false;
    win_mat = [];
    t_grid = [];
    isSingleFrame = (info.frequency_hz <= 0) || (info.window_samples <= 1);

    if isSingleFrame
        % Single-Frame / Image / Non-Time-Series Mode: pass frame directly
        if isempty(st.t_hist)
            st.t_hist = t_now;
        else
            st.t_hist(end) = t_now;
        end
        if (t_now - st.last_infer_t) >= (st.step_sec - 1e-9)
            res = ei_universal_mex(rawIn);
            didInfer = true;
        end
    else
        % Streaming Time-Series / Audio Mode: rolling window + resample
        u_raw = double(rawIn(:)');
        nCh = info.num_channels;
        if numel(u_raw) >= nCh
            u_now = u_raw(1:nCh);
        else
            u_now = [u_raw, zeros(1, nCh - numel(u_raw))];
        end

        if isempty(st.t_hist) || (t_now > st.t_hist(end) + 1e-12)
            st.t_hist(end+1, 1) = t_now;
            st.u_hist(end+1, :) = u_now;
        end

        winDurSec = (info.window_samples - 1) / info.frequency_hz;

        if ~isempty(st.t_hist) && (t_now - st.t_hist(1)) > (winDurSec + 1.0)
            keepMask = st.t_hist >= (t_now - winDurSec - 0.2);
            st.t_hist = st.t_hist(keepMask);
            st.u_hist = st.u_hist(keepMask, :);
        end

        if length(st.t_hist) >= 2 && (t_now - st.t_hist(1)) >= (winDurSec - 1e-6) && ...
           (t_now - st.last_infer_t) >= (st.step_sec - 1e-9)
            t_grid = linspace(t_now - winDurSec, t_now, info.window_samples)';
            win_mat = interp1(st.t_hist, st.u_hist, t_grid, 'linear', 'extrap');
            res = ei_universal_mex(win_mat);
            didInfer = true;
        end
    end

    if didInfer
        st.pred_id     = double(res.predicted_id);
        st.pred_label  = char(res.predicted_label);
        st.pred_conf   = double(res.best_confidence);
        st.pred_scores = double(res.scores(:)');
        st.pred_anom   = NaN;
        if isfield(res, 'anomaly')
            st.pred_anom = double(res.anomaly);
        end
        st.last_infer_t = t_now;

        isProb = (info.num_classes > 1) || (st.pred_conf >= 0.0 && st.pred_conf <= 1.0001);
        if isProb
            confTxt = sprintf('Confidence: %5.1f%%', st.pred_conf * 100.0);
        else
            confTxt = sprintf('Value: %.4g', st.pred_conf);
        end
        if isfinite(st.pred_anom)
            anomTxt = sprintf(' | Anomaly: %+.3f', st.pred_anom);
        else
            anomTxt = '';
        end

        if st.verbose
            fprintf('[EI_Model @ t=%5.2fs] >> %s << (Class %d) | %s%s | Scores: [%s]\n', ...
                t_now, upper(st.pred_label), st.pred_id, confTxt, anomTxt, ...
                strtrim(sprintf('%.2f ', st.pred_scores)));
        end

        % Update Live Prediction Monitor HUD Window
        if st.show_hud && ~isempty(st.hud.fig) && handle_valid(st.hud.fig)
            try
                if isSingleFrame
                    if ndims(rawIn) == 3 || (ismatrix(rawIn) && min(size(rawIn)) > 1)
                        imagesc(st.hud.axWave, rawIn);
                        axis(st.hud.axWave, 'image');
                    else
                        plot(st.hud.axWave, double(rawIn(:)), 'LineWidth', 1.5);
                    end
                else
                    plot(st.hud.axWave, t_grid, win_mat, 'LineWidth', 1.5);
                    xlim(st.hud.axWave, [t_grid(1), max(t_grid(end), t_grid(1) + 0.01)]);
                end
                set(st.hud.axWave, 'Color', [0.06, 0.08, 0.11], ...
                    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
                grid(st.hud.axWave, 'on');
                title(st.hud.axWave, ...
                    sprintf('PREDICTED:  >> %s <<   (Class #%d  |  %s%s  @  t = %.2fs)', ...
                        upper(st.pred_label), st.pred_id, confTxt, anomTxt, t_now), ...
                    'Color', [0.3, 0.95, 0.5], 'FontSize', 13, 'FontWeight', 'bold', 'Interpreter', 'none');

                set(st.hud.hBar, 'YData', st.pred_scores);
                minS = min(st.pred_scores);
                maxS = max(st.pred_scores);
                if minS < 0 || maxS > 1.05
                    pad = max(1.0, 0.1 * (maxS - minS));
                    ylim(st.hud.axBar, [min(0, minS - pad), max(1.05, maxS + pad)]);
                else
                    ylim(st.hud.axBar, [0, 1.05]);
                end
                drawnow limitrate;
            catch
            end
        end
    end

    block.OutputPort(1).Data = st.pred_id;
    block.OutputPort(2).Data = st.pred_conf;
    nOut3 = block.OutputPort(3).Dimensions;
    scoresOut = zeros(nOut3, 1);
    nCopy = min(nOut3, numel(st.pred_scores));
    scoresOut(1:nCopy) = st.pred_scores(1:nCopy)';
    block.OutputPort(3).Data = scoresOut;

    stateMap(blkH) = st;
end

function ok = handle_valid(h)
    ok = ~isempty(h) && all(ishandle(h));
end

function Terminate(block)
    stateMap = getClassifierMap();
    blkH = block.BlockHandle;
    if isKey(stateMap, blkH)
        remove(stateMap, blkH);
    end
end
