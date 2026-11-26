function ei_browse_and_import_callback(blkH)
    % ei_browse_and_import_callback
    % Triggered when the user clicks '[ Browse & Build Edge Impulse .ZIP Model... ]'
    % inside the Simulink EI_Model block dialog.

    tbxDir = fileparts(mfilename('fullpath'));
    addpath(tbxDir);

    [fName, fDir] = uigetfile({'*.zip', 'Edge Impulse C++ Deployment (*.zip)'}, ...
        'Select Edge Impulse C++ Library Deployment ZIP');
    if isequal(fName, 0)
        return;
    end

    zipFull = fullfile(fDir, fName);
    try
        set_param(blkH, 'ei_zip_path', zipFull);
    catch
    end

    % Import and compile the selected ZIP right away
    try
        info = ei_import_model(zipFull);
    catch ME
        errordlg(ME.message, 'Edge Impulse Model Build Error');
        return;
    end

    if ~isempty(info)
        if info.frequency_hz > 0
            winLine = sprintf('Window  : %d samples (%.2fs @ %g Hz)', ...
                info.window_samples, info.window_samples / info.frequency_hz, info.frequency_hz);
        else
            winLine = sprintf('Frame   : %d values (Image / Single-Frame)', info.frame_size);
        end
        extraLines = '';
        if isfield(info, 'has_anomaly') && info.has_anomaly
            extraLines = [extraLines newline 'Anomaly : ENABLED (K-means / GMM)'];
        end
        if isfield(info, 'is_object_detection') && info.is_object_detection
            extraLines = [extraLines newline 'Obj Det : ENABLED (Bounding Boxes)'];
        end
        msg = sprintf(['Successfully imported & compiled Edge Impulse model!\n\n' ...
                       'Project : %s (ID: %d)\n' ...
                       'Channels: %d\n' ...
                       '%s\n' ...
                       'Classes : %s%s'], ...
            info.project_name, info.project_id, info.num_channels, ...
            winLine, strjoin(info.labels, ', '), extraLines);
        msgbox(msg, 'Edge Impulse Model Ready', 'help');
    end
end
