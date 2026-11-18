function ei_browse_and_import_callback(blkH)
    % ei_browse_and_import_callback
    % Triggered when the user clicks '[ Browse & Build Edge Impulse .ZIP Model... ]'
    % inside the Simulink EI_Classifier block dialog.

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
    info = ei_import_model(zipFull);

    if ~isempty(info)
        msg = sprintf(['Successfully imported & compiled Edge Impulse model!\n\n' ...
                       'Project : %s (ID: %d)\n' ...
                       'Channels: %d\n' ...
                       'Window  : %d samples (%.2fs @ %g Hz)\n' ...
                       'Classes : %s'], ...
            info.project_name, info.project_id, info.num_channels, ...
            info.window_samples, info.window_samples / info.frequency_hz, ...
            info.frequency_hz, strjoin(info.labels, ', '));
        msgbox(msg, 'Edge Impulse Model Ready', 'help');
    end
end
