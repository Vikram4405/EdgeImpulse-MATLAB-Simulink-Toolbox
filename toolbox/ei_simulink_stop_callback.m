function ei_simulink_stop_callback(blockHandle)
    % ei_simulink_stop_callback
    % Automatically invoked by the Simulink 'EI_Data_Logger' block's StopFcn
    % as soon as a Simulink simulation finishes or is stopped by the user.

    try
        % Read Mask Parameters from the block
        labelStr    = strtrim(get_param(blockHandle, 'ei_label'));
        categoryStr = strtrim(get_param(blockHandle, 'ei_category'));
        sensorsRaw  = strtrim(get_param(blockHandle, 'ei_sensors'));
        unitsRaw    = strtrim(get_param(blockHandle, 'ei_units'));
        apiKeyStr   = strtrim(get_param(blockHandle, 'ei_apikey'));
        outDirStr   = strtrim(get_param(blockHandle, 'ei_outdir'));

        % Clean quotes if user entered them
        labelStr  = strrep(strrep(labelStr, '''', ''), '"', '');
        apiKeyStr = strrep(strrep(apiKeyStr, '''', ''), '"', '');
        outDirStr = strrep(strrep(outDirStr, '''', ''), '"', '');

        if isempty(labelStr)
            labelStr = 'simulink_signal';
        end

        % Fetch the recorded timeseries from base workspace or SimulationOutput
        if evalin('base', 'exist(''ei_simulink_logged_ts'', ''var'')')
            tsData = evalin('base', 'ei_simulink_logged_ts');
        elseif evalin('base', 'exist(''out'', ''var'')')
            outObj = evalin('base', 'out');
            if isprop(outObj, 'ei_simulink_logged_ts')
                tsData = outObj.ei_simulink_logged_ts;
            else
                fprintf('[EI_Data_Logger] No logged signal found in workspace.\n');
                return;
            end
        else
            fprintf('[EI_Data_Logger] No logged signal found in workspace.\n');
            return;
        end

        % Parse comma-separated sensor names and units
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

        % Ensure toolbox directory is on path
        tbxDir = fileparts(mfilename('fullpath'));
        addpath(tbxDir);

        if isempty(outDirStr) || strcmpi(outDirStr, 'default')
            outDirStr = fullfile(tbxDir, 'universal_dataset');
        end

        uploader = EdgeImpulseUploader(apiKeyStr, outDirStr);
        uploader.sendTimeSeries(tsData, labelStr, ...
            'Category', categoryStr, ...
            'SensorNames', sensorNames, ...
            'Units', sensorUnits);

    catch ME
        fprintf('[EI_Data_Logger Warning] Could not auto-export signal: %s\n', ME.message);
    end
end
