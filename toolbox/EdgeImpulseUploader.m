classdef EdgeImpulseUploader < handle
    % EdgeImpulseUploader (Plugin 1: Universal MATLAB/Simulink -> Edge Impulse Studio)
    %
    % Allows ANY data produced in MATLAB or Simulink to be formatted, saved,
    % and uploaded directly to Edge Impulse for model training & testing:
    %   1. Time-Series / Scalar Sensors (Temperature, Vibration, 6-Axis IMU, CAN bus, Voltage)
    %   2. Images / Thermal Maps / Spectrogram Figures (2D Grayscale, 3D RGB, or Figure handles)
    %   3. Audio / Acoustic Waveforms (.wav at any sampling rate)
    %   4. Direct Simulink Logged Signals (Simulink.SimulationData.Signal / timeseries)

    properties
        ApiKey (1,:) char = ''
        DeviceName (1,:) char = 'MATLAB_Universal_Plugin'
        DeviceType (1,:) char = 'MATLAB_SIMULINK'
        OutputDir (1,:) char = ''
    end

    methods
        function obj = EdgeImpulseUploader(apiKey, outDir)
            if nargin >= 1 && ~isempty(apiKey)
                obj.ApiKey = strtrim(char(apiKey));
            end
            if nargin >= 2 && ~isempty(outDir)
                obj.OutputDir = char(outDir);
            else
                baseDir = fileparts(mfilename('fullpath'));
                obj.OutputDir = fullfile(baseDir, 'universal_dataset');
            end
            if ~exist(obj.OutputDir, 'dir')
                mkdir(obj.OutputDir);
            end
        end

        % =================================================================
        % 1. UNIVERSAL TIME-SERIES & SCALAR SENSOR UPLOADER
        % =================================================================
        function result = sendTimeSeries(obj, dataInput, label, varargin)
            p = inputParser;
            addParameter(p, 'Fs', 100, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
            addParameter(p, 'Category', 'training', @(x) ischar(x) || isstring(x));
            addParameter(p, 'SensorNames', {}, @(x) iscell(x) || isstring(x));
            addParameter(p, 'Units', {}, @(x) iscell(x) || isstring(x));
            parse(p, varargin{:});

            label       = strtrim(char(string(label)));
            if isempty(label)
                warning('EdgeImpulseUploader:EmptyLabel', 'Empty label provided; using ''sample''.');
                label = 'sample';
            end
            fs          = double(p.Results.Fs);
            category    = lower(strtrim(char(string(p.Results.Category))));
            if ~ismember(category, {'training', 'testing', 'split'})
                warning('EdgeImpulseUploader:InvalidCategory', ...
                    'Invalid Category ''%s''. Using ''training''.', category);
                category = 'training';
            end
            sensorNames = p.Results.SensorNames;
            if isstring(sensorNames), sensorNames = cellstr(sensorNames(:)'); end
            sensorUnits = p.Results.Units;
            if isstring(sensorUnits), sensorUnits = cellstr(sensorUnits(:)'); end

            [rawMatrix, fs] = obj.extractSignalMatrix(dataInput, fs);

            if isempty(rawMatrix)
                error('EdgeImpulseUploader:EmptyData', 'Input time-series data is empty.');
            end
            if ~isfinite(fs) || fs <= 0
                error('EdgeImpulseUploader:InvalidFs', 'Sampling rate Fs must be positive and finite.');
            end

            nonFiniteMask = ~isfinite(rawMatrix);
            if any(nonFiniteMask(:))
                warning('EdgeImpulseUploader:NonFiniteData', ...
                    'Found %d NaN/Inf samples in signal; replacing with 0 for valid JSON.', sum(nonFiniteMask(:)));
                rawMatrix(nonFiniteMask) = 0.0;
            end

            [numSamples, numChannels] = size(rawMatrix);
            intervalMs = 1000.0 / fs;

            if isempty(sensorNames) || numel(sensorNames) ~= numChannels
                if ~isempty(sensorNames)
                    warning('EdgeImpulseUploader:SensorCountMismatch', ...
                        'SensorNames count (%d) does not match channel count (%d); using ch1..chN.', ...
                        numel(sensorNames), numChannels);
                end
                sensorNames = arrayfun(@(c) sprintf('ch%d', c), 1:numChannels, 'UniformOutput', false);
            else
                sensorNames = cellfun(@(s) char(string(s)), sensorNames, 'UniformOutput', false);
            end
            if isempty(sensorUnits) || numel(sensorUnits) ~= numChannels
                if ~isempty(sensorUnits)
                    warning('EdgeImpulseUploader:UnitCountMismatch', ...
                        'Units count (%d) does not match channel count (%d); using default ''unit''.', ...
                        numel(sensorUnits), numChannels);
                end
                sensorUnits = repmat({'unit'}, 1, numChannels);
            else
                sensorUnits = cellfun(@(s) char(string(s)), sensorUnits, 'UniformOutput', false);
            end

            sensorsStruct = struct('name', {}, 'units', {});
            for c = 1:numChannels
                sensorsStruct(c).name  = sensorNames{c};
                sensorsStruct(c).units = sensorUnits{c};
            end

            % Build values array as cell of cells so 1-channel signals serialize as [[v1],[v2],...]
            valuesCell = cell(numSamples, 1);
            for r = 1:numSamples
                valuesCell{r} = num2cell(double(rawMatrix(r, :)));
            end

            payload = struct();
            payload.protected = struct('ver', 'v1', 'alg', 'none');
            payload.signature = repmat('0', 1, 64);
            payload.payload = struct( ...
                'device_name', obj.DeviceName, ...
                'device_type', obj.DeviceType, ...
                'interval_ms', intervalMs, ...
                'sensors',     {num2cell(sensorsStruct)}, ...
                'values',      {valuesCell} ...
            );

            jsonStr = jsonencode(payload);

            subDir = fullfile(obj.OutputDir, category);
            if ~exist(subDir, 'dir'), mkdir(subDir); end

            % Sanitize filename so spaces or special characters in label never break HTTP headers
            safeLabel = regexprep(strtrim(label), '[^a-zA-Z0-9_-]', '_');
            if isempty(safeLabel), safeLabel = 'sample'; end
            uniqueTag = obj.makeUniqueTag(safeLabel);
            jsonFile  = fullfile(subDir, [uniqueTag '.json']);
            csvFile   = fullfile(subDir, [uniqueTag '.csv']);

            [fid, msg] = fopen(jsonFile, 'w', 'n', 'UTF-8');
            if fid == -1
                error('EdgeImpulseUploader:FileOpenFailed', 'Cannot write %s: %s', jsonFile, msg);
            end
            cJson = onCleanup(@() fclose(fid));
            fwrite(fid, unicode2native(jsonStr, 'UTF-8'), 'uint8');
            clear cJson;

            [fidCsv, msgCsv] = fopen(csvFile, 'w', 'n', 'UTF-8');
            if fidCsv == -1
                error('EdgeImpulseUploader:FileOpenFailed', 'Cannot write %s: %s', csvFile, msgCsv);
            end
            cCsv = onCleanup(@() fclose(fidCsv));
            fprintf(fidCsv, 'timestamp,%s\n', strjoin(sensorNames, ','));
            for r = 1:numSamples
                t_ms = round((r - 1) * intervalMs);
                valStr = sprintf(',%.5f', rawMatrix(r, :));
                fprintf(fidCsv, '%d%s\n', t_ms, valStr);
            end
            clear cCsv;

            result = struct('status', 'saved_locally', 'json_path', jsonFile, ...
                            'csv_path', csvFile, 'samples', numSamples, 'channels', numChannels);

            if ~isempty(obj.ApiKey)
                if strcmp(category, 'split')
                    result = obj.postFileToCloud(jsonFile, [uniqueTag '.json'], 'application/json', label, 'split', result);
                else
                    result = obj.postJsonToCloud(jsonStr, [uniqueTag '.json'], label, category, result);
                end
            else
                fprintf('[EI Plugin 1] Saved Locally (No API Key set): %s (%d samples x %d ch @ %.1f Hz) -> %s\n', ...
                    upper(label), numSamples, numChannels, fs, csvFile);
            end
        end

        % =================================================================
        % 2. UNIVERSAL IMAGE / THERMAL / SPECTROGRAM UPLOADER
        % =================================================================
        function result = sendImage(obj, imgInput, label, varargin)
            p = inputParser;
            addParameter(p, 'Category', 'training', @(x) ischar(x) || isstring(x));
            parse(p, varargin{:});
            label    = strtrim(char(string(label)));
            if isempty(label)
                warning('EdgeImpulseUploader:EmptyLabel', 'Empty label provided; using ''sample''.');
                label = 'sample';
            end
            category = lower(strtrim(char(string(p.Results.Category))));
            if ~ismember(category, {'training', 'testing', 'split'})
                category = 'training';
            end

            subDir = fullfile(obj.OutputDir, category);
            if ~exist(subDir, 'dir'), mkdir(subDir); end

            safeLabel = regexprep(strtrim(label), '[^a-zA-Z0-9_-]', '_');
            if isempty(safeLabel), safeLabel = 'sample'; end
            uniqueTag = [obj.makeUniqueTag([safeLabel '_img']) '.png'];
            imgPath   = fullfile(subDir, uniqueTag);

            if ischar(imgInput) || isstring(imgInput)
                srcPath = char(imgInput);
                if ~isfile(srcPath)
                    error('EdgeImpulseUploader:FileNotFound', 'Image file not found: %s', srcPath);
                end
                copyfile(srcPath, imgPath);
            elseif ishghandle(imgInput)
                saveas(imgInput, imgPath);
            elseif isnumeric(imgInput) || islogical(imgInput)
                imwrite(imgInput, imgPath);
            else
                error('Unsupported image input. Pass a numeric matrix, figure handle, or file path.');
            end

            imgInfo = imfinfo(imgPath);
            result = struct('status', 'saved_locally', 'file_path', imgPath, ...
                            'width', imgInfo.Width, 'height', imgInfo.Height);

            if ~isempty(obj.ApiKey)
                result = obj.postFileToCloud(imgPath, uniqueTag, 'image/png', label, category, result);
            else
                fprintf('[EI Plugin 1] Saved Image %s (%dx%d px) -> %s\n', ...
                    upper(label), imgInfo.Width, imgInfo.Height, imgPath);
            end
        end

        % =================================================================
        % 3. UNIVERSAL AUDIO / ACOUSTIC WAVEFORM UPLOADER (.WAV)
        % =================================================================
        function result = sendAudio(obj, audioWave, fs, label, varargin)
            p = inputParser;
            addParameter(p, 'Category', 'training', @(x) ischar(x) || isstring(x));
            parse(p, varargin{:});
            label    = strtrim(char(string(label)));
            if isempty(label)
                warning('EdgeImpulseUploader:EmptyLabel', 'Empty label provided; using ''sample''.');
                label = 'sample';
            end
            category = lower(strtrim(char(string(p.Results.Category))));
            if ~ismember(category, {'training', 'testing', 'split'})
                category = 'training';
            end
            if ~isnumeric(fs) || ~isscalar(fs) || ~isfinite(fs) || fs <= 0
                error('EdgeImpulseUploader:InvalidFs', 'Audio sampling rate fs must be positive.');
            end
            if isempty(audioWave)
                error('EdgeImpulseUploader:EmptyData', 'Input audio waveform is empty.');
            end

            subDir = fullfile(obj.OutputDir, category);
            if ~exist(subDir, 'dir'), mkdir(subDir); end

            safeLabel = regexprep(strtrim(label), '[^a-zA-Z0-9_-]', '_');
            if isempty(safeLabel), safeLabel = 'sample'; end
            uniqueTag = [obj.makeUniqueTag([safeLabel '_audio']) '.wav'];
            wavPath   = fullfile(subDir, uniqueTag);

            audioWave = double(audioWave(:));
            nonFiniteMask = ~isfinite(audioWave);
            if any(nonFiniteMask)
                warning('EdgeImpulseUploader:NonFiniteData', ...
                    'Found %d NaN/Inf samples in audio; replacing with 0.', sum(nonFiniteMask));
                audioWave(nonFiniteMask) = 0.0;
            end
            maxVal = max(abs(audioWave));
            if maxVal > 1.0
                audioWave = audioWave / maxVal;
            end
            audiowrite(wavPath, audioWave, round(fs));

            result = struct('status', 'saved_locally', 'file_path', wavPath, ...
                            'samples', length(audioWave), 'fs', fs);

            if ~isempty(obj.ApiKey)
                result = obj.postFileToCloud(wavPath, uniqueTag, 'audio/wav', label, category, result);
            else
                fprintf('[EI Plugin 1] Saved Audio %s (%.2fs @ %d Hz) -> %s\n', ...
                    upper(label), length(audioWave)/fs, round(fs), wavPath);
            end
        end
    end

    methods (Access = private)
        function tag = makeUniqueTag(~, prefix)
            ts = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
            tag = sprintf('%s_%s_%04d', prefix, ts, randi([0, 9999]));
        end

        function [mat, fs] = extractSignalMatrix(~, dataInput, defaultFs)
            fs = defaultFs;
            if isa(dataInput, 'Simulink.SimulationData.Signal') || isa(dataInput, 'timeseries')
                if isa(dataInput, 'Simulink.SimulationData.Signal')
                    ts = dataInput.Values;
                else
                    ts = dataInput;
                end
                t = double(ts.Time(:));
                rawD = double(ts.Data);
                nd = ndims(rawD);
                nT = length(t);
                if nd == 2
                    mat = rawD;
                    if isvector(mat)
                        if nT == 1
                            mat = mat(:)';
                        else
                            mat = mat(:);
                        end
                    elseif size(mat, 1) ~= nT && size(mat, 2) == nT
                        mat = mat';
                    end
                else
                    sz = size(rawD);
                    if sz(end) == nT
                        mat = permute(rawD, [nd, 1:(nd-1)]);
                        mat = reshape(mat, nT, []);
                    elseif sz(1) == nT
                        mat = reshape(rawD, nT, []);
                    else
                        mat = squeeze(rawD);
                        if isvector(mat)
                            mat = mat(:);
                        elseif size(mat, 1) ~= nT && size(mat, 2) == nT
                            mat = mat';
                        end
                    end
                end
                validIdx = isfinite(t);
                if ~all(validIdx)
                    t = t(validIdx);
                    mat = mat(validIdx, :);
                end
                if length(t) > 1
                    if any(diff(t) <= 0)
                        [t, sortIdx] = sort(t);
                        mat = mat(sortIdx, :);
                        [t, uniqIdx] = unique(t, 'stable');
                        mat = mat(uniqIdx, :);
                    end
                    if length(t) > 1
                        dt_mean = mean(diff(t));
                        if dt_mean > 0
                            fs = 1.0 / dt_mean;
                            if max(abs(diff(t) - dt_mean)) > 0.05 * dt_mean
                                t_uni = (t(1) : (1.0 / fs) : t(end))';
                                if length(t_uni) >= 2
                                    mat = interp1(t, mat, t_uni, 'linear', 'extrap');
                                end
                            end
                        end
                    end
                end
            elseif isnumeric(dataInput) || islogical(dataInput)
                mat = double(dataInput);
                if size(mat, 1) == 1 && size(mat, 2) > 6
                    mat = mat';
                end
            else
                error('Unsupported time-series input type.');
            end
        end

        function result = postJsonToCloud(obj, jsonStr, fileName, label, category, result)
            import matlab.net.*
            import matlab.net.http.*

            url = URI(sprintf('https://ingestion.edgeimpulse.com/api/%s/data', category));
            cleanLabel = regexprep(strtrim(char(string(label))), '[\r\n]+', ' ');
            cleanKey   = regexprep(strtrim(char(string(obj.ApiKey))), '[\r\n]+', '');
            headers = [
                HeaderField('x-api-key', cleanKey), ...
                HeaderField('x-label', cleanLabel), ...
                HeaderField('x-file-name', fileName), ...
                HeaderField('Content-Type', 'application/json')
            ];
            % CRITICAL: Set body.Payload directly to raw uint8 bytes so MATLAB
            % does not JSON-encode a string or uint8 array a second time!
            body = MessageBody();
            body.Payload = unicode2native(char(jsonStr), 'UTF-8');
            req  = RequestMessage('POST', headers, body);
            opts = HTTPOptions('ConnectTimeout', 20);

            try
                resp = req.send(url, opts);
                result.http_code = double(resp.StatusCode);
                result.response  = obj.extractResponseText(resp);

                if resp.StatusCode == StatusCode.OK
                    result.status = 'uploaded_to_cloud';
                    msg = sprintf('[EI Cloud OK] Uploaded %s (Label: %s) -> Edge Impulse %s set! (Response: %s)', ...
                        fileName, label, category, strtrim(result.response));
                    fprintf('%s\n', msg);
                    try sldiagviewer.reportInfo(msg); catch, end
                else
                    result.status = 'cloud_error';
                    msg = sprintf('[EI Cloud Error %d] %s', result.http_code, result.response);
                    fprintf('%s\n', msg);
                    try sldiagviewer.reportWarning(msg); catch, end
                end
            catch ME
                result.status = 'network_error';
                result.error_msg = ME.message;
                fprintf('[EI Network Error] %s\n', ME.message);
            end
        end

        function txt = extractResponseText(~, resp)
            txt = '';
            try
                if ~isempty(resp.Body) && ~isempty(resp.Body.Data)
                    d = resp.Body.Data;
                    if ischar(d)
                        txt = d;
                    elseif isstring(d)
                        txt = char(d);
                    elseif isstruct(d)
                        txt = jsonencode(d);
                    elseif isa(d, 'uint8')
                        txt = char(d(:)');
                    end
                end
            catch
            end
            if isempty(txt)
                txt = char(string(resp.StatusCode));
            end
        end

        function result = postFileToCloud(obj, filePath, fileName, mimeType, label, category, result)
            import matlab.net.*
            import matlab.net.http.*

            url = URI(sprintf('https://ingestion.edgeimpulse.com/api/%s/files', category));
            [fid, fmsg] = fopen(filePath, 'rb');
            if fid == -1
                error('EdgeImpulseUploader:FileOpenFailed', 'Cannot read %s: %s', filePath, fmsg);
            end
            cRead = onCleanup(@() fclose(fid));
            rawBytes = fread(fid, Inf, '*uint8')';
            clear cRead;

            boundary = sprintf('----MatlabEIBoundary%d', randi([100000, 999999]));
            crlf = sprintf('\r\n');
            headerPart = uint8([ ...
                '--' boundary crlf ...
                'Content-Disposition: form-data; name="data"; filename="' fileName '"' crlf ...
                'Content-Type: ' mimeType crlf crlf]);
            footerPart = uint8([crlf '--' boundary '--' crlf]);

            fullBody = [headerPart, rawBytes, footerPart];

            cleanLabel = regexprep(strtrim(char(string(label))), '[\r\n]+', ' ');
            cleanKey   = regexprep(strtrim(char(string(obj.ApiKey))), '[\r\n]+', '');
            headers = [
                HeaderField('x-api-key', cleanKey), ...
                HeaderField('x-label', cleanLabel), ...
                HeaderField('x-add-date-id', '1'), ...
                HeaderField('Content-Type', ['multipart/form-data; boundary=' boundary])
            ];
            body = MessageBody();
            body.Payload = fullBody;
            req = RequestMessage('POST', headers, body);
            opts = HTTPOptions('ConnectTimeout', 20);

            try
                resp = req.send(url, opts);
                result.http_code = double(resp.StatusCode);
                result.response  = obj.extractResponseText(resp);

                if resp.StatusCode == StatusCode.OK
                    result.status = 'uploaded_to_cloud';
                    fprintf('[EI Cloud OK] Uploaded File %s (%s) -> Edge Impulse %s set!\n', ...
                        fileName, upper(label), category);
                else
                    result.status = 'cloud_error';
                    fprintf('[EI Cloud Error %d] %s\n', result.http_code, result.response);
                end
            catch ME
                result.status = 'network_error';
                result.error_msg = ME.message;
                fprintf('[EI Network Error] %s\n', ME.message);
            end
        end
    end
end
