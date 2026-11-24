function modelInfo = ei_import_model(zipOrFolderPath)
    % ei_import_model (Plugin 2: Universal Edge Impulse C++ Model Importer)
    %
    % Imports ANY Edge Impulse C++ library deployment (.zip or folder),
    % automatically parses model_metadata.h (channels, window size, sample
    % frequency, class labels), patches MSVC headers, and compiles all 59
    % DSP + TFLite Micro C++ sources into 'ei_universal_mex'!
    %
    % Usage:
    %   info = ei_import_model('C:\path\to\my-edge-impulse-cpp.zip');
    %   info = ei_import_model(); % Opens a file browser to pick the .zip file

    tbxDir = fileparts(mfilename('fullpath'));
    if isempty(tbxDir)
        tbxDir = pwd;
    end
    addpath(tbxDir);

    if nargin < 1 || isempty(zipOrFolderPath)
        [fName, fDir] = uigetfile({'*.zip', 'Edge Impulse C++ Deployment (*.zip)'}, ...
            'Select Edge Impulse C++ Library Deployment ZIP');
        if isequal(fName, 0)
            fprintf('[ei_import_model] Cancelled by user.\n');
            modelInfo = [];
            return;
        end
        zipOrFolderPath = fullfile(fDir, fName);
    end

    zipOrFolderPath = char(zipOrFolderPath);

    fprintf('======================================================================\n');
    fprintf('  Edge Impulse Universal Model Importer (Plugin 2)                    \n');
    fprintf('======================================================================\n');

    % Unlock and unload any existing MEX binary in memory before recompiling
    try munlock('ei_universal_mex'); catch, end
    clear ei_universal_mex;
    try evalin('base', 'clear ei_universal_mex'); catch, end

    activeDir = fullfile(tbxDir, 'active_model');

    if isfolder(zipOrFolderPath)
        sdkRoot = zipOrFolderPath;
        fprintf('[1/4] Using model directory: %s\n', sdkRoot);
    elseif isfile(zipOrFolderPath)
        fprintf('[1/4] Extracting ZIP archive: %s\n', zipOrFolderPath);
        if exist(activeDir, 'dir')
            try rmdir(activeDir, 's'); catch, end
        end
        mkdir(activeDir);
        unzip(zipOrFolderPath, activeDir);
        sdkRoot = activeDir;
    else
        error('[ei_import_model] File or folder not found: %s', zipOrFolderPath);
    end

    if ~exist(fullfile(sdkRoot, 'edge-impulse-sdk'), 'dir')
        subDirs = dir(sdkRoot);
        for k = 1:numel(subDirs)
            if subDirs(k).isdir && ~ismember(subDirs(k).name, {'.', '..'})
                candidate = fullfile(sdkRoot, subDirs(k).name);
                if exist(fullfile(candidate, 'edge-impulse-sdk'), 'dir')
                    sdkRoot = candidate;
                    break;
                end
            end
        end
    end

    if ~exist(fullfile(sdkRoot, 'edge-impulse-sdk'), 'dir') || ...
       ~exist(fullfile(sdkRoot, 'model-parameters', 'model_metadata.h'), 'file')
        error('[ei_import_model] Invalid Edge Impulse C++ archive. Missing edge-impulse-sdk or model_metadata.h.');
    end

    fprintf('[2/4] Applying MSVC C++20 portability patches...\n');
    patchHeaderForMSVC(fullfile(sdkRoot, 'edge-impulse-sdk', 'dsp', 'config.hpp'));
    patchHeaderForMSVC(fullfile(sdkRoot, 'edge-impulse-sdk', 'porting', 'ei_classifier_porting.h'));
    patchAnomalyHeaderForMSVC(fullfile(sdkRoot, 'model-parameters', 'anomaly_metadata.h'));

    fprintf('[3/4] Collecting full DSP & TFLite Micro C++ kernel suite...\n');
    mexCpp = fullfile(tbxDir, 'ei_universal_mex.cpp');

    tflDir = fullfile(sdkRoot, 'tflite-model');
    tflCpp = dir(fullfile(tflDir, '*.cpp'));
    tflC   = dir(fullfile(tflDir, '*.c'));
    modelSources = {};
    for k = 1:numel(tflCpp), modelSources{end+1} = fullfile(tflDir, tflCpp(k).name); end
    for k = 1:numel(tflC),   modelSources{end+1} = fullfile(tflDir, tflC(k).name);   end

    % Full suite of 59 DSP & TFLite Micro kernel files so ANY neural network
    % architecture (Dense, 1D/2D CNN, MobileNet, Pooling, LSTM, Audio, Image) works!
    relCandidates = {
        'edge-impulse-sdk/dsp/memory.cpp'
        'edge-impulse-sdk/dsp/kissfft/kiss_fft.cpp'
        'edge-impulse-sdk/dsp/kissfft/kiss_fftr.cpp'
        'edge-impulse-sdk/dsp/dct/fast-dct-fft.cpp'
        'edge-impulse-sdk/tensorflow/lite/core/api/common.cc'
        'edge-impulse-sdk/tensorflow/lite/core/api/error_reporter.cc'
        'edge-impulse-sdk/tensorflow/lite/core/api/flatbuffer_conversions.cc'
        'edge-impulse-sdk/tensorflow/lite/kernels/kernel_util_lite.cc'
        'edge-impulse-sdk/tensorflow/lite/kernels/internal/quantization_util.cc'
        'edge-impulse-sdk/tensorflow/lite/kernels/internal/portable_tensor_utils.cc'
        'edge-impulse-sdk/tensorflow/lite/kernels/internal/tensor_ctypes.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/kernel_util_micro.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/micro_tensor_utils.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/fully_connected.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/fully_connected_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/softmax.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/softmax_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/conv.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/conv_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/depthwise_conv.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/depthwise_conv_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/pooling.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/pooling_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/reshape.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/pad.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/strided_slice.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/pack.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/unpack.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/concatenation.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/split.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/squeeze.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/expand_dims.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/transpose.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/add.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/add_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/mul.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/mul_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/sub.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/sub_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/quantize.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/quantize_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/dequantize.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/dequantize_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/activations.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/activations_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/logistic.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/logistic_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/tanh.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/leaky_relu.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/leaky_relu_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/reduce.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/reduce_common.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/elementwise.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/maximum_minimum.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/kernels/arg_min_max.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/micro_log.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/micro_string.cc'
        'edge-impulse-sdk/tensorflow/lite/micro/micro_utils.cc'
    };

    sdkSources = {};
    for k = 1:numel(relCandidates)
        fullP = fullfile(sdkRoot, strrep(relCandidates{k}, '/', filesep));
        if isfile(fullP)
            sdkSources{end+1} = fullP;
        end
    end

    allSources = [{mexCpp}, modelSources, sdkSources];

    incFlags = {
        ['-I' sdkRoot]
        ['-I' fullfile(sdkRoot, 'edge-impulse-sdk')]
        ['-I' fullfile(sdkRoot, 'edge-impulse-sdk', 'third_party', 'flatbuffers', 'include')]
        ['-I' fullfile(sdkRoot, 'edge-impulse-sdk', 'third_party', 'gemmlowp')]
        ['-I' fullfile(sdkRoot, 'edge-impulse-sdk', 'third_party', 'ruy')]
        ['-I' fullfile(sdkRoot, 'model-parameters')]
        ['-I' fullfile(sdkRoot, 'tflite-model')]
    };

    defFlags = {
        '-DEIDSP_QUANTIZE_FILTERBANK=0'
        '-DEI_CLASSIFIER_TFLITE_MICRO_C_H=1'
        '-DEI_CLASSIFIER_ENABLE_DETECTION_POSTPROCESS_OP=1'
        '-DTF_LITE_STATIC_MEMORY=1'
        '-DTF_LITE_DISABLE_X86_NEON=1'
    };

    cc = mex.getCompilerConfigurations('C++', 'Selected');
    if ~isempty(cc) && contains(cc.Name, 'Visual', 'IgnoreCase', true)
        % Use targeted '/wd...' instead of '/w' so cl.exe never prints 'warning D9025'
        cxxFlags = {'COMPFLAGS="$COMPFLAGS /std:c++20 /EHsc /wd4244 /wd4267 /wd4305 /wd4018 /wd4996 /wd4101 /wd4146"'};
    else
        cxxFlags = {'CXXFLAGS="$CXXFLAGS -std=c++17 -w"'};
    end

    outMex = fullfile(tbxDir, 'ei_universal_mex');
    fprintf('[4/4] Compiling all %d C++ source & kernel files (takes ~25 seconds, please wait)...\n', numel(allSources));

    mexArgs = [{'-silent', '-O'}, cxxFlags, incFlags', defFlags', {'-output', outMex}, allSources];
    mex(mexArgs{:});

    modelInfo = ei_universal_mex('info');
    modelInfo.zip_path = zipOrFolderPath;
    save(fullfile(tbxDir, 'active_model_info.mat'), 'modelInfo');

    fprintf('\n======================================================================\n');
    fprintf('  MODEL IMPORTED & COMPILED SUCCESSFULLY!                             \n');
    fprintf('----------------------------------------------------------------------\n');
    fprintf('  Project Name      : %s (ID: %d)\n', modelInfo.project_name, modelInfo.project_id);
    fprintf('  Input Channels    : %d channel(s)\n', modelInfo.num_channels);
    if modelInfo.frequency_hz > 0
        fprintf('  Window Size       : %d samples (%.2f sec @ %g Hz)\n', ...
            modelInfo.window_samples, modelInfo.window_samples / modelInfo.frequency_hz, modelInfo.frequency_hz);
    else
        fprintf('  Input Frame Size  : %d values (Image / Non-Time-Series)\n', modelInfo.frame_size);
    end
    fprintf('  Output Classes    : %d -> [%s]\n', ...
        modelInfo.num_classes, strjoin(modelInfo.labels, ', '));
    if isfield(modelInfo, 'has_anomaly') && modelInfo.has_anomaly
        fprintf('  Anomaly Detection : ENABLED (K-means / GMM score returned in .anomaly)\n');
    end
    if isfield(modelInfo, 'is_object_detection') && modelInfo.is_object_detection
        fprintf('  Object Detection  : ENABLED (FOMO / YOLO boxes returned in .bounding_boxes)\n');
    end
    fprintf('  Compiled Binary   : %s.%s\n', outMex, mexext);
    fprintf('======================================================================\n\n');
end

function patchHeaderForMSVC(headerPath)
    if ~isfile(headerPath)
        return;
    end
    txt = fileread(headerPath);
    if ~contains(txt, '#define __attribute__(x)')
        patchStr = sprintf('#if defined(_MSC_VER) && !defined(__attribute__)\n#define __attribute__(x)\n#endif\n');
        fid = fopen(headerPath, 'w');
        fwrite(fid, [patchStr, txt], 'char');
        fclose(fid);
    end
end

function patchAnomalyHeaderForMSVC(headerPath)
    % Converts GCC/C99 compound literals `( float[N] ) { ... }` inside
    % Edge Impulse's generated anomaly_metadata.h into standard C++ static
    % float arrays so MSVC (cl.exe) compiles K-means anomaly models cleanly
    % without error C4576.
    if ~isfile(headerPath)
        return;
    end
    txt = fileread(headerPath);
    pat = '\(\s*float\s*\[\s*(\d+)\s*\]\s*\)\s*\{([^}]+)\}';
    [starts, ends, tokens] = regexp(txt, pat, 'start', 'end', 'tokens');
    if isempty(starts)
        return;
    end

    decls = cell(numel(starts), 1);
    % Build replacement from back to front so indices stay valid
    newTxt = txt;
    for k = numel(starts):-1:1
        arrLen  = tokens{k}{1};
        arrVals = tokens{k}{2};
        varName = sprintf('_ei_msvc_anom_c_%d', k);
        decls{k} = sprintf('static float %s[%s] = {%s};\n', varName, arrLen, arrVals);
        newTxt = [newTxt(1:starts(k)-1), varName, newTxt(ends(k)+1:end)];
    end

    declBlock = sprintf('// [MSVC C++ compatibility: hoisted from C99 compound literals]\n%s\n', strjoin(decls, ''));
    anchor = regexp(newTxt, 'const\s+ei_classifier_anom_cluster_t', 'once');
    if ~isempty(anchor)
        newTxt = [newTxt(1:anchor-1), declBlock, newTxt(anchor:end)];
    else
        newTxt = [declBlock, newTxt];
    end

    fid = fopen(headerPath, 'w');
    fwrite(fid, newTxt, 'char');
    fclose(fid);
end
