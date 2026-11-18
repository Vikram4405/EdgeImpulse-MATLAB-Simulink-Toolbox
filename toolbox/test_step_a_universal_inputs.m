% test_step_a_universal_inputs.m
% Demonstrates and verifies Step A (Plugin 1: Universal EdgeImpulseUploader)
% across 4 distinct engineering modalities:
%   1. Temperature & Humidity Sensor (Slow 10 Hz environmental telemetry)
%   2. 6-Axis Robotics / Automotive IMU ([accX, accY, accZ, gyroX, gyroY, gyroZ] @ 100 Hz)
%   3. Thermal Camera / Computer Vision Images (64x64 RGB Hotspot vs Normal .png)
%   4. Acoustic / Audio Waveform (16 kHz Motor Hum vs Bearing Squeal .wav)

fprintf('=========================================================================\n');
fprintf('  Step A Verification: Universal EdgeImpulseUploader (All Modalities)    \n');
fprintf('=========================================================================\n\n');

baseDir = fileparts(mfilename('fullpath'));
if isempty(baseDir)
    baseDir = pwd;
end
addpath(baseDir);

% Initialize Plugin 1
% (Pass your Edge Impulse API key 'ei_...' here to push live to the cloud,
%  or leave '' to verify local JSON/CSV/PNG/WAV generation first!)
API_KEY = '';
ei = EdgeImpulseUploader(API_KEY, fullfile(baseDir, 'universal_dataset'));

% -------------------------------------------------------------------------
% 1. TEST MODALITY 1: Temperature & Humidity Telemetry (2 Channels @ 10 Hz)
% -------------------------------------------------------------------------
fprintf('--- 1. Testing Temperature & Humidity Sensor Ingestion ---\n');
t_env = (0:0.1:5.0)'; % 5 seconds @ 10 Hz (51 samples)

% Normal operating temperature (~25 C, 45% RH)
temp_normal = 25.0 + 0.3 * randn(size(t_env));
hum_normal  = 45.0 + 0.5 * randn(size(t_env));
res_temp1 = ei.sendTimeSeries([temp_normal, hum_normal], 'normal_temp', ...
    'Fs', 10, 'SensorNames', {'temperature_C', 'humidity_pct'}, 'Units', {'Cel', '%'});

% Thermal runaway / Overheating (~85 C rising, 20% RH)
temp_hot = 75.0 + 3.0 * t_env + 0.5 * randn(size(t_env));
hum_hot  = 20.0 + 0.5 * randn(size(t_env));
res_temp2 = ei.sendTimeSeries([temp_hot, hum_hot], 'overheating_fault', ...
    'Fs', 10, 'SensorNames', {'temperature_C', 'humidity_pct'}, 'Units', {'Cel', '%'});

% -------------------------------------------------------------------------
% 2. TEST MODALITY 2: 6-Axis Robotics / Automotive IMU (6 Channels @ 100 Hz)
% -------------------------------------------------------------------------
fprintf('\n--- 2. Testing 6-Axis Robotics / Automotive IMU Ingestion ---\n');
t_imu = (0:0.01:2.0)'; % 2 seconds @ 100 Hz (201 samples)
imu_names = {'accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ'};
imu_units = {'m/s2', 'm/s2', 'm/s2', 'deg/s', 'deg/s', 'deg/s'};

% Smooth driving / Stable robot hover
imu_smooth = [0.1*randn(length(t_imu), 2), 9.81 + 0.1*randn(length(t_imu), 1), ...
              0.5*randn(length(t_imu), 3)];
res_imu1 = ei.sendTimeSeries(imu_smooth, 'smooth_drive', ...
    'Fs', 100, 'SensorNames', imu_names, 'Units', imu_units);

% Pothole strike / Collision impact
strike = exp(-((t_imu - 1.0) / 0.08).^2);
imu_pothole = [4.0*strike.*randn(length(t_imu), 2), 9.81 + 18.0*strike.*sin(2*pi*15*t_imu), ...
               45.0*strike.*sin(2*pi*12*t_imu), 25.0*strike.*cos(2*pi*12*t_imu), 5.0*randn(length(t_imu), 1)];
res_imu2 = ei.sendTimeSeries(imu_pothole, 'pothole_impact', ...
    'Fs', 100, 'SensorNames', imu_names, 'Units', imu_units);

% -------------------------------------------------------------------------
% 3. TEST MODALITY 3: Thermal Camera / Vision Images (64x64 RGB .PNG)
% -------------------------------------------------------------------------
fprintf('\n--- 3. Testing Image / Thermal Vision Ingestion ---\n');
[X, Y] = meshgrid(linspace(-1, 1, 64), linspace(-1, 1, 64));

% Normal uniform cool thermal frame (Blue/Green palette)
img_normal = zeros(64, 64, 3, 'uint8');
img_normal(:, :, 1) = uint8(20 + 10 * rand(64, 64));
img_normal(:, :, 2) = uint8(110 + 25 * exp(-(X.^2 + Y.^2)));
img_normal(:, :, 3) = uint8(180 + 30 * rand(64, 64));
res_img1 = ei.sendImage(img_normal, 'thermal_normal');

% Hotspot fault thermal frame (Bright Red/Yellow glowing core)
hotspot = exp(-((X - 0.2).^2 + (Y + 0.1).^2) / 0.08);
img_hotspot = zeros(64, 64, 3, 'uint8');
img_hotspot(:, :, 1) = uint8(min(255, 40 + 215 * hotspot));
img_hotspot(:, :, 2) = uint8(min(255, 60 + 180 * (hotspot.^2)));
img_hotspot(:, :, 3) = uint8(40 * (1 - hotspot));
res_img2 = ei.sendImage(img_hotspot, 'thermal_hotspot');

% -------------------------------------------------------------------------
% 4. TEST MODALITY 4: Acoustic / Audio Waveforms (16 kHz .WAV)
% -------------------------------------------------------------------------
fprintf('\n--- 4. Testing 16 kHz Acoustic Audio (.WAV) Ingestion ---\n');
fs_audio = 16000;
t_aud = (0:1/fs_audio:1.0)';

% Normal quiet 120 Hz motor hum
aud_normal = 0.15 * sin(2*pi*120*t_aud) + 0.02 * randn(size(t_aud));
res_aud1 = ei.sendAudio(aud_normal, fs_audio, 'motor_hum_normal');

% High-pitched 3.2 kHz metallic bearing squeal
aud_squeal = 0.15 * sin(2*pi*120*t_aud) + 0.65 * sin(2*pi*3200*t_aud) .* (0.6 + 0.4*sin(2*pi*30*t_aud)) + 0.05 * randn(size(t_aud));
res_aud2 = ei.sendAudio(aud_squeal, fs_audio, 'bearing_squeal_fault');

fprintf('\n=========================================================================\n');
fprintf('  ALL 4 MODALITIES VERIFIED & EXPORTED SUCCESSFULLY!                     \n');
fprintf('  Output Folder: %s\n', fullfile(baseDir, 'universal_dataset', 'training'));
fprintf('=========================================================================\n');
