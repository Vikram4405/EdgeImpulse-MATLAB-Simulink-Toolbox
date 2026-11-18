% build_bearing_sfun.m
% Compiles the Motor Bearing & 3-Axis MEMS Accelerometer C-MEX S-Function.

fprintf('=== Compiling Motor Bearing & 3-Axis Accelerometer S-Function ===\n');

baseDir = fileparts(mfilename('fullpath'));
if isempty(baseDir)
    baseDir = pwd;
end

srcFile = fullfile(baseDir, 'motor_bearing_accel_sfun.cpp');
outName = fullfile(baseDir, 'motor_bearing_accel_sfun');

mex('-O', srcFile, '-output', outName);

fprintf('SUCCESS: Compiled %s.%s!\n\n', outName, mexext);
