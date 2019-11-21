function [magnetometerHeading,gyroMeasurement] = ComputeGyroscope(realState)
%COMPUTEACCELEROMETER Summary of this function goes here
%   Detailed explanation goes here
RMSGyroscopeNoise = 0.1;%?/s-rms
RMSGyroscopeNoise = RMSGyroscopeNoise/360*2*pi;% rad/s-rms 
gyroMeasurement = realState(4)+normrnd(0,RMSGyroscopeNoise);
magnetometerHeading = realState(3);

end

