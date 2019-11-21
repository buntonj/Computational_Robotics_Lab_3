function [goal] = CreatePath(trajectoryLength)
aux1=[0:0.1:2*pi];
x1=[0.35+0.3.*sin(aux1) 0.35 0.35];
y1=[0.25-0.2.*cos(aux1) 0.25 0.25];
% x1 = [0.6 0.6 0.6 0.6];
% y1 = [0.4 0.4 0.4 0.4];
x1y1 = interp1([1:length(aux1) 70 75]' ,[x1;y1]',linspace(1,75,trajectoryLength)');
% x1y1 = interp1([1 2 70 75]' ,[x1;y1]',linspace(1,75,trajectoryLength)');

goal=[x1y1';zeros(1,trajectoryLength);zeros(1,trajectoryLength);zeros(1,trajectoryLength)];
end

