function [rangeForward,rangeRight] = ComputeLaser(roomBounds,state)
%PLOTLASER Summary of this function goes here
%   Detailed explanation goes here
xlimit = roomBounds(1,:);
ylimit = roomBounds(2,:);
xbox = xlimit([1 1 2 2 1]);
ybox = ylimit([1 2 2 1 1]);

x = [state(1) state(1)+cos(state(3)+pi/2)];
y = [state(2) state(2)+sin(state(3)+pi/2)];


[xi,yi] = polyxpoly(x,y,xbox,ybox);

rangeForward = norm([xi,yi]-[x(1),y(1)]);
x = [state(1) state(1)+cos(state(3))];
y = [state(2) state(2)+sin(state(3))];


[xi,yi] = polyxpoly(x,y,xbox,ybox);

rangeRight = norm([xi,yi]-[x(1),y(1)]);
rangeForward=rangeForward+normrnd(0,rangeForward*0.03);
rangeRight=rangeRight+normrnd(0,rangeRight*0.03);
end
