function PlotLaser(roomBounds,state)
%PLOTLASER Summary of this function goes here
%   Detailed explanation goes here
xlimit = roomBounds(1,:);
ylimit = roomBounds(2,:);
xbox = xlimit([1 1 2 2 1]);
ybox = ylimit([1 2 2 1 1]);
mapshow(xbox,ybox,'DisplayType','polygon','LineStyle','none')

x = [state(1) state(1)+cos(state(3)+pi/2)];
y = [state(2) state(2)+sin(state(3)+pi/2)];


[xi,yi] = polyxpoly(x,y,xbox,ybox);
mapshow(xi,yi,'DisplayType','point','Color','r','Marker','o')
mapshow([x(1) xi],[y(1) yi],'Color','r','Marker','+')
axis([-0.1 0.85 -0.1 0.6])
x = [state(1) state(1)+cos(state(3))];
y = [state(2) state(2)+sin(state(3))];


[xi,yi] = polyxpoly(x,y,xbox,ybox);
mapshow(xi,yi,'DisplayType','point','Marker','o','Color','b')
mapshow([x(1) xi],[y(1) yi],'Color','b','Marker','+')

end

