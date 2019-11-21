function [inputs] = ComputeInput(y,goal)
%COMPUTEINPUT Summary of this function goes here
%   Detailed explanation goes here
r = 0.025;
w = 0.090;
Gain1 = 30/r;
Gain2 = 30*w/r;
reverse = false;
y(3) = mod(y(3),2*pi);
if y(3)>pi
    y(3)=y(3)-2*pi;
elseif y(3)<-pi
    y(3)=2*pi+y(3);
end
Distance = goal(1:2)-[y(1);y(2)];
angleGoal=mod(atan2(goal(2)-y(2),goal(1)-y(1))-pi/2,2*pi);
if angleGoal>pi
    angleGoal=angleGoal-2*pi;
end
if isnan(angleGoal)
    auxInputs(2) = 0;
elseif abs(angleGoal-y(3))<=pi/2
    auxInputs(2) = Gain2*sign(angleGoal-y(3))*abs(angleGoal-y(3));
elseif angleGoal-y(3)>pi/2 && angleGoal-y(3)<3/2*pi
    reverse = true;
    angleGoal = angleGoal-pi;
    auxInputs(2) = Gain2*sign(angleGoal-y(3))*abs(angleGoal-y(3));
elseif angleGoal-y(3)<-pi/2 && angleGoal-y(3)>-3/2*pi
    reverse = true;
    angleGoal = angleGoal+pi;
    auxInputs(2) = Gain2*sign(angleGoal-y(3))*abs(angleGoal-y(3));
elseif angleGoal-y(3)>3/2*pi
    reverse = false;
    angleGoal = 2*pi-angleGoal;
    auxInputs(2) = Gain2*sign(angleGoal-y(3))*abs(angleGoal-y(3));
elseif angleGoal-y(3)<-3/2*pi
    reverse = true;
    angleGoal = 2*pi-angleGoal;
    auxInputs(2) = Gain2*sign(angleGoal-y(3))*abs(angleGoal-y(3));
end
auxInputs(2) = sign(auxInputs(2))*min(abs(auxInputs(2)),120);
auxInputs(1) = min(norm(Gain1*Distance),120-abs(auxInputs(2)));
if     reverse
    auxInputs(1)=-auxInputs(1);
end
inputs(1)=(auxInputs(1)+auxInputs(2))/2;
inputs(2)=(auxInputs(1)-auxInputs(2))/2;
end

