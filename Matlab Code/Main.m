clc
clear
close all
profile on


%% Initialization of environment and robot geometric variables
% dbstop if error
roomLength=0.75;
roomWidth=0.5;
robotLength=0.1;
robotWidth=0.09;
robotWheelRadius=0.025;
roomBounds=[0 roomLength; 0 roomWidth];
roomVertices=[0 roomWidth; roomLength roomWidth; roomLength 0; 0 0];
RPMtoRadiansPerSecond = 2*pi/60;
RMSGyroscopeNoise = 0.1;%1/s-rms
RMSGyroscopeNoise = RMSGyroscopeNoise/360*2*pi;% rad/s-rms
robotDriveTime = 1; %simulation time at each RRT iteration

%Requires export_fig toolbox, download from https://www.mathworks.com/matlabcentral/fileexchange/23629-export_fig
filename = strcat('UKF','.gif'); % Name of the gif we will create


%% Initialize state and goal

state=[0.35;0.1;0;0;0];                 % initial state
StateEstimate=[0.35;0.1;0;0;0];         %initial state estimate
% goal=[0.35;0.25;0;0;0];               %controller set-point goal
trajectoryLength = 150;                 %trajectory granularity
goal = CreatePath(trajectoryLength);    %controller trajectory tracking
n=5;                                    %number of states
m=4;                                    %number of measurements
%% Initialize Covariances

Q=zeros(n);                             % covariance of process
Q(4,4)=60*RPMtoRadiansPerSecond*0.05;   % Slippage std
Q(5,5)=60*RPMtoRadiansPerSecond*0.05;   % Slippage std
R=zeros(m);                             % covariance of measurement
R(1,1) = 2*pi/9600;                     % Magnetometer std
R(2,2) = RMSGyroscopeNoise;             % Gyroscope std
R(3,3) = 0.6;                           % Rangefinder std
R(4,4) = 0.6;                           % Rangefinder std
integrationTime = 0.1;
P = eye(n)/10^10;                       % initial state covariance

%% allocate memory
stateEstimateHistory = zeros(n,trajectoryLength);                                 %state estimate
stateHistory = zeros(n,trajectoryLength);                                         %actual state
stateHistory(:,1)= state;                                                         % save actual state
stateEstimateHistory(:,1) = StateEstimate;                                        % save estimate
measurementHistory = zeros(m,trajectoryLength);

%% Create grid for distribution plot
[X, Y] = ndgrid(0:0.001:roomLength,0:0.001:roomWidth);
probabilityMap = zeros(size(X));

%% Create figure for GIF
figure('units','normalized','outerposition',[0 0 1 1]);
hold on
axis square equal tight manual
axis([-0.1 0.8 -0.1 0.6 -1 1.2])
plot3(goal(1,:),goal(2,:),ones(trajectoryLength,1),'g-.','Linewidth',2)
scatter3(stateHistory(1,1),stateHistory(2,1),1,'fill','r')% update process
scatter3(stateEstimateHistory(1,1),stateEstimateHistory(2,1),1,'fill','k')% update process

%% Run Robot
for k=2:trajectoryLength
    %% Get measurements
    [rangeForward,rangeRight] = ComputeLaser(roomBounds,state);
    [magnetometerHeading,gyroMeasurement] = ComputeGyroscope(state);
    z=[magnetometerHeading;gyroMeasurement;rangeForward;rangeRight];        % measurments
    
    %% Estimate State
    [StateEstimate,P]=UncentedKalmanFilterWork(StateEstimate,P,z,Q,R,goal(:,k),integrationTime,roomBounds);
    
    %% Evolve State
    [newState,trajectory,u] = RobotDynamicsStep(StateEstimate,goal(:,k),integrationTime);
    state = newState';
    
    %% Save variables
    stateHistory(:,k)= state;                                                                 % save actual state
    stateEstimateHistory(:,k) = StateEstimate;                                                % save estimate
    measurementHistory(:,k-1)  = z;                                                           % save measurment
    
    
    %% Plot GIF frames
    hold on
    scatter3(stateHistory(1,k),stateHistory(2,k),1,'fill','r')% update process
    scatter3(stateEstimateHistory(1,k),stateEstimateHistory(2,k),1,'fill','k')% update process
    if k>1
        plot3([stateHistory(1,k-1) stateHistory(1,k)],[stateHistory(2,k-1) stateHistory(2,k)],ones(2,1),'r--')
        plot3([stateEstimateHistory(1,k-1) stateEstimateHistory(1,k)],[stateEstimateHistory(2,k-1) stateEstimateHistory(2,k)],ones(2,1),'k')
        
    end
    probabilityMapVector=mvnpdf([X(:) Y(:)],StateEstimate(1:2)',(P(1:2,1:2)+P(1:2,1:2)')/2);
    probabilityMap=reshape(probabilityMapVector/norm(probabilityMapVector),size(X,1),[]);
    if k>2
        delete(h)
    end
    h=surf(X,Y,probabilityMap-1);
    colormap jet
    axis([-0.1 0.8 -0.1 0.6 -1 1.2])
    shading interp
    alpha(1)
    view(2);
    legend('Trajectory to Follow','Real State','State Estimate')
    
    %% Get GIF frames, first on requires speacial instructions
    if k==2
        rectangle('Position',[0,0,0.75,0.5],'EdgeColor','k',...
            'LineWidth',5)
        xlabel('X coordinate','FontSize',12,'FontWeight','bold')
        ylabel('Y coordinate','FontSize',12,'FontWeight','bold')
        title('State Estimate Distribution','FontSize',24,'FontWeight','bold')
        legend('Trajectory to Follow','Real State','State Estimate','FontSize',12,'FontWeight','bold')
        
        F = getframe;
        im = frame2im(F);
        [imind,cm] = rgb2ind(im,256);
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
        
    else
        F = getframe;
        im = frame2im(F);
        [imind,cm] = rgb2ind(im,256);
        imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append');
    end
    
end


%% Plot estimate convergence
plotTitles{1}='X Coordinate';
plotTitles{2}='Y Coordinate';
plotTitles{3}='Heading';
plotYLabel{1}='X Coordinate [m]';
plotYLabel{2}='Y Coordinate [m]';
plotYLabel{3}='Heading [rads]';

for k=1:3                                 % plot results
    subplot(3,1,k)
    plot(1:trajectoryLength, stateHistory(k,:), '-', 1:trajectoryLength, stateEstimateHistory(k,:), 'k--')
    xlabel('Steps')
    legend('Real State','State Estimate','Location','Best')
    title(plotTitles{k})
    ylabel(plotYLabel{k})
end
suptitle('UKF State Estimation')
