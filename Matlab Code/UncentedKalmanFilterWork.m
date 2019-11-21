function [stateEstimate,P]=UncentedKalmanFilterWork(stateEstimate,P,z,Q,R,goal,integrationTime,roomBounds)
stateDimension=length(stateEstimate);                                           %number of states
sensingDimension=length(z);                                                     %number of measurements
alpha=5e-4;                                                                     %tunable parameter
ki=0;                                                                           %tunable parameter
beta=2;                                                                         %tunable parameter
lambda=alpha^2*(stateDimension+ki)-stateDimension;                              %scaling factor
s=sqrt(stateDimension+lambda);                                                  %scaling factor
meanWeights=[lambda/s^2 0.5/s^2+zeros(1,2*stateDimension)];                     %weights for means
% Wm0=0.8;
% Wm=ones(2*stateDimension+1,1)*((1-Wm0)/(2*stateDimension));
covarianceWeights=meanWeights;
covarianceWeights(1)=covarianceWeights(1)+(1-alpha^2+beta);                     %weights for covariance
sigmaPoints=ComputeSigmaPoints(stateEstimate,P,s);                              %sigma points around state estimate
[transformedMean,transformedsigmaPoints,processCovariance,transformedDeviations]=unscentedProcessTransform(sigmaPoints,meanWeights,covarianceWeights,stateDimension,Q,goal,integrationTime);    %unscented transformation of process
[transformMeasurementsMean,transformedMeasurementSigmaPoints,sensingCovariance,transformedMeasurementDeviations]=unscentedMeasurementTransform(transformedsigmaPoints,meanWeights,covarianceWeights,sensingDimension,R,roomBounds);                                    %unscented transformation of measurements
transformedCrossCovariance=transformedDeviations*diag(covarianceWeights)*transformedMeasurementDeviations';                                                            
K=transformedCrossCovariance/sensingCovariance;
stateEstimate=transformedMean+K*(z-transformMeasurementsMean);                                                                  %state update
P=processCovariance-K*transformedCrossCovariance';                                          %covariance update
% make sure the covariance is positive semidefinite
P=(P+P')/2;
d = eig(P);
tol = length(d)*100*eps(max(d));
isposdef = all(d > tol);
if ~isposdef     P = nearestSPD(P);
end