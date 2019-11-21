function [transformedMean,transformedsigmaPoints,processCovariance,transformedDeviations]=unscentedProcessTransform(sigmaPoints,meanWeights,covarianceWeights,stateDimension,Q,goal,integrationTime)
numberOfPoints=size(sigmaPoints,2);
transformedMean=zeros(stateDimension,1);
transformedsigmaPoints=zeros(stateDimension,numberOfPoints);
for k=1:numberOfPoints
    [newState,trajectory,u] = RobotDynamicsStep(sigmaPoints(:,k),goal,integrationTime);
    transformedsigmaPoints(:,k)=newState;
    transformedMean=transformedMean+meanWeights(k)*transformedsigmaPoints(:,k);
end
transformedDeviations=transformedsigmaPoints-transformedMean(:,ones(1,numberOfPoints));
processCovariance=transformedDeviations*diag(covarianceWeights)*transformedDeviations'+Q;