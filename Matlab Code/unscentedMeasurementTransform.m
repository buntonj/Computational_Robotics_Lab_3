function [transformMeasurementsMean,transformedMeasurementSigmaPoints,sensingCovariance,transformedMeasurementDeviations]=unscentedMeasurementTransform(transformedsigmaPoints,meanWeights,covarianceWeights,sensingDimension,R,roomBounds)

numberOfPoints=size(transformedsigmaPoints,2);
transformMeasurementsMean=zeros(sensingDimension,1);
transformedMeasurementSigmaPoints=zeros(sensingDimension,numberOfPoints);
for k=1:numberOfPoints
    [rangeForward,rangeRight] = ComputeLaser(roomBounds,transformedsigmaPoints(:,k));
    [magnetometerHeading,gyroMeasurement] = ComputeGyroscope(transformedsigmaPoints(:,k));
    transformedMeasurementSigmaPoints(:,k)=[magnetometerHeading,gyroMeasurement,rangeForward,rangeRight];
    transformMeasurementsMean=transformMeasurementsMean+meanWeights(k)*transformedMeasurementSigmaPoints(:,k);
end
transformedMeasurementDeviations=transformedMeasurementSigmaPoints-transformMeasurementsMean(:,ones(1,numberOfPoints));
sensingCovariance=transformedMeasurementDeviations*diag(covarianceWeights)*transformedMeasurementDeviations'+R;

