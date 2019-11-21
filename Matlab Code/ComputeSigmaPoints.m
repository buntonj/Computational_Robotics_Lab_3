function sigmaPoints=ComputeSigmaPoints(stateEstimate,P,s)

P=(P+P')/2;
covarianceEigenvalues = eig(P);
tol = length(covarianceEigenvalues)*100*eps(max(covarianceEigenvalues));
isposdef = all(covarianceEigenvalues > tol);
if ~isposdef
    P = nearestSPD(P);
end
A = s*chol(P)';
Y = stateEstimate(:,ones(1,numel(stateEstimate)));
sigmaPoints = [stateEstimate Y+A Y-A];