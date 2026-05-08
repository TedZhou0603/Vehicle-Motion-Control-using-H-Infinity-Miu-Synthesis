function sim = plotTrackingSimulation_without_side_wind(CL,vx,caseName)
%PLOTTRACKINGSIMULATION_WITHOUT_SIDE_WIND Plot tracking with zero side wind.

if nargin < 3 || isempty(caseName)
    caseName = 'No Side Wind';
end

sim = plotTrackingSimulation(CL,vx,0.0,caseName);
end
