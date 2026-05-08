function sim = plotTrackingSimulation(CL,vx,sideWind,caseName,refYProfile)
%PLOTTRACKINGSIMULATION Simulate and plot lateral/yaw tracking performance.
%
%   sim = plotTrackingSimulation(CL,vx)
%   sim = plotTrackingSimulation(CL,vx,sideWind)
%   sim = plotTrackingSimulation(CL,vx,sideWind,caseName)
%   sim = plotTrackingSimulation(CL,vx,sideWind,caseName,refYProfile)
%
%   sideWind can be a scalar, a vector the same length as the simulation
%   time vector, or a function handle of time.
%   refYProfile can be a scalar, vector, or function handle. If omitted,
%   the default S-shaped reference trajectory is used.

if nargin < 3 || isempty(sideWind)
    sideWind = 0.0;
end

if nargin < 4 || isempty(caseName)
    caseName = 'Tracking Simulation';
end

t_sim = 0:0.01:100;

if nargin < 5 || isempty(refYProfile)
    ref_y = makeDefaultReference(t_sim);
else
    ref_y = makeProfileSignal(refYProfile,t_sim,'refYProfile');
end

ref_y_dot = gradient(ref_y,t_sim);
ref_phi = atan2(ref_y_dot,vx*ones(size(t_sim)));
side_wind_sim = makeProfileSignal(sideWind,t_sim,'sideWind');
fprintf('%s:\n', caseName);
nonzero_wind_idx = find(abs(side_wind_sim) > 1e-12,1);
if isempty(nonzero_wind_idx)
    first_nonzero_wind_time = NaN;
else
    first_nonzero_wind_time = t_sim(nonzero_wind_idx);
end
fprintf('  side_wind_speed range = [%.4g, %.4g] m/s\n', ...
    min(side_wind_sim), max(side_wind_sim));
fprintf('  first nonzero side_wind_speed time = %.4g s\n', ...
    first_nonzero_wind_time);
x_ref = cumtrapz(t_sim,vx*cos(ref_phi));
y_ref = ref_y;

u_sim = zeros(numel(t_sim),numel(CL.InputName));
u_sim = setInputSignal(u_sim,CL.InputName,'r_y',ref_y);
u_sim = setInputSignal(u_sim,CL.InputName,'r_yaw',ref_phi);
u_sim = setInputSignal(u_sim,CL.InputName,'side_wind_speed',side_wind_sim);

y_sim = lsim(CL,u_sim,t_sim);

y_actual = getOutputSignal(y_sim,CL.OutputName,'y_rig');
phi_actual = getOutputSignal(y_sim,CL.OutputName,'yaw');
e_y_sim = getOutputSignal(y_sim,CL.OutputName,'e_y');
e_phi_sim = getOutputSignal(y_sim,CL.OutputName,'e_yaw');
x_actual = cumtrapz(t_sim,vx*cos(phi_actual.'));

figure('Name',caseName);

subplot(2,2,1);
plot(t_sim,y_actual,'b','LineWidth',2);
hold on;
plot(t_sim,ref_y,'r:','LineWidth',2);
grid on;
xlabel('Time [Sec]');
ylabel('y [m]');
legend('actual lateral position','desired lateral position');
title(['Lateral Position Tracking: ', caseName]);

subplot(2,2,2);
plot(t_sim,phi_actual,'b','LineWidth',2);
hold on;
plot(t_sim,ref_phi,'r:','LineWidth',2);
grid on;
xlabel('Time [Sec]');
ylabel('\phi [rad]');
legend('actual heading angle','desired heading angle');
title(['Heading Angle Tracking: ', caseName]);

subplot(2,2,3);
plot(t_sim,e_y_sim,'b','LineWidth',2);
grid on;
xlabel('Time [Sec]');
ylabel('Tracking Error [m]');
title(['Lateral Tracking Error: ', caseName]);

subplot(2,2,4);
plot(t_sim,e_phi_sim,'b','LineWidth',2);
grid on;
xlabel('Time [Sec]');
ylabel('Tracking Error [rad]');
title(['Heading Angle Tracking Error: ', caseName]);

figure('Name',['x-y ', caseName]);
plot(x_actual,y_actual,'b','LineWidth',2);
hold on;
plot(x_ref,y_ref,'r:','LineWidth',2);
grid on;
axis tight;
ylim(1.2*[min([y_actual(:); y_ref(:)]), max([y_actual(:); y_ref(:)])]);
xlabel('x [m]');
ylabel('y [m]');
legend('actual vehicle trajectory','reference trajectory');
title(['Vehicle Position Tracking in x-y Plane: ', caseName]);

sim = struct();
sim.t = t_sim;
sim.ref_y = ref_y;
sim.ref_phi = ref_phi;
sim.side_wind = side_wind_sim;
sim.x_ref = x_ref;
sim.y_ref = y_ref;
sim.x_actual = x_actual;
sim.y_actual = y_actual;
sim.phi_actual = phi_actual;
sim.e_y = e_y_sim;
sim.e_phi = e_phi_sim;
end

function ref_y = makeDefaultReference(t_sim)
ref_y = zeros(size(t_sim));

idx1 = t_sim >= 3.0 & t_sim <= 10.0;
tau1 = (t_sim(idx1) - 3.0) / (10.0 - 3.0);
ref_y(idx1) = 2 * 0.5 * (1.0 - cos(pi*tau1));

idx2 = t_sim > 10.0 & t_sim <= 17.0;
tau2 = (t_sim(idx2) - 10.0) / (17.0 - 10.0);
ref_y(idx2) = 2 * 0.5 * (1.0 + cos(pi*tau2));

idx3 = t_sim >= 20.0 & t_sim <= 27.0;
tau3 = (t_sim(idx3) - 20.0) / (27.0 - 20.0);
ref_y(idx3) = -2 * 0.5 * (1.0 - cos(pi*tau3));

idx4 = t_sim > 27.0 & t_sim <= 34.0;
ref_y(idx4) = -2;

idx5 = t_sim > 34.0 & t_sim <= 41.0;
tau5 = (t_sim(idx5) - 34.0) / (41.0 - 34.0);
ref_y(idx5) = -2 * 0.5 * (1.0 + cos(pi*tau5));
end

function signal = makeProfileSignal(profile,t_sim,profileName)
if isa(profile,'function_handle')
    signal = profile(t_sim);
elseif isscalar(profile)
    signal = profile*ones(size(t_sim));
elseif isvector(profile) && numel(profile) == numel(t_sim)
    signal = reshape(profile,size(t_sim));
else
    error('plotTrackingSimulation:InvalidProfile', ...
        '%s must be a scalar, function handle, or vector matching t_sim.', ...
        profileName);
end
end

function u_sim = setInputSignal(u_sim,inputNames,signalName,signal)
idx = find(strcmp(inputNames,signalName),1);
if ~isempty(idx)
    u_sim(:,idx) = signal(:);
end
end

function signal = getOutputSignal(y_sim,outputNames,signalName)
idx = find(strcmp(outputNames,signalName),1);
if isempty(idx)
    error('plotTrackingSimulation:MissingOutput', ...
        'Closed-loop model does not have output "%s".', signalName);
end
signal = y_sim(:,idx);
end
