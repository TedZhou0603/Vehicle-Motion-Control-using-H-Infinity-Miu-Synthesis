clc;
clear all
% Lateral-only ABCD model for the AlpaSim linear bicycle dynamics.
%
% Run this script to define continuous-time and discrete-time state-space
% matrices in the MATLAB workspace.
%
% Lateral state:
%   x_lat = [y_rig; yaw; vy_cg; yaw_rate]
%
% Inputs:
%   u_lat = [steering_cmd; side_wind_speed]
%
% The side wind speed is converted to a lateral aerodynamic force applied at
% the vehicle CG. Positive side_wind_speed produces force in the positive
% lateral direction. Because this is a linear state-space model, the
% quadratic drag law is represented by a local linearization:
%
%   F_wind = 0.5*rho*Cy*A_side*abs(v_wind)*v_wind
%   dF_wind ~= side_wind_force_per_speed*d(side_wind_speed)
%
% Since the force acts at the CG, there is no yaw-moment term in B(4,2).
%
% Simplification:
%   steering_cmd is applied directly as the front steering angle. The
%   original Python model's first-order steering actuator state is removed.
%
% Default output:
%   y_out = [y_rig; yaw]

vx = 10.0;       % Operating longitudinal speed [m/s]

mass = 2014.4;                       % Vehicle mass [kg]
inertia = 3414.2;                    % Yaw inertia [kg*m^2]
l_rig_to_cg = 1.59;                  % Rear axle/rig origin to CG [m]
wheelbase = 2.85;                    % Wheelbase [m]
front_cornering_stiffness = 93534.5; % Front cornering stiffness [N/rad]
rear_cornering_stiffness = 176162.1; % Rear cornering stiffness [N/rad]

air_density = 1.225;                 % Air density [kg/m^3]
side_drag_coefficient = 1.0;         % Lateral aerodynamic coefficient [-]
side_projected_area = 5.0;           % Vehicle side projected area [m^2]
side_wind_speed_op = 10.0;           % Wind speed linearization point [m/s]

% Normal operating input ranges for analysis.
steering_cmd_range = deg2rad([-30.0, 30.0]); % Steering angle range [rad]
side_wind_speed_range = [-20.0, 20.0];       % Signed side wind speed range [m/s]

lr = l_rig_to_cg;
lf = wheelbase - lr;
Caf = front_cornering_stiffness;
Car = rear_cornering_stiffness;

lf_caf = lf * Caf;
lr_car = lr * Car;

a00 = -2.0 * (Caf + Car) / (mass * vx);
a01 = -vx - 2.0 * (lf_caf - lr_car) / (mass * vx);
a10 = -2.0 * (lf_caf - lr_car) / (inertia * vx);
a11 = -2.0 * (lf * lf_caf + lr * lr_car) / (inertia * vx);

b00 = 2.0 * Caf / mass;
b10 = 2.0 * lf_caf / inertia;
side_wind_force_per_speed = air_density ...
    * side_drag_coefficient ...
    * side_projected_area ...
    * abs(side_wind_speed_op);

A = [ ...
    0.0,  vx,   1.0, -lr; ...
    0.0,  0.0,  0.0,  1.0; ...
    0.0,  0.0,  a00,  a01; ...
    0.0,  0.0,  a10,  a11 ...
];

B = [ ...
    0.0, 0.0; ...
    0.0, 0.0; ...
    b00, side_wind_force_per_speed / mass; ...
    b10, 0.0 ...
];

C = [ ...
    1.0, 0.0, 0.0, 0.0; ...
    0.0, 1.0, 0.0, 0.0; ...
    0.0, 0.0, 1.0, 0.0; ...
    0.0, 0.0, 0.0, 1.0 ...
];

D = zeros(size(C, 1), size(B, 2));

%% Use the continuous model to conduct the control design.
% Using the ABCD matrices.
s = tf('s');
lateral_sys = ss(A,B,C,D);
lateral_sys.StateName = {'y_rig(m)'; 'yaw(rad)'; 'vy_cg(m/s)'; 'yaw_rate(rad/s)';};
lateral_sys.InputName = {'steering_cmd','side_wind_speed'};
lateral_sys.OutputName = {'y_rig'; 'yaw'; 'vy_cg'; 'yaw_rate'};

Rpass = ss(eye(2));
Rpass.InputName  = {'r_y','r_yaw'};
Rpass.OutputName = {'rmeas_y','rmeas_yaw'};

disp('Normal input ranges:');
fprintf('  steering_cmd: [%6.3f, %6.3f] rad  =  [%5.1f, %5.1f] deg\n', ...
    steering_cmd_range(1), steering_cmd_range(2), ...
    rad2deg(steering_cmd_range(1)), rad2deg(steering_cmd_range(2)));
fprintf('  side_wind_speed: [%5.1f, %5.1f] m/s\n', ...
    side_wind_speed_range(1), side_wind_speed_range(2));
fprintf('  side_wind_speed_op: %5.1f m/s\n\n', side_wind_speed_op);

lateral_tf = tf(lateral_sys({'y_rig','yaw'},{'steering_cmd','side_wind_speed'}));

G_y_steer = lateral_sys('y_rig','steering_cmd');
G_yaw_steer = lateral_sys('yaw','steering_cmd');
G_y_wind = lateral_sys('y_rig','side_wind_speed');
G_yaw_wind = lateral_sys('yaw','side_wind_speed');

G_y_steer_tf = tf(G_y_steer);
G_yaw_steer_tf = tf(G_yaw_steer);
G_y_wind_tf = tf(G_y_wind);
G_yaw_wind_tf = tf(G_yaw_wind);

disp('Transfer functions from steering command and side wind speed to y_rig and yaw:');
lateral_tf

disp('Transfer function: y_rig / steering_cmd');
G_y_steer_tf

disp('Transfer function: yaw / steering_cmd');
G_yaw_steer_tf

disp('Transfer function: y_rig / side_wind_speed');
G_y_wind_tf

disp('Transfer function: yaw / side_wind_speed');
G_yaw_wind_tf

% Calculate the zeros and print them out.
disp('Zeros: y_rig / steering_cmd');
disp(tzero(G_y_steer));

disp('Zeros: yaw / steering_cmd');
disp(tzero(G_yaw_steer));

disp('Zeros: y_rig / side_wind_speed');
disp(tzero(G_y_wind));

disp('Zeros: yaw / side_wind_speed');
disp(tzero(G_yaw_wind));

% Calculate the poles for these transfer functions.
disp('Poles: full lateral system');
disp(pole(lateral_sys));

disp('Poles: y_rig / steering_cmd');
disp(pole(G_y_steer));

disp('Poles: yaw / steering_cmd');
disp(pole(G_yaw_steer));

disp('Poles: y_rig / side_wind_speed');
disp(pole(G_y_wind));

disp('Poles: yaw / side_wind_speed');
disp(pole(G_yaw_wind));

% Use bodemag function to plot the magnitude responses.
figure;
bodemag(G_y_steer, 'b', G_y_wind, 'r--');
grid on;
legend('y_rig / steering_cmd', 'y_rig / side_wind_speed');
title('Magnitude Response to y\_rig');

figure;
bodemag(G_yaw_steer, 'b', G_yaw_wind, 'r--');
grid on;
legend('yaw / steering_cmd', 'yaw / side_wind_speed');
title('Magnitude Response to yaw');
% 
% 
%% Controller design
%compute the error signal for y_cg and yaw;
Sub1 = sumblk('e_y = rmeas_y - y_rig_ns');
Sub2 = sumblk('e_yaw = rmeas_yaw - yaw_ns');
SumMeasY = sumblk('y_rig_ns = y_rig + n_y_scaled');
SumMeasYaw = sumblk('yaw_ns = yaw + n_yaw_scaled');

%assume localization for y_rig, yaw 0.05m, 0.01 radus the nosiy measurement
%error for IMU for lateral speed acceleration and yaw rate is 0.05 m/s^2,
%0.05 rad/sec, design the weight for the Wn for nosiesy
y_noise_std = 0.0;
yaw_noise_std = 0.0;
Wn = ss(diag([y_noise_std, yaw_noise_std]));
Wn.InputName = {'n_y'; 'n_yaw'};
Wn.OutputName = {'n_y_scaled'; 'n_yaw_scaled'};


% define the tracking error ey is less than 0.1m, and 0.02 radius
% for frequency less than 10 hz, larger than 10hz does not matter.
tracking_bandwidth_hz =0.5;
tracking_bandwidth_rad = 2.0*pi*tracking_bandwidth_hz;
max_y_error = 0.18;
max_yaw_error = 0.3;

Wa_y = (1.0 / max_y_error) ...
    * tracking_bandwidth_rad / (s + tracking_bandwidth_rad);
Wa_y.InputName = 'e_y';
Wa_y.OutputName = 'e_y1';

Wa_yaw = (1.0 / max_yaw_error) ...
    * tracking_bandwidth_rad / (s + tracking_bandwidth_rad);
Wa_yaw.InputName = 'e_yaw';
Wa_yaw.OutputName = 'e_yaw1';


% define the control input limits delta is [-0.1,0.1] raduis and due to the 
% speed 10m/s for the turning. Assume the 
W_cmd = tf(0.5,[0.05/pi 1]);
W_cmd.InputName = 'steering_cmd_in';
W_cmd.OutputName = 'steering_cmd';


W_wind = tf(4.9,[1 1]);
W_wind.InputName = 'side_wind_speed_in';
W_wind.OutputName = 'side_wind_speed';

%connect all the block before we use the hinfsyn design
nominal_inputs = { ...
    'r_y', ...
    'r_yaw', ...
    'n_y', ...
    'n_yaw', ...
    'side_wind_speed_in', ...
    'steering_cmd_in' ...
};

% nominal_outputs = { ...
%     'e_y1', ...
%     'e_yaw1', ...
%     'e_y', ...
%     'e_yaw' ...
% };

nominal_outputs = { ...
    'e_y1', ...
    'e_y', ...
};

nominal_op = connect( ...
    lateral_sys, ...
    Rpass, ...
    Wn, ...
    SumMeasY, ...
    SumMeasYaw, ...
    Sub1, ...
    Sub2, ...
    Wa_y, ...
    Wa_yaw, ...
    W_cmd, ...
    W_wind, ...
    nominal_inputs, ...
    nominal_outputs);

disp('Generalized plant inputs:');
disp(nominal_inputs.');

disp('Generalized plant outputs:');
disp(nominal_outputs.');

ncont = 1; nmeas = 1;
controller_measurements = nominal_outputs(end-nmeas+1:end);
controller_commands = nominal_inputs(end-ncont+1:end);
performance_outputs = nominal_outputs(1:end-nmeas);
disturbance_inputs = nominal_inputs(1:end-ncont);

disp('hinfsyn performance outputs z:');
disp(performance_outputs.');

disp('hinfsyn measured outputs y to controller:');
disp(controller_measurements.');

disp('hinfsyn exogenous inputs w:');
disp(disturbance_inputs.');

disp('hinfsyn control inputs u from controller:');
disp(controller_commands.');

[K1,Scl1,gam1] = hinfsyn(nominal_op,nmeas,ncont);
K1.InputName = controller_measurements;
K1.OutputName = controller_commands;
Scl1.InputName = disturbance_inputs;
Scl1.OutputName = performance_outputs;
fprintf('H-infinity gamma = %.4f\n', gam1);

%% The close-loop performance for the Hinfity Controller with Nonminal Model
% K1 was synthesized from e_y to steering_cmd_in.  The original plant
% lateral_sys expects the physical steering_cmd input, so include W_cmd as
% part of the implemented controller, not as part of the plant.
K1_plant = W_cmd*K1;
K1_plant.InputName = {'e_y_feedback'};
K1_plant.OutputName = {'steering_cmd'};

% Build the nominal augmented lateral system from the original lateral_sys.
% Inputs are ordered as [external inputs; controller output].
% Outputs are ordered as [analysis outputs; controller measurement].
SumErrorY = sumblk('e_y = r_y - y_rig');
SumErrorYaw = sumblk('e_yaw = r_yaw - yaw');
SumErrorYFeedback = sumblk('e_y_feedback = r_y - y_rig');
nominal_augment_lateral_inputs = { ...
    'r_y', ...
    'r_yaw', ...
    'side_wind_speed', ...
    'steering_cmd' ...
};
nominal_augment_lateral_outputs = { ...
    'y_rig', ...
    'yaw', ...
    'vy_cg', ...
    'yaw_rate', ...
    'e_y', ...
    'e_yaw', ...
    'e_y_feedback' ...
};

nominal_augment_lateral_system = connect( ...
    lateral_sys, ...
    SumErrorY, ...
    SumErrorYaw, ...
    SumErrorYFeedback, ...
    nominal_augment_lateral_inputs, ...
    nominal_augment_lateral_outputs);

CL1 = lft(nominal_augment_lateral_system,K1_plant,1,1);
CL1.InputName = nominal_augment_lateral_inputs(1:end-1);
CL1.OutputName = nominal_augment_lateral_outputs(1:end-1);

%% The close-loop performance with measurement noise
% This model keeps the same true plant and controller, but feeds the
% controller through noisy measured lateral position.
SumMeasY_CL = sumblk('y_rig_ns = y_rig + n_y_scaled');
SumMeasYaw_CL = sumblk('yaw_ns = yaw + n_yaw_scaled');
SumErrorYNoise = sumblk('e_y = r_y - y_rig');
SumErrorYawNoise = sumblk('e_yaw = r_yaw - yaw');
SumErrorYFeedbackNoise = sumblk('e_y_feedback = r_y - y_rig_ns');

nominal_augment_lateral_noise_inputs = { ...
    'r_y', ...
    'r_yaw', ...
    'n_y', ...
    'n_yaw', ...
    'side_wind_speed', ...
    'steering_cmd' ...
};

nominal_augment_lateral_noise_outputs = { ...
    'y_rig', ...
    'yaw', ...
    'vy_cg', ...
    'yaw_rate', ...
    'e_y', ...
    'e_yaw', ...
    'y_rig_ns', ...
    'yaw_ns', ...
    'e_y_feedback' ...
};

nominal_augment_lateral_system_with_noise = connect( ...
    lateral_sys, ...
    Wn, ...
    SumMeasY_CL, ...
    SumMeasYaw_CL, ...
    SumErrorYNoise, ...
    SumErrorYawNoise, ...
    SumErrorYFeedbackNoise, ...
    nominal_augment_lateral_noise_inputs, ...
    nominal_augment_lateral_noise_outputs);

CL1_with_noise = lft(nominal_augment_lateral_system_with_noise,K1_plant,1,1);
CL1_with_noise.InputName = nominal_augment_lateral_noise_inputs(1:end-1);
CL1_with_noise.OutputName = nominal_augment_lateral_noise_outputs(1:end-1);

disp('Nominal closed-loop inputs without measurement noise:');
disp(CL1.InputName);

disp('Nominal closed-loop outputs without measurement noise:');
disp(CL1.OutputName);

disp('Nominal augmented lateral system with noise inputs [external; control]:');
disp(nominal_augment_lateral_noise_inputs.');

disp('Nominal augmented lateral system with noise outputs [analysis; controller measurement]:');
disp(nominal_augment_lateral_noise_outputs.');

disp('Nominal closed-loop inputs with measurement noise:');
disp(CL1_with_noise.InputName);

disp('Nominal closed-loop outputs with measurement noise:');
disp(CL1_with_noise.OutputName);

%% Bode plots for clean nominal closed-loop tracking errors
figure;

subplot(2,2,1);
bodemag(CL1('e_y','r_y'));
grid on;
title('e\_y / r\_y');

subplot(2,2,2);
bodemag(CL1('e_yaw','r_y'));
grid on;
title('e\_yaw / r\_y');

subplot(2,2,3);
bodemag(CL1('e_y','side_wind_speed'));
grid on;
title('e\_y / side\_wind\_speed');

subplot(2,2,4);
bodemag(CL1('e_yaw','side_wind_speed'));
grid on;
title('e\_yaw / side\_wind\_speed');

%% Bode plots for noisy nominal closed-loop tracking errors
figure;

subplot(2,2,1);
bodemag(CL1_with_noise('e_y','r_y'));
grid on;
title('Noisy: e\_y / r\_y');

subplot(2,2,2);
bodemag(CL1_with_noise('e_yaw','r_y'));
grid on;
title('Noisy: e\_yaw / r\_y');

subplot(2,2,3);
bodemag(CL1_with_noise('e_y','side_wind_speed'));
grid on;
title('Noisy: e\_y / side\_wind\_speed');

subplot(2,2,4);
bodemag(CL1_with_noise('e_yaw','side_wind_speed'));
grid on;
title('Noisy: e\_yaw / side\_wind\_speed');

%% Time-domain nominal trajectory tracking simulations for Hinfinity Controller
tracking_sim_without_side_wind = plotTrackingSimulation(CL1,vx,0.0, ...
    'No Side Wind');

% side_wind_profile = @(t) ...
%     8.0*(t >= 5.0 & t < 15.0) ...
%     - 8.0*(t >= 25.0 & t < 35.0);
side_wind_profile = @(t) ...
    8.0*(t >= 5.0);
tracking_sim_with_side_wind = plotTrackingSimulation(CL1,vx, ...
    side_wind_profile, ...
    'Side Wind Steps with Zero Reference', ...
    0.0);
%% Model uncertainty and robustness test
% Compute the parameters uncertainty for the tire and the mass
% The mass is variant with [-20%,20%]around the nominal value
% The Caf and Car is variant between [-10%,0%] around the nominal value
mass_uc = ureal('Mass',mass,'Percentage',[-5 15]);
Caf_uc = ureal('Caf',Caf,'Percentage',[-10 1e-6]);
Car_uc = ureal('Car',Car,'Percentage',[-10 1e-6]);

% compute the new system with uncertainty
lf_caf_uc = lf*Caf_uc;
lr_car_uc = lr*Car_uc;

a00_uc = -2.0*(Caf_uc + Car_uc)/(mass_uc*vx);
a01_uc = -vx - 2.0*(lf_caf_uc - lr_car_uc)/(mass_uc*vx);
a10_uc = -2.0*(lf_caf_uc - lr_car_uc)/(inertia*vx);
a11_uc = -2.0*(lf*lf_caf_uc + lr*lr_car_uc)/(inertia*vx);

b00_uc = 2.0*Caf_uc/mass_uc;
b10_uc = 2.0*lf_caf_uc/inertia;

A_uc = [ ...
    0.0,  vx,   1.0, -lr; ...
    0.0,  0.0,  0.0,  1.0; ...
    0.0,  0.0,  a00_uc,  a01_uc; ...
    0.0,  0.0,  a10_uc,  a11_uc ...
];

B_uc = [ ...
    0.0, 0.0; ...
    0.0, 0.0; ...
    b00_uc, side_wind_force_per_speed/mass_uc; ...
    b10_uc, 0.0 ...
];

lateral_sys_uc = ss(A_uc,B_uc,C,D);
lateral_sys_uc.StateName = lateral_sys.StateName;
lateral_sys_uc.InputName = lateral_sys.InputName;
lateral_sys_uc.OutputName = lateral_sys.OutputName;

% compute the new connected block for open loop system with uncertainty
uncertain_op = connect( ...
    lateral_sys_uc, ...
    Rpass, ...
    Wn, ...
    SumMeasY, ...
    SumMeasYaw, ...
    Sub1, ...
    Sub2, ...
    Wa_y, ...
    Wa_yaw, ...
    W_cmd, ...
    W_wind, ...
    nominal_inputs, ...
    nominal_outputs);

Scl1_uc = lft(uncertain_op,K1,nmeas,ncont);
Scl1_uc.InputName = disturbance_inputs;
Scl1_uc.OutputName = performance_outputs;

uncertain_augment_lateral_system = connect( ...
    lateral_sys_uc, ...
    SumErrorY, ...
    SumErrorYaw, ...
    SumErrorYFeedback, ...
    nominal_augment_lateral_inputs, ...
    nominal_augment_lateral_outputs);

CL1_uc = lft(uncertain_augment_lateral_system,K1_plant,1,1);
CL1_uc.InputName = nominal_augment_lateral_inputs(1:end-1);
CL1_uc.OutputName = nominal_augment_lateral_outputs(1:end-1);

robust_stability_margin = robstab(CL1_uc);
robust_performance_margin = robgain(Scl1_uc,1.0);

fprintf('Physical closed-loop robust stability margin bounds = [%.4f, %.4f]\n', ...
    robust_stability_margin.LowerBound, ...
    robust_stability_margin.UpperBound);
fprintf('Robust performance margin bounds for gamma = 1 = [%.4f, %.4f]\n', ...
    robust_performance_margin.LowerBound, ...
    robust_performance_margin.UpperBound);

% sampling the 40 numbers of the uncertainty for mass and stifiness
% perfromace in on single plot with the simulation code we have to see the
% perforamnce
n_uncertain_samples = 40;
CL1_uc_samples = usample(CL1_uc,n_uncertain_samples);

t_uc = tracking_sim_without_side_wind.t;
ref_y_uc = tracking_sim_without_side_wind.ref_y;
ref_phi_uc = tracking_sim_without_side_wind.ref_phi;

u_uc = zeros(numel(t_uc),numel(CL1_uc.InputName));
u_uc(:,strcmp(CL1_uc.InputName,'r_y')) = ref_y_uc(:);
u_uc(:,strcmp(CL1_uc.InputName,'r_yaw')) = ref_phi_uc(:);
u_uc(:,strcmp(CL1_uc.InputName,'side_wind_speed')) = 0.0;

y_rig_idx_uc = strcmp(CL1_uc.OutputName,'y_rig');
yaw_idx_uc = strcmp(CL1_uc.OutputName,'yaw');
e_y_idx_uc = strcmp(CL1_uc.OutputName,'e_y');
e_yaw_idx_uc = strcmp(CL1_uc.OutputName,'e_yaw');

uncertain_time_fig = figure('Name','Uncertain Plants with Nominal Controller, S Reference, No Side Wind');
sgtitle('Uncertain Plants with Nominal Controller, S Reference, No Side Wind');

uncertain_time_ax(1) = subplot(2,2,1);
hold on;
grid on;
title('Lateral Position');
xlabel('Time [Sec]');
ylabel('y [m]');

uncertain_time_ax(2) = subplot(2,2,2);
hold on;
grid on;
title('Heading Angle');
xlabel('Time [Sec]');
ylabel('\phi [rad]');

uncertain_time_ax(3) = subplot(2,2,3);
hold on;
grid on;
title('Lateral Tracking Error');
xlabel('Time [Sec]');
ylabel('e_y [m]');

uncertain_time_ax(4) = subplot(2,2,4);
hold on;
grid on;
title('Heading Tracking Error');
xlabel('Time [Sec]');
ylabel('e_\phi [rad]');

uncertain_xy_fig = figure('Name','x-y Uncertain Plants with Nominal Controller, S Reference, No Side Wind');
hold on;
grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Vehicle Position Tracking in x-y Plane: Uncertain Plants, No Side Wind');

for sample_idx = 1:n_uncertain_samples
    y_uc = lsim(CL1_uc_samples(:,:,sample_idx),u_uc,t_uc);
    x_uc = cumtrapz(t_uc(:),vx*cos(y_uc(:,yaw_idx_uc)));
    if sample_idx == 1
        sample_visibility = 'on';
    else
        sample_visibility = 'off';
    end

    plot(uncertain_time_ax(1),t_uc,y_uc(:,y_rig_idx_uc),'Color',[0.7 0.7 0.7], ...
        'DisplayName','uncertain samples','HandleVisibility',sample_visibility);

    plot(uncertain_time_ax(2),t_uc,y_uc(:,yaw_idx_uc),'Color',[0.7 0.7 0.7], ...
        'DisplayName','uncertain samples','HandleVisibility',sample_visibility);

    plot(uncertain_time_ax(3),t_uc,y_uc(:,e_y_idx_uc),'Color',[0.7 0.7 0.7], ...
        'DisplayName','uncertain samples','HandleVisibility',sample_visibility);

    plot(uncertain_time_ax(4),t_uc,y_uc(:,e_yaw_idx_uc),'Color',[0.7 0.7 0.7], ...
        'DisplayName','uncertain samples','HandleVisibility',sample_visibility);

    plot(x_uc,y_uc(:,y_rig_idx_uc),'Color',[0.7 0.7 0.7], ...
        'DisplayName','uncertain samples','HandleVisibility',sample_visibility);
end

plot(uncertain_time_ax(1),t_uc,tracking_sim_without_side_wind.y_actual,'b','LineWidth',2, ...
    'DisplayName','nominal');
plot(uncertain_time_ax(1),t_uc,ref_y_uc,'r:','LineWidth',2, ...
    'DisplayName','S reference');
legend;

plot(uncertain_time_ax(2),t_uc,tracking_sim_without_side_wind.phi_actual,'b','LineWidth',2, ...
    'DisplayName','nominal');
plot(uncertain_time_ax(2),t_uc,ref_phi_uc,'r:','LineWidth',2, ...
    'DisplayName','S reference');
legend;

plot(uncertain_time_ax(3),t_uc,tracking_sim_without_side_wind.e_y,'b','LineWidth',2, ...
    'DisplayName','nominal');
legend;

plot(uncertain_time_ax(4),t_uc,tracking_sim_without_side_wind.e_phi,'b','LineWidth',2, ...
    'DisplayName','nominal');
legend;

figure(uncertain_xy_fig);
plot(tracking_sim_without_side_wind.x_actual,tracking_sim_without_side_wind.y_actual,'b','LineWidth',2, ...
    'DisplayName','nominal');
plot(tracking_sim_without_side_wind.x_ref,tracking_sim_without_side_wind.y_ref,'r:','LineWidth',2, ...
    'DisplayName','S reference');
ylim([min(ref_y_uc)-1.0 max(ref_y_uc)+1.0]);
legend;

% Bode plots for sampled uncertain closed-loop transfer functions.
w_uc_bode = logspace(-2,2,500);
bode_output_names = {'y_rig','yaw','e_y','e_yaw'};
bode_input_names = {'r_y','r_yaw','r_y','r_yaw'};
bode_plot_titles = { ...
    'y\_rig / r\_y', ...
    'yaw / r\_yaw', ...
    'e\_y / r\_y', ...
    'e\_yaw / r\_yaw' ...
};

figure('Name','Bode: Sampled Uncertain CLTF with Nominal Controller');
tiledlayout(2,2);
for bode_idx = 1:numel(bode_output_names)
    nexttile;
    hold on;
    grid on;

    output_idx = strcmp(CL1_uc.OutputName,bode_output_names{bode_idx});
    input_idx = strcmp(CL1_uc.InputName,bode_input_names{bode_idx});

    for sample_idx = 1:n_uncertain_samples
        mag_sample = squeeze(bode(CL1_uc_samples(output_idx,input_idx,sample_idx),w_uc_bode));
        if sample_idx == 1
            sample_visibility = 'on';
        else
            sample_visibility = 'off';
        end
        semilogx(w_uc_bode,20*log10(mag_sample),'Color',[0.7 0.7 0.7], ...
            'DisplayName','uncertain samples','HandleVisibility',sample_visibility);
    end

    mag_nominal = squeeze(bode(CL1(bode_output_names{bode_idx},bode_input_names{bode_idx}),w_uc_bode));
    semilogx(w_uc_bode,20*log10(mag_nominal),'b','LineWidth',2, ...
        'DisplayName','nominal');
    xlabel('Frequency [rad/s]');
    ylabel('Magnitude [dB]');
    title(bode_plot_titles{bode_idx});
    legend;
end

%% Mu synthesis controller design
mu_opts = musynOptions('Display','short');
[K_mu,mu_perf,mu_info] = musyn(uncertain_op,nmeas,ncont,K1,mu_opts);
K_mu.InputName = controller_measurements;
K_mu.OutputName = controller_commands;

K_mu_plant = W_cmd*K_mu;
K_mu_plant.InputName = {'e_y_feedback'};
K_mu_plant.OutputName = {'steering_cmd'};

Scl_mu_uc = lft(uncertain_op,K_mu,nmeas,ncont);
Scl_mu_uc.InputName = disturbance_inputs;
Scl_mu_uc.OutputName = performance_outputs;

CL_mu_uc = lft(uncertain_augment_lateral_system,K_mu_plant,1,1);
CL_mu_uc.InputName = nominal_augment_lateral_inputs(1:end-1);
CL_mu_uc.OutputName = nominal_augment_lateral_outputs(1:end-1);

CL_mu = lft(nominal_augment_lateral_system,K_mu_plant,1,1);
CL_mu.InputName = nominal_augment_lateral_inputs(1:end-1);
CL_mu.OutputName = nominal_augment_lateral_outputs(1:end-1);

fprintf('Mu synthesis robust performance = %.4f\n', mu_perf);
fprintf('Mu nominal physical closed-loop stable = %d\n', isstable(CL_mu));

% Sample uncertain plants with the mu-synthesis controller for the same
% S-shaped reference and zero side wind as the nominal no-wind simulation.
n_mu_uncertain_samples = 200;
CL_mu_uc_samples = usample(CL_mu_uc,n_mu_uncertain_samples);

u_mu_uc = zeros(numel(t_uc),numel(CL_mu_uc.InputName));
u_mu_uc(:,strcmp(CL_mu_uc.InputName,'r_y')) = ref_y_uc(:);
u_mu_uc(:,strcmp(CL_mu_uc.InputName,'r_yaw')) = ref_phi_uc(:);
u_mu_uc(:,strcmp(CL_mu_uc.InputName,'side_wind_speed')) = 0.0;

y_rig_idx_mu_uc = strcmp(CL_mu_uc.OutputName,'y_rig');
e_y_idx_mu_uc = strcmp(CL_mu_uc.OutputName,'e_y');

y_mu_nominal = lsim(CL_mu,u_mu_uc,t_uc);

figure('Name','Uncertain Plants with Mu Controller, S Reference, No Side Wind');
sgtitle('Uncertain Plants with \mu Controller, S Reference, No Side Wind');

mu_uncertain_ax(1) = subplot(2,1,1);
hold on;
grid on;
title('Lateral Position Tracking');
xlabel('Time [Sec]');
ylabel('y [m]');

mu_uncertain_ax(2) = subplot(2,1,2);
hold on;
grid on;
title('Lateral Tracking Error');
xlabel('Time [Sec]');
ylabel('e_y [m]');

for sample_idx = 1:n_mu_uncertain_samples
    y_mu_uc = lsim(CL_mu_uc_samples(:,:,sample_idx),u_mu_uc,t_uc);
    if sample_idx == 1
        sample_visibility = 'on';
    else
        sample_visibility = 'off';
    end

    plot(mu_uncertain_ax(1),t_uc,y_mu_uc(:,y_rig_idx_mu_uc),'Color',[0.0 0.55 0.0], ...
        'LineWidth',0.8,'DisplayName','uncertain samples','HandleVisibility',sample_visibility);
    plot(mu_uncertain_ax(2),t_uc,y_mu_uc(:,e_y_idx_mu_uc),'Color',[0.0 0.55 0.0], ...
        'LineWidth',0.8,'DisplayName','uncertain samples','HandleVisibility',sample_visibility);
end

plot(mu_uncertain_ax(1),t_uc,y_mu_nominal(:,y_rig_idx_mu_uc),'b','LineWidth',2, ...
    'DisplayName','nominal \mu');
plot(mu_uncertain_ax(1),t_uc,ref_y_uc,'r:','LineWidth',2, ...
    'DisplayName','S reference');
legend(mu_uncertain_ax(1),'show');

plot(mu_uncertain_ax(2),t_uc,y_mu_nominal(:,e_y_idx_mu_uc),'b','LineWidth',2, ...
    'DisplayName','nominal \mu');
legend(mu_uncertain_ax(2),'show');
