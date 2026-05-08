function [A, B, C, D, Ad, Bd, Cd, Dd, meta] = lateral_abcd(vx, dt, outputSelection, params)
%LATERAL_ABCD Build the lateral-only state-space model used by LinearMPC.
%
%   [A,B,C,D,Ad,Bd,Cd,Dd,meta] = lateral_abcd(vx, dt)
%
%   Continuous-time model:
%       xdot = A*x + B*u
%       yout = C*x + D*u
%
%   Discrete-time model:
%       x(k+1) = Ad*x(k) + Bd*u(k)
%       y(k)   = Cd*x(k) + Dd*u(k)
%
%   Lateral state:
%       x = [y_rig; yaw; vy_cg; yaw_rate]
%
%   Input:
%       u = steering_cmd
%
%   Simplification:
%       steering_cmd is applied directly as the front steering angle. The
%       original Python model's first-order steering actuator state is removed.
%
%   The default outputSelection is "tracking":
%       yout = [y_rig; yaw]
%
%   Use outputSelection = "full" to output all lateral states.
%
%   This function mirrors:
%       alpasim_controller/mpc_impl/linear_mpc.py::_linearize_dynamics
%
%   It assumes a straight-line operating point:
%       yaw = 0, vy_cg = 0, yaw_rate = 0

if nargin < 1 || isempty(vx)
    vx = 10.0;
end

if nargin < 2 || isempty(dt)
    dt = 0.1;
end

if nargin < 3 || isempty(outputSelection)
    outputSelection = "tracking";
end

if nargin < 4 || isempty(params)
    params = defaultVehicleParams();
else
    params = fillDefaultParams(params);
end

if vx <= 0
    error("lateral_abcd:InvalidVelocity", "vx must be positive.");
end

nx = 4;
nu = 1;

IY = 1;
IYAW = 2;
IVY = 3;
IYAW_RATE = 4;

A = zeros(nx, nx);
B = zeros(nx, nu);

lr = params.l_rig_to_cg;
L = params.wheelbase;

if vx < params.kinematic_threshold_speed
    modelType = "kinematic";

    gain = 10.0;

    A(IY, IYAW) = vx;
    A(IYAW, IYAW_RATE) = 1.0;

    A(IVY, IVY) = -gain;
    B(IVY, 1) = gain * vx * lr / L;

    A(IYAW_RATE, IYAW_RATE) = -gain;
    B(IYAW_RATE, 1) = gain * vx / L;
else
    modelType = "dynamic";

    m = params.mass;
    Iz = params.inertia;
    lf = L - lr;
    Caf = params.front_cornering_stiffness;
    Car = params.rear_cornering_stiffness;

    lf_caf = lf * Caf;
    lr_car = lr * Car;

    a00 = -2.0 * (Caf + Car) / (m * vx);
    a01 = -vx - 2.0 * (lf_caf - lr_car) / (m * vx);
    a10 = -2.0 * (lf_caf - lr_car) / (Iz * vx);
    a11 = -2.0 * (lf * lf_caf + lr * lr_car) / (Iz * vx);

    b00 = 2.0 * Caf / m;
    b10 = 2.0 * lf_caf / Iz;

    A(IY, IYAW) = vx;
    A(IY, IVY) = 1.0;
    A(IY, IYAW_RATE) = -lr;

    A(IYAW, IYAW_RATE) = 1.0;

    A(IVY, IVY) = a00;
    A(IVY, IYAW_RATE) = a01;
    B(IVY, 1) = b00;

    A(IYAW_RATE, IVY) = a10;
    A(IYAW_RATE, IYAW_RATE) = a11;
    B(IYAW_RATE, 1) = b10;
end

switch string(outputSelection)
    case "tracking"
        C = [1, 0, 0, 0;
             0, 1, 0, 0];
    case "full"
        C = eye(nx);
    otherwise
        error("lateral_abcd:InvalidOutputSelection", ...
            'outputSelection must be "tracking" or "full".');
end

D = zeros(size(C, 1), nu);

M = zeros(nx + nu, nx + nu);
M(1:nx, 1:nx) = A * dt;
M(1:nx, nx + (1:nu)) = B * dt;
expM = expm(M);

Ad = expM(1:nx, 1:nx);
Bd = expM(1:nx, nx + (1:nu));
Cd = C;
Dd = D;

meta = struct();
meta.model_type = modelType;
meta.state_names = ["y_rig"; "yaw"; "vy_cg"; "yaw_rate"];
meta.input_names = "steering_cmd";
meta.output_selection = string(outputSelection);
meta.vx = vx;
meta.dt = dt;
meta.parameters = params;
end

function params = defaultVehicleParams()
params = struct();
params.mass = 2014.4;
params.inertia = 3414.2;
params.l_rig_to_cg = 1.59;
params.wheelbase = 2.85;
params.front_cornering_stiffness = 93534.5;
params.rear_cornering_stiffness = 176162.1;
params.kinematic_threshold_speed = 5.0;
end

function params = fillDefaultParams(params)
defaults = defaultVehicleParams();
fields = fieldnames(defaults);
for idx = 1:numel(fields)
    field = fields{idx};
    if ~isfield(params, field) || isempty(params.(field))
        params.(field) = defaults.(field);
    end
end
end
