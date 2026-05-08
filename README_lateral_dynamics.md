# Simplified Lateral Dynamics ABCD Derivation

This note derives the simplified lateral-only state-space model implemented in
`controller.m` and `lateral_abcd.m`.

The original Python vehicle model includes a first-order steering actuator:

```text
steering_dot = (steering_cmd - steering)/tau_s
```

The simplified MATLAB model removes that actuator state and assumes the
commanded steering angle is applied directly to the front wheels:

```text
delta = steering_cmd
```

## State, Input, and Output

The full Python vehicle state is:

```text
x_full = [x; y; yaw; vx_cg; vy_cg; yaw_rate; steering; accel]
```

For simplified lateral control, the longitudinal states and steering actuator
state are removed. The longitudinal speed `vx_cg` is treated as a constant
operating condition.

The lateral state is:

```text
x_lat = [y_rig; yaw; vy_cg; yaw_rate]
```

where:

```text
y_rig    = lateral position of the rig/rear-axle origin [m]
yaw      = vehicle heading angle [rad]
vy_cg    = lateral velocity at the center of gravity [m/s]
yaw_rate = yaw angular velocity [rad/s]
```

The control input is:

```text
u_lat = steering_cmd = delta
```

where `delta` is the front steering angle in radians.

The default output is:

```text
y_out = [y_rig; yaw]
```

## Parameters

The MATLAB files use the same vehicle parameters as the Python model:

```text
m   = 2014.4 kg
Iz  = 3414.2 kg*m^2
lr  = 1.59 m
L   = 2.85 m
lf  = L - lr
Caf = 93534.5 N/rad
Car = 176162.1 N/rad
vx  = 10.0 m/s
dt  = 0.1 s
```

Definitions:

```text
m   = vehicle mass
Iz  = yaw moment of inertia
lr  = distance from rig/rear-axle origin to CG
lf  = distance from CG to front axle
L   = wheelbase
Caf = front cornering stiffness
Car = rear cornering stiffness
vx  = constant longitudinal speed at the CG
```

## Dynamic Bicycle Model

For the dynamic bicycle branch, the lateral velocity and yaw-rate dynamics are:

```text
vy_dot = a00*vy_cg + a01*yaw_rate + b00*delta
r_dot  = a10*vy_cg + a11*yaw_rate + b10*delta
```

where:

```text
r = yaw_rate
delta = steering_cmd
```

The coefficients are:

```text
a00 = -2*(Caf + Car)/(m*vx)
a01 = -vx - 2*(lf*Caf - lr*Car)/(m*vx)
a10 = -2*(lf*Caf - lr*Car)/(Iz*vx)
a11 = -2*(lf^2*Caf + lr^2*Car)/(Iz*vx)

b00 =  2*Caf/m
b10 =  2*lf*Caf/Iz
```

## Lateral Position Kinematics

The model state uses `vy_cg`, but `y_rig` is measured at the rig/rear-axle
origin. The lateral velocity of the rig origin is:

```text
vy_rig = vy_cg - lr*yaw_rate
```

The planar kinematics are:

```text
y_dot = vx*sin(yaw) + vy_rig*cos(yaw)
```

For a straight-line operating point:

```text
yaw = 0
vy_cg = 0
yaw_rate = 0
```

Using:

```text
sin(yaw) ~= yaw
cos(yaw) ~= 1
```

the lateral position equation becomes:

```text
y_dot = vx*yaw + vy_cg - lr*yaw_rate
```

The yaw kinematics are:

```text
yaw_dot = yaw_rate
```

## Continuous-Time State-Space Model

Collecting the simplified lateral equations:

```text
y_dot        = vx*yaw + vy_cg - lr*yaw_rate
yaw_dot      = yaw_rate
vy_dot       = a00*vy_cg + a01*yaw_rate + b00*steering_cmd
yaw_rate_dot = a10*vy_cg + a11*yaw_rate + b10*steering_cmd
```

With:

```text
x_lat = [y_rig; yaw; vy_cg; yaw_rate]
u_lat = steering_cmd
```

the continuous-time model is:

```text
x_dot = A*x_lat + B*u_lat
y_out = C*x_lat + D*u_lat
```

where:

```text
A = [
    0, vx,  1,  -lr;
    0,  0,  0,    1;
    0,  0, a00, a01;
    0,  0, a10, a11
]

B = [
      0;
      0;
    b00;
    b10
]
```

The default tracking output is:

```text
C = [
    1, 0, 0, 0;
    0, 1, 0, 0
]

D = [
    0;
    0
]
```

## Numerical Continuous-Time Matrices at vx = 10 m/s

Using the default parameters:

```text
A =
[
    0, 10.0000,   1.0000,  -1.5900;
    0,  0,        0,        1.0000;
    0,  0,      -26.7769,   6.1084;
    0,  0,        9.5041, -34.7871
]

B =
[
     0;
     0;
    92.8659;
    69.0372
]

C =
[
    1, 0, 0, 0;
    0, 1, 0, 0
]

D =
[
    0;
    0
]
```

## Discrete-Time Model

The MATLAB files also compute exact zero-order-hold discretization:

```text
x_lat(k+1) = Ad*x_lat(k) + Bd*u_lat(k)
y_out(k)   = Cd*x_lat(k) + Dd*u_lat(k)
```

The augmented matrix is:

```text
M = [
    A, B;
    0, 0
]
```

Then:

```text
exp(M*dt) = [
    Ad, Bd;
     0,  1
]
```

and:

```text
Cd = C
Dd = D
```

## Low-Speed Kinematic Branch

The reusable helper `lateral_abcd.m` keeps the Python model's low-speed branch:

```text
vx < 5.0 m/s
```

For the simplified four-state model, that branch is:

```text
y_dot        = vx*yaw
yaw_dot      = yaw_rate
vy_dot       = -gain*vy_cg + gain*(vx*lr/L)*steering_cmd
yaw_rate_dot = -gain*yaw_rate + gain*(vx/L)*steering_cmd
```

with:

```text
gain = 10
```
