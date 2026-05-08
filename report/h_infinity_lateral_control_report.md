# H-Infinity Lateral Control Design for the AlpaSim Vehicle Model

## 1. Project Overview

This project develops a lateral steering controller for the AlpaSim vehicle model using a linear bicycle model and an H-infinity synthesis workflow in MATLAB. The implementation is contained mainly in `matlab.m`, with the time-domain plotting and trajectory simulation moved into `plotTrackingSimulation.m`.

The work follows the same general control motivation as Rafaila and Livint's paper, "H-infinity control of automatic vehicle steering": autonomous steering is treated as a lateral control problem at constant longitudinal speed, and the controller is designed by shaping closed-loop tracking performance and disturbance rejection. The contribution of this project is the full MATLAB block interconnection for the current AlpaSim-style model, including side-wind-speed disturbance modeling, named-signal generalized plant construction, LFT-based closed-loop analysis models, and reusable trajectory simulation plots.

## 2. Lateral Dynamics Derivation

### 2.1 Assumptions

The model used in `matlab.m` is a lateral-only linear bicycle model. The main assumptions are:

- The longitudinal speed is constant at `vx = 10 m/s`.
- Vehicle motion is evaluated near small yaw angle and small tire slip angle.
- Tire lateral forces are linear with respect to slip angle.
- The steering command is applied directly as the front steering angle. The first-order steering actuator from the original model is removed from the physical plant.
- Longitudinal dynamics, roll dynamics, tire saturation, and hard steering saturation are not included in the state-space plant.
- Side wind is modeled as a lateral force acting at the center of gravity, so it produces lateral acceleration but no direct yaw moment.
- The aerodynamic side force is locally linearized around `side_wind_speed_op = 10 m/s`.

The state vector is

```text
x_lat = [ y_rig, yaw, vy_cg, yaw_rate ]'
```

where `y_rig` is the lateral position at the rear/rig reference point, `yaw` is the heading angle, `vy_cg` is lateral velocity at the center of gravity, and `yaw_rate` is yaw rate.

The physical plant inputs are

```text
u_lat = [ steering_cmd, side_wind_speed ]'
```

### 2.2 Linear Tire Forces

Using the bicycle model, the front and rear lateral tire forces are approximated by

```text
F_yf = 2*Caf*( steering_cmd - (vy_cg + lf*yaw_rate)/vx )
F_yr = 2*Car*( -(vy_cg - lr*yaw_rate)/vx )
```

where `Caf` and `Car` are the front and rear cornering stiffnesses, `lf` is the CG-to-front-axle distance, and `lr` is the rear/rig-origin-to-CG distance.

The lateral and yaw equations are

```text
mass*(d(vy_cg)/dt + vx*yaw_rate) = F_yf + F_yr + F_wind
inertia*d(yaw_rate)/dt = lf*F_yf - lr*F_yr
```

The wind yaw moment term is zero because the wind force is assumed to act at the CG.

### 2.3 Side-Wind Speed to Force

The physical aerodynamic side force is nonlinear:

```text
F_wind = 0.5*rho*Cy*A_side*abs(v_wind)*v_wind
```

For the linear state-space model, this is locally linearized as

```text
dF_wind ~= rho*Cy*A_side*abs(v_wind_op)*d(side_wind_speed)
```

In the code, this gain is

```text
side_wind_force_per_speed =
    air_density * side_drag_coefficient * side_projected_area
    * abs(side_wind_speed_op)
```

Then the wind input enters the `vy_cg` acceleration equation as

```text
B(3,2) = side_wind_force_per_speed / mass
```

and the yaw-rate equation has

```text
B(4,2) = 0
```

because the disturbance is applied at the CG.

### 2.4 Kinematic Equations

The lateral position is defined at the rear/rig reference point instead of the CG. With small-angle approximation,

```text
d(y_rig)/dt = vx*yaw + vy_cg - lr*yaw_rate
d(yaw)/dt = yaw_rate
```

This gives the first two rows of the state matrix:

```text
[ 0, vx, 1, -lr ]
[ 0,  0, 0,   1 ]
```

### 2.5 State-Space Model

The current state-space model is

```text
dx/dt = A*x + B*u
y     = C*x + D*u
```

with `C = eye(4)`, so the model outputs all four states:

```text
y_out = [ y_rig, yaw, vy_cg, yaw_rate ]'
```

The two integrators in the open-loop poles come from lateral position and yaw angle. This is why transfer functions such as `y_rig / side_wind_speed` show two poles at zero: lateral position is an accumulated effect of lateral velocity and heading, and heading is also an accumulated effect of yaw rate.

## 3. Control Problem Formulation

The control objective is to command steering so that the vehicle tracks lateral position and heading references while rejecting side-wind disturbances.

The current H-infinity synthesis is lateral-error-only:

```text
ncont = 1
nmeas = 1
```

The generalized plant inputs are

```text
[ r_y, r_yaw, n_y, n_yaw, side_wind_speed_in, steering_cmd_in ]
```

The generalized plant outputs currently used by `hinfsyn` are

```text
[ e_y1, e_y ]
```

Here:

- `e_y1` is the weighted lateral tracking error and is the performance output.
- `e_y` is the controller measurement.
- `steering_cmd_in` is the controller output.
- `side_wind_speed_in` is treated as an exogenous disturbance because it is not part of the final `ncont` control-input partition.

MATLAB's `hinfsyn(P,nmeas,ncont)` uses the last `nmeas` outputs of the generalized plant as the controller measurements and the last `ncont` inputs as the controller commands. That is why the ordering of `nominal_inputs` and `nominal_outputs` is important.

The synthesis objective is to minimize the induced norm from exogenous inputs to weighted performance outputs:

```text
minimize gamma = || T_zw ||_infinity
```

For the current code, the achieved value from the latest run is

```text
gamma = 0.8567
```

Because `gamma < 1`, the current weighted lateral-error objective is feasible for the shaped plant. This does not mean every time-domain tracking error is below `max_y_error` for every possible trajectory. The frequency-domain weight defines a shaped gain requirement, while time-domain peak error also depends on reference size, reference slope, wind profile, and steering authority.

## 4. Block Interconnection and Weighting Architecture

### 4.1 Generalized Plant for Synthesis

The H-infinity generalized plant is built using MATLAB named signals and `connect`. The main blocks are:

- `lateral_sys`: original unweighted physical lateral vehicle plant.
- `Rpass`: passes reference signals into the measurement-error equations.
- `Wn`: measurement-noise scaling block.
- `Sub1`: computes `e_y = rmeas_y - y_rig_ns`.
- `Sub2`: computes `e_yaw = rmeas_yaw - yaw_ns`.
- `Wa_y`: lateral tracking-error performance weight.
- `Wa_yaw`: yaw tracking-error performance weight, currently defined but not active in `nominal_outputs`.
- `W_cmd`: steering command shaping block.
- `W_wind`: side-wind disturbance shaping block.

The current synthesis output list is

```text
nominal_outputs = { 'e_y1', 'e_y' }
```

so only `Wa_y` directly affects the H-infinity optimization. The warning about unused `e_yaw1` is expected in the current code because `Wa_yaw` is connected but not selected as a performance output.

### 4.2 Implemented Controller and LFT Analysis Model

After synthesis, the controller `K1` maps

```text
e_y -> steering_cmd_in
```

The physical plant expects `steering_cmd`, so the implemented controller for analysis is

```text
K1_plant = W_cmd*K1
```

This means `W_cmd` remains part of the implemented control path when closing the loop around the true plant.

The unweighted analysis plant is named

```text
nominal_augment_lateral_system
```

Its inputs are

```text
[ r_y, r_yaw, side_wind_speed, steering_cmd ]
```

and its outputs are

```text
[ y_rig, yaw, vy_cg, yaw_rate, e_y, e_yaw, e_y_feedback ]
```

The clean closed-loop model is built by

```text
CL1 = lft(nominal_augment_lateral_system, K1_plant, 1, 1)
```

The resulting closed-loop inputs are

```text
[ r_y, r_yaw, side_wind_speed ]
```

and the closed-loop outputs are

```text
[ y_rig, yaw, vy_cg, yaw_rate, e_y, e_yaw ]
```

This is the correct structure for analyzing the original plant with the controller, without leaving the performance weights inside the physical plant.

### 4.3 Noisy Analysis Model

A second analysis model is constructed as

```text
nominal_augment_lateral_system_with_noise
CL1_with_noise
```

This model keeps the same true plant and same controller, but adds measurement noise before the controller feedback signal:

```text
y_rig_ns = y_rig + n_y_scaled
yaw_ns   = yaw   + n_yaw_scaled
e_y_feedback = r_y - y_rig_ns
```

In the current code, the noise standard deviations are set to zero:

```text
y_noise_std = 0.0
yaw_noise_std = 0.0
```

Therefore `CL1_with_noise` has the correct structure for noise analysis, but the present numerical case has noise disabled.

## 5. Weight Design

### 5.1 Lateral Tracking Weight `Wa_y`

The lateral tracking performance weight is

```text
Wa_y = (1/max_y_error) * tracking_bandwidth_rad/(s + tracking_bandwidth_rad)
```

Current values:

```text
tracking_bandwidth_hz  = 1.0 Hz
tracking_bandwidth_rad = 2*pi rad/s
max_y_error            = 0.18 m
```

At low frequency, `Wa_y` has approximately constant gain

```text
1/max_y_error = 5.56 1/m
```

This means the design asks the controller to keep low-frequency lateral tracking error below about `0.18 m`. Above the bandwidth, the weight rolls off, so high-frequency tracking error is penalized less strongly.

This weight is an absolute-error requirement, not a percentage requirement. If the desired requirement is percentage tracking error, the reference should be normalized or the allowed absolute error should be computed from a chosen reference scale. For example, a 10 percent error requirement for a 2 m lane-change reference corresponds to an absolute error target of `0.2 m`.

### 5.2 Yaw Tracking Weight `Wa_yaw`

The yaw weight is defined as

```text
Wa_yaw = (1/max_yaw_error) * tracking_bandwidth_rad/(s + tracking_bandwidth_rad)
```

Current value:

```text
max_yaw_error = 0.3 rad
```

However, `Wa_yaw` is not currently active in synthesis because `e_yaw1` is not included in `nominal_outputs`. To design a controller that explicitly balances lateral error and yaw-angle error, the synthesis output list should include both weighted errors and `nmeas` should be adjusted if the controller also uses yaw error as a measurement.

### 5.3 Steering Command Weight `W_cmd`

The steering command block is

```text
W_cmd = 0.5/(0.05/pi*s + 1)
```

This maps the normalized controller command `steering_cmd_in` to the physical steering command `steering_cmd`. Its DC gain is `0.5 rad`, and its pole is approximately `62.8 rad/s`, or `10 Hz`.

This block shapes the steering path but it is not a hard saturation. If the real steering system has strict angle or rate limits, those must be checked separately in simulation or included with additional actuator modeling.

### 5.4 Side-Wind Weight `W_wind`

The side-wind disturbance weight is

```text
W_wind = 4.9/(s + 1)
```

This treats `side_wind_speed_in` as a shaped exogenous signal. At low frequency, the gain is `4.9`, so the controller is designed against roughly several meters per second of low-frequency wind disturbance. The pole at `1 rad/s` means the weight emphasizes slow and step-like wind changes more than very high-frequency wind.

For sudden wind steps, this is a better direction than using a very slow low-pass wind weight that mostly penalizes near-DC behavior. If the simulation uses an 8 m/s step but the design weight only represents about 4.9 m/s low-frequency disturbance, the time-domain response can still exceed the desired lateral-error bound.

## 6. Simulation Results

The MATLAB script currently generates three types of analysis results:

1. Open-loop transfer functions, zeros, poles, and magnitude plots for `y_rig` and `yaw` with respect to steering and side-wind speed.
2. Closed-loop Bode magnitude plots for `CL1` and `CL1_with_noise`, including:
   - `e_y / r_y`
   - `e_yaw / r_y`
   - `e_y / side_wind_speed`
   - `e_yaw / side_wind_speed`
3. Time-domain tracking simulations using `plotTrackingSimulation.m`.

The latest run of the current code produced:

| Case | Reference | Side wind | Peak `abs(e_y)` | Peak `abs(e_yaw)` |
|---|---:|---:|---:|---:|
| No Side Wind | default S trajectory, +/-2 m | 0 m/s | 0.264 m | 0.00739 rad |
| Side Wind Steps with Zero Reference | 0 m | 8 m/s step at 5 s | 0.246 m | 0.000827 rad |

These two cases should not be interpreted as "wind improves tracking," because they do not use the same reference trajectory. The no-wind case uses the default S-shaped trajectory, while the wind case uses zero reference to isolate wind disturbance rejection. For a fair wind/no-wind comparison, both simulations should use the same `refYProfile` and only change the `sideWind` input.

The default S trajectory in `plotTrackingSimulation.m` is:

```text
0-3 s:    y_ref = 0
3-10 s:   smooth rise to +2 m
10-17 s:  smooth return to 0
20-27 s:  smooth rise to -2 m
27-34 s:  hold -2 m
34-41 s:  smooth return to 0
41-100 s: y_ref = 0
```

The script also reconstructs the approximate x-y trajectory using constant speed:

```text
x_ref    = integral(vx*cos(ref_phi)) dt
x_actual = integral(vx*cos(phi_actual)) dt
```

and plots the reference and actual vehicle paths in the x-y plane.

## 7. Discussion

The current controller satisfies the active weighted H-infinity objective because `gamma = 0.8567 < 1`. The main design limitation is that only lateral position error is active in the synthesis performance channel. Yaw error is computed and plotted, but the yaw performance weight is not currently part of the optimization.

Another important point is that the wind and steering weights are shaping filters, not hard physical bounds. A large wind step, a fast reference trajectory, or a steering-limited implementation can still create larger time-domain errors than the nominal low-frequency error target. This is why the frequency-domain result must be checked with time-domain simulations.

The current structure is useful because it separates three different systems:

- `lateral_sys`: the original physical plant.
- `nominal_op`: the weighted generalized plant used only for H-infinity synthesis.
- `nominal_augment_lateral_system` and `CL1`: the unweighted true plant and closed-loop controller used for analysis.

This separation prevents the performance weights from being accidentally interpreted as physical vehicle dynamics.

## 8. Current Contributions

This project adds the following pieces beyond a basic H-infinity steering example:

- A named-signal MATLAB implementation of the AlpaSim lateral bicycle model.
- A side-wind-speed input converted to lateral force at the vehicle CG.
- Normal operating ranges for steering and signed side-wind speed.
- A generalized H-infinity plant with reference, disturbance, optional measurement noise, steering command shaping, and tracking-error weighting.
- A clean LFT-based closed-loop analysis model built around the original unweighted plant.
- A matching noisy closed-loop analysis model for future localization-noise studies.
- Reusable simulation plotting through `plotTrackingSimulation.m`, including time histories and x-y trajectory plots.

## 9. Recommended Next Steps

To improve the design and make the results easier to defend:

- Activate yaw tracking in the H-infinity performance outputs if heading-angle tracking is a true requirement.
- Run matched simulations with and without wind using the same reference trajectory.
- Check the commanded steering angle and steering rate in time-domain simulation.
- Decide whether `max_y_error` is an absolute requirement or a percentage-of-reference requirement, then normalize the tracking channel consistently.
- If sudden wind gusts are the main scenario, tune `W_wind` to represent the expected wind-step magnitude and bandwidth.
- If measurement noise matters, set nonzero `y_noise_std` and `yaw_noise_std`, then compare `CL1` and `CL1_with_noise`.

## Reference

R. C. Rafaila and G. Livint, "H-infinity control of automatic vehicle steering," 2016 International Conference and Exposition on Electrical and Power Engineering (EPE), Iasi, Romania, 2016, pp. 031-036, doi: 10.1109/ICEPE.2016.7781297.
