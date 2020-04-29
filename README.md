# Rotary Inverted Pendulum: Swing Up and Stabilization
Swing Up and Stabilization (through LQR or SMC) of a Rotary Inverted Pendulum.

## Video

[LINK](https://youtu.be/2koXcs0IhOc) to YouTube video.

## Running

Upload code on board with:
```
arduino --upload main.ino
```

## Components

- Rotary linear potentiometer WDD35D4-5K to measure pendulum position;
- 12V DC motor + Incremental encoder to rotate base and measure base position;
- DFRobot Dual H-Bridge (L298N).
- Arduino Mega 2560;

## Schematics

<p align="left"><img src="media/schematics.jpg"></p>

## Control System

### Sensoring

Pendulum (phi) and base (theta) positions are measured through the potentiometer and encoder, respectively. Their derivatives are computed and then filtered through a simplified 1-dimension Kalman Filter:
<p align="left"><img src="media/kf.png"></p>

### Swing Up

Control action is given by
<p align="left"><img src="media/swingup.png"></p>
where
<p align="left"><img src="media/e.png"></p>

### Linear Quadratic Regulator (LQR)

The dynamic model of the rotary inverted pendulum is given by the following equations
<p align="left"><img src="media/model.png"></p>
which can be derived from the application of the Euler–Lagrange equations <br />
<p align="left"><img src="media/lagrange.png"></p>
to the Lagrangian <br />
<p align="left"><img src="media/l.png"></p>

The model can be linearized
<p align="left"><img src="media/linmodel.png"></p>
and transformed in state space form <br />
<p align="left"><img src="media/ssmodel.png"></p>

Motor parameters (which are not directly available) are estimated through Simulink Parameter Estimation tool: check matlab_utils/motorParamEst.m

Control action is given by the u = -Kx that minimizes the following
<p align="left"><img src="media/lqr.png"></p>

### Sliding Mode Control (SMC)

Control action is given by
<p align="left"><img src="media/smc.png"></p>
where the sliding surface is given by <br />
<p align="left"><img src="media/sigma.png"></p>

## Issues, Limitations and Future Developments

Vibrations and oscillations are caused by
- Motor deadzone between -1.5 and 1.5 V;
- Low resolution of pendulum angle sensor (potentiometer).

Future Developments
- Use encoder to measure pendulum angle;
- Use motor without deadzone at low voltages;
- Implement n-dimention Kalman Filter.

## References

- [Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter)
- [Linear Quadratic Regulator](https://en.wikipedia.org/wiki/Linear–quadratic_regulator)
- [Sliding Mode Control](https://en.wikipedia.org/wiki/Sliding_mode_control)
