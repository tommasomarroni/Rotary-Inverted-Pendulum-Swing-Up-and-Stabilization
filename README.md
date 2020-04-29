# Rotary-Inverted-Pendulum-Swing-Up-and-Stabilization
Swing Up and Stabilization (through LQR or SMC) of a Rotary Inverted Pendulum.

## Video

[LINK](https://youtu.be/2koXcs0IhOc) to YouTube video.

## Components

Used components:
- Rotary linear potentiometer WDD35D4-5K to measure pendulum position;
- 12V DC motor + Incremental encoder to rotate base and measure base position;
- Arduino Mega 2560;
- DFRobot Dual H-Bridge (L298N).

## Schematics

Schematics:
<p align="left"><img src="media/schematics.jpg"></p>

##

<p align="left"><img src="media/e.png"></p>
<p align="left"><img src="media/kf.png"></p>
<p align="left"><img src="media/l.png"></p>
<p align="left"><img src="media/lagrange.png"></p>
<p align="left"><img src="media/linmodel.png"></p>
<p align="left"><img src="media/lqr.png"></p>
<p align="left"><img src="media/model.png"></p>
<p align="left"><img src="media/sigma.png"></p>
<p align="left"><img src="media/smc.png"></p>
<p align="left"><img src="media/ssmodel.png"></p>
<p align="left"><img src="media/swingup.png"></p>







## Running

Upload code on board with:
```
arduino --upload main.ino
```

## References

- [Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter)
- [Linear Quadratic Regulator](https://en.wikipedia.org/wiki/Linear–quadratic_regulator)
- [Sliding Mode Control](https://en.wikipedia.org/wiki/Sliding_mode_control)
