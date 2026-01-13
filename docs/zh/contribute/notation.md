# 术语

本指南中的文本和图表中使用了以下术语、符号和装饰器。

## 符号

- Bold face variables indicate vectors or matrices and non-bold face variables represent scalars.
- The default frame for each variable is the local frame: $\ell{}$.
  Right [superscripts](#superscripts) represent the coordinate frame.
  If no right superscript is present, then the default frame $\ell{}$ is assumed.
  An exception is given by Rotation Matrices, where the lower right subscripts indicates the current frame and the right superscripts the target frame.
- Variables and subscripts can share the same letter, but they always have different meaning.

## Acronyms

| Acronym    | Expansion                                                                                                                                                                                      |
| ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| AOA        | Angle Of Attack. Also named _alpha_.                                                                                                                           |
| AOS        | Angle Of Sideslip. Also named _beta_.                                                                                                                          |
| FRD        | Coordinate system where the X-axis is pointing towards the Front of the vehicle, the Y-axis is pointing Right and the Z-axis is pointing Down, completing the right-hand rule. |
| FW         | Fixed-wing (planes).                                                                                                                                        |
| MC         | MultiCopter.                                                                                                                                                                   |
| MPC 或 MCPC | MultiCopter Position Controller. MultiCopter Position Controller. MPC is also used for Model Predictive Control.                               |
| NED        | Coordinate system where the X-axis is pointing towards the true North, the Y-axis is pointing East and the Z-axis is pointing Down, completing the right-hand rule.            |
| PID        | Controller with Proportional, Integral and Derivative actions.                                                                                                                 |

## Symbols

| Variable                                              | 描述                                                                                                                                                                                                                   |
| ----------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| $x,y,z$                                               | Translation along coordinate axis x,y and z respectively.                                                                                                                                            |
| $\boldsymbol{\mathrm{r}}$                             | Position vector: $\boldsymbol{\mathrm{r}} = [x \quad y \quad z]^{T}$                                                                             |
| $\boldsymbol{\mathrm{v}}$                             | Velocity vector: $\boldsymbol{\mathrm{v}} = \boldsymbol{\mathrm{\dot{r}}}$                                                                                                                           |
| $\boldsymbol{\mathrm{a}}$                             | Acceleration vector: $\boldsymbol{\mathrm{a}} = \boldsymbol{\mathrm{\dot{v}}} = \boldsymbol{\mathrm{\ddot{r}}}$                                                                                      |
| $\alpha$                                              | Angle of attack (AOA).                                                                                                                                                            |
| $b$                                                   | Wing span (from tip to tip).                                                                                                                                                      |
| $S$                                                   | Wing area.                                                                                                                                                                                           |
| $AR$                                                  | Aspect ratio: $AR = b^2/S$                                                                                                                                                                           |
| $\beta$                                               | Angle of sideslip (AOS).                                                                                                                                                          |
| $c$                                                   | Wing chord length.                                                                                                                                                                                   |
| $\delta$                                              | Aerodynamic control surface angular deflection. A positive deflection generates a negative moment. A positive deflection generates a negative moment.                |
| $\phi,\theta,\psi$                                    | Euler angles roll (=Bank), pitch and yaw (=Heading).                                                                                                           |
| $\Psi$                                                | Attitude vector: $\Psi = [\phi \quad \theta \quad \psi]^T$                                                                                       |
| $X,Y,Z$                                               | Forces along coordinate axis x,y and z.                                                                                                                                                              |
| $\boldsymbol{\mathrm{F}}$                             | Force vector: $\boldsymbol{\mathrm{F}}= [X \quad Y \quad Z]^T$                                                                                   |
| $D$                                                   | Drag force.                                                                                                                                                                                          |
| $C$                                                   | Cross-wind force.                                                                                                                                                                                    |
| $L$                                                   | Lift force.                                                                                                                                                                                          |
| $g$                                                   | Gravity.                                                                                                                                                                                             |
| $l,m,n$                                               | Moments around coordinate axis x,y and z.                                                                                                                                                            |
| $\boldsymbol{\mathrm{M}}$                             | Moment vector $\boldsymbol{\mathrm{M}} = [l \quad m \quad n]^T$                                                                                                  |
| $M$                                                   | Mach number. Can be neglected for scale aircraft.                                                                                                                                    |
| $\boldsymbol{\mathrm{q}}$                             | Vector part of Quaternion.                                                                                                                                                                           |
| $\boldsymbol{\mathrm{\tilde{q}}}$                     | Hamiltonian attitude quaternion (see `1` below)                                                                                                                                                   |
| $\boldsymbol{\mathrm{R}}_\ell^b$ | Rotation matrix. Rotates a vector from frame $\ell{}$ to frame $b{}$. $\boldsymbol{\mathrm{v}}^b = \boldsymbol{\mathrm{R}}_\ell^b \boldsymbol{\mathrm{v}}^\ell$ |
| $\Lambda$                                             | Leading-edge sweep angle.                                                                                                                                                                            |
| $\lambda$                                             | Aspect ratio. $$AR = b^2/S$$.                                                                                                                                                        |
| $w$                                                   | Wind velocity.                                                                                                                                                                                       |
| $p,q,r$                                               | Angular rates around body axis x,y and z.                                                                                                                                                            |
| $\boldsymbol{\omega}^b$                               | Attitude vector. $$\Psi = [\phi \quad \theta \quad \psi]^T$$.                                                                                                                  |
| $\boldsymbol{\mathrm{x}}$                             | General state vector.                                                                                                                                                                                |

- `1` Hamiltonian attitude quaternion. Hamiltonian attitude quaternion. $$\boldsymbol{\mathrm{\tilde{q}}} = (q_0, q_1, q_2, q_3) = (q_0, \boldsymbol{\mathrm{q}})$$. To represent a vector in local frame given a vector in body frame, the following operation can be used: $\boldsymbol{\mathrm{\tilde{v}}}^\ell = \boldsymbol{\mathrm{\tilde{q}}} \, \boldsymbol{\mathrm{\tilde{v}}}^b \, \boldsymbol{\mathrm{\tilde{q}}}^_{}$ (or $\boldsymbol{\mathrm{\tilde{q}}}^{-1}{}$ instead of $\boldsymbol{\mathrm{\tilde{q}}}^_{}$ if $\boldsymbol{\mathrm{\tilde{q}}}{}$ is not unitary). $\boldsymbol{\mathrm{\tilde{v}}}{}$ represents a _quaternionized_ vector: $\boldsymbol{\mathrm{\tilde{v}}} = (0,\boldsymbol{\mathrm{v}})$

### Subscripts / Indices

| Subscripts / Indices | 描述                                                                               |
| -------------------- | -------------------------------------------------------------------------------- |
| $a$                  | Aileron.                                                         |
| $e$                  | Elevator.                                                        |
| $r$                  | Rudder.                                                          |
| $Aero$               | Aerodynamic.                                                     |
| $T$                  | Thrust force.                                                    |
| $w$                  | Relative airspeed.                                               |
| $x,y,z$              | Component of vector along coordinate axis x, y and z.            |
| $N,E,D$              | Component of vector along global north, east and down direction. |

<a id="superscripts"></a>

### Superscripts / Indices

| Superscripts / Indices | 描述                                                                                                           |
| ---------------------- | ------------------------------------------------------------------------------------------------------------ |
| $\ell$                 | Local-frame. Local-frame. Default for PX4 related variables. |
| $b$                    | Body-frame.                                                                                  |
| $w$                    | Wind-frame.                                                                                  |

## Decorators

| Decorator                       | 描述                                 |
| ------------------------------- | ---------------------------------- |
| $()^\*$      | Complex conjugate. |
| $\dot{()}$   | Time derivative.   |
| $\hat{()}$   | Estimate.          |
| $\bar{()}$   | Mean.              |
| $()^{-1}$    | Matrix inverse.    |
| $()^T$       | Matrix transpose.  |
| $\tilde{()}$ | Quaternion.        |
