# Active Disturbance Rejection Control (ADRC) - Pendulum

This project implements an Active Disturbance Rejection Control (ADRC) system coded in C, designed to control a pendulum system. The ADRC method enhances control performance by actively rejecting disturbances and accommodating model uncertainties, providing a robust alternative to traditional PID controllers.

## Project Overview

The core idea behind ADRC is to treat disturbances and model uncertainties equivalently, allowing the system to operate effectively even with a simplified model. This project uses a second-order ADRC structure to control the dynamics of a pendulum.

## Features

- **ADRC Implementation**: A classical PID-type controller with an observer to handle disturbances.
- **Pendulum Plant Model**: A mathematical model of a pendulum with differential equations governing its motion.
- **Robust Control**: Capable of handling nonlinearities and disturbances effectively.

## Mathematical Model

The dynamics of the pendulum are represented by the following equation:

$$ \tau = m l^2 \theta'' + k \theta' + m g l \sin(\theta) $$

Where:

- $\tau$: Torque
- $m = 0.5 ~ \text{kg}$: Mass of the pendulum
- $l = 1 ~ \text{m}$: Length of the pendulum
- $k = 0.5 ~ \text{Nms}$: Damping coefficient
- $g$: Acceleration due to gravity

The system can be reduced to a second-order differential equation:

$$ \theta'' = \frac{\tau}{ml^2} - \frac{(k\theta' + mgl \sin(\theta))}{ml^2} $$

With:

- $f(t) = -\frac{(k\theta' + mgl \sin(\theta))}{ml^2}$
- $b_0 = \frac{1}{ml^2}$
- $y = \theta$
- $u = \tau$

The observer is defined as:

$$
\begin{align*}
\dot{z}_1 &= z_2 + l_1(y - z_1) \\
\dot{z}_2 &= z_3 + l_2(y - z_1) + b_0u \\
\dot{z}_3 &= l_3(y - z_1) \\
\end{align*}
$$

The control command is given by:

$$ u = \frac{k_p(r - z_1) - k_dz_2 - z_3}{b_0} $$

## Implementation

The ADRC strategy is implemented in C and structured to separate the controller parameters and execution steps. The pendulum model is similarly implemented to facilitate integration with the controller.

- **ADRC Control Step**: `ADRC_Control_Step` function updates the internal state of the controller.
- **Pendulum Simulation**: Simulates the pendulum's behavior under the control strategy.

## Simulation

The simulation demonstrates the system's response using a sample time of 10 ms. Initial attempts with a 100 ms sample time were unsuccessful, suggesting further tuning might be needed. The reference "Tuning and Implementation Variants of Discrete-Time ADRC" by Gernot Herbst and Rafal Madonski, Ph.D., provides insights for future improvements.

## Acknowledgments
This project is inspired by the work of Gernot Herbst in "A Simulative Study on Active Disturbance Rejection Control (ADRC) as a Control Tool for Practitioners."

## Author
Simone Bertoni
For more insights, visit [Simone Bertoni Lab](https://simonebertonilab.com/).

## Contact
Feel free to reach out via [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/) for any questions or collaboration opportunities.
