# Pursuit-Evasion Simulation

This MATLAB project simulates a pursuit-evasion scenario where an attacker robot tries to reach a protected circle while a defender robot attempts to intercept the attacker. The simulation uses control strategies and geometric computations to dynamically visualize the interactions between the attacker and defender. The strategies and computation is based on the research paper "Local-game Decomposition for Multiplayer Perimeter-defense Problem"(2018) by Daigo Shishika and Vijay Kumar.

## Description

The simulation involves an attacker trying to intrude a protected area represented by a circle. A defender moves around this circle attempting to intercept the attacker. The code uses a Proportional-Integral-Derivative (PID) controller to adjust the defender's speed based on the attacker's position. The attacker's path is determined by calculating tangent points to a secondary circle, and the optimal intrusion point on the protected circle is calculated.

## Features

- Dynamic simulation of pursuit-evasion with an attacker and defender
- PID controller for defender's speed adjustment
- Calculation of tangent points and optimal intrusion point
- Visualization of the protected circle, secondary circle, winning region, tangent paths, and positions of attacker and defender

The program will prompt to enter the initial parameters for the simulation:
- Attacker position (x,y)
- Defender position in degrees
- Maximum attacker speed
- Maximum defender speed


