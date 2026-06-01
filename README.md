# Predictive Artificial Potential Field Algorithm

## Research Implementation 
This has been coded from scratch with the mathematical ideas inspired from [International Journal of Advanced Robotic Systems](https://arxiv.org/abs/2602.19062)

## Abstract of the Algorithm
Path planning for high-speed unmanned surface vehicles requires more complex solutions to reduce sailing time and save energy. This article proposes a new predictive artificial potential field that incorporates time information and predictive potential to plan smoother paths. It explores the principles of the artificial potential field, considering vehicle dynamics and local minimum reachability. The study first analyzes the most advanced traditional artificial potential field and its drawbacks in global and local path planning. It then introduces three modifications to the predictive artificial potential field-angle limit, velocity adjustment, and predictive potential to enhance the feasibility and flatness of the generated path. A comparison between the traditional and predictive artificial potential fields demonstrates that the latter successfully restricts the maximum turning angle, shortens sailing time, and intelligently avoids obstacles. Simulation results further verify that the predictive artificial potential field addresses the concave local minimum problem and improves reachability in special scenarios, ultimately generating a more efficient path that reduces sailing time and conserves energy for unmanned surface vehicles.

## Simulations
2D simulations were run on F1tenth-gym-ros based simulator using ROS2 Humble. 

<img width="2480" height="3508" alt="papf_visualisation pdf_page-0001" src="https://github.com/user-attachments/assets/864dfb39-0a07-4d2a-94c6-21ad02f52a98" />

This algorithm can be used as a building block for fast local planner suitable for UGVs traversing at high speeds.
