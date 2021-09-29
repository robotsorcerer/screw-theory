### Screw Theory Toolbox for Robotics

An adaptation of Pardos' [STFR Book and Code](https://github.com/DrPardosGotor/Screw-Theory-Toolbox-for-Robotics-ST24R)..

This toolbox encompasses a collection of functions to facilitate the analysis of the robot kinematics through formulas of the theory of Lie groups and the geometry of screws.
The “ST24R” provides a relatively reduced range of functions for the representation of serial-link manipulator robots, even though you can quite easily extend the toolbox to other robot mechanisms. The functions support screw theory kinematics for robotics, as well as robot simulations.
Naturally, the toolbox also covers the necessary screw mathematics in order to follow the examples of this book. For instance, the toolbox has functions for: conversion between data types, position and rotation representation, homogeneous transformations, twists, screws or POE.
At this moment, the “ST24R” is programmed only in MATLAB©. All functions are documented and commented thoroughly for didactical reasons, so as to facilitate their comprehension.
It is important for you to understand that the software for “ST24R” is developed mainly for teaching purposes. Therefore, it is not optimised for industrial use. The main aim of this code’s use is that you better understand screw theory concepts, rather than the software’s execution performance. Nevertheless, as you will appreciate, the advantages of this new mathematical approach are so great that they will become the cornerstone of very powerful applications for you.
Be aware that the “ST24R” functions implementing the canonical inverse kinematics subproblems are built to always give a result (exact or approximate). This means that in the case of having a problem without an exact solution, the algorithms will give you the most approximate result. These functions have been programmed in this way because it is very beneficial for the simulations and leads to quite natural solutions for many robots. Should you need a different approach for these functions, you can modify them for your own aims and utility.

#### Resources

[Pardos Book](https://www.amazon.com/Screw-Theory-Robotics-practical-KINEMATICS/dp/1717931812/ref=sr_1_1?ie=UTF8&qid=1541950371&sr=8-1&keywords=screw+theory+for+robotics).
