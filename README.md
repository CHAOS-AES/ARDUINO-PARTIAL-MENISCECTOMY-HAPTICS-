# PARTIAL MENISCECTOMY HAPTICS

This is a repository for a project on reality-based modelling of partial meniscectomy.

This project builds on other open-source projects:
- Arduino SimpleFOC library for BLDC control, MIT Licence (Copyright (c) Antun Skuric).
- Basic Linear Algebra library for Arduino, MIT Licence (Copyright (c) tomstewart89).



Repository contains code and CAD models for:
1) Prototype for collecting force-position data from Acufect Upbiter Punch, with ARDUINO MKR Zero based FSR-sensor and potentiometer.
2) Protype for rendering haptic feedback of partial meniscectomy using self-designed hardware, Arduino DUE with SimpleFOC shield, BLDC motor, and AMT 102V encoder.
3) Arduino code for sensor fusion of real-time finite element simulation and empirical data using Kalman Filter.
4) Visualization of partial meniscectomy punch using Processing.

Prototype for collecting data:
![image](https://user-images.githubusercontent.com/78421762/133457903-c202d6c7-8d1a-4292-8d2d-aaf9bed40d0f.png)

Prototype for haptic rendering:
![image](https://user-images.githubusercontent.com/78421762/133458198-22d59474-da87-4391-bc65-bfd4d2baa5f2.png)

![image](https://user-images.githubusercontent.com/78421762/133458282-5e0b2b49-fe14-40e2-82d8-60fbe14adc72.png)


Instructions for running haptic rendering:
1) Build hardware according to bill-of-materials and CAD-models.
2) Download Arduino code.
3) Install hapticFEM-library, simpleFOC-library, Basic Linear Algebra library.
4) Run Arduino sketch: KF_HapticEstimate_with_FEMlib.ino
5) Launch "Processing" and run "PUNCHER_3D_RevF.pde".