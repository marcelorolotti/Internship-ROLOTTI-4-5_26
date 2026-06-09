Z-Scanning Automation & Fabrication Repository

This repository accompanies Marcelo ROLOTTI's internship report submitted in June 2026. It contains the hardware design files and software source code developed for the BÖHM Spinal Dynamics Lab (Institut de Psychiatrie et Neurosciences de Paris).
--------------------------
Fabrication Files

2D and 3D designs for laser cutting and 3D-printing, including larva chambers panels and chamber mount.

STEP files for CAD designs

3MF files ready for 3D printing

DXF files ready for laser cutting

--------------------------

Software Architecture Overview

The software repository contains the Program.cs source code for the standalone C# z-scanning application. The program operates on a state machine architecture running across six operational phases:

Initialization: Launches the program, connects to hardware, and initiates the homing sequence.

Alignment: From the home position, the user jogs the stage into place for the first scan.

ParameterInput: The user enters the finish position, number of slices, step size, and slice duration.

Running: The user initiates the scan, and safe movements are automatically executed according to parameters.

Post-Experiment: When the last slice finishes, the user may navigate by menu to any other state except Initialization.

Exit: Disconnects the main loop from hardware before program termination.

--------------------------

Safety Overrides
Soft-Stop Key: Active during Alignment, ParameterInput, and Running states. Pauses program progression to allow backward menu navigation or transition to the Exit state.

Emergency-Stop Key: Active during any state. Instantly overrides all movements and program progression, entering the Exit state for a safe shut down.
