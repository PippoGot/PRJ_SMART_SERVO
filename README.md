# Smart Servo Project
----------------------------------------------------------------

The scope of this repository is to design a new controller board for the MG996R
hobby servo, in order to improve position accuracy and performance.

In particular, the goal is to:

- Implement a more accurate position controller;

- Implement a current controller to limit stall overheating;

- Implement a I2C interface to control the servo, for:
    + current and position readings;
    + flexible and overwritable PID controller & threshold values;
    + calibration routine to estimate motor parameter;
    + possibility of daisy-chaining more servos using a limited number of ports;

More useful things to have are:

- Thermal safety shutdown;


----------------------------------------------------------------
## Folder Structure:

- "DATASHEETS": contains all of the datasheets for the components used,
both in the original controller board and in the new developed board;

- "default servo control board": contains the KiCAD files of the reverse engineered
original servo controller board;

- "MODELS & SIMULATIONS": contains the Simulink and LTSpice models, as well as the
Matlab scripts for parameters and controller tuning;

- "SOURCE": contains all the code for the microcontroller;

- "STM32 PDFs & DATASHEET": contains the documentation of the microcontroller
applied to the board, as well as some useful PDFs for making and programming
embedded projects using the microcontroller itself;

- "STM32F103C8T6 minimum board": contains the KiCAD files of a minimum board
for any embedded project using that specific microcontroller;