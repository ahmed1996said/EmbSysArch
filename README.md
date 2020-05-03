# Overview

Two major projects we're developed on FRDM-KL25Z Microcontroller with ARM processor.

## Project 1:
•Developed a system that modifies the LED colors on the board based on the accelerometer.
•Provided synchornization structure between the I2C communication carrying the accelerometer measurements and the LED controls to improve timing behavior for the non-preemptive system.
•Improved timing performance of the system by removing busy-waiting in the base code and added optimization features such as Finite State Machines & a Run to Completion Scheduler API.
•Tested the system performance using oscilloscope and mixed-signals analysis.

## Project 2:
•Configured, soldered a touch display screen to the system.
•Developed code to control buck-converter driving a High Brightness LED (HBLED).
•Utilized Real-time operating system (CMSIS-RTOS) API to improve the sharing of the processor by several peripherals in the system.
•Configured Direct Memeory Access (DMA) controller for hardware triggered ADC conversions.
•Developed code to graph real-time current measurements of the HBLED on the touch screen.
•Tested the system performance using oscilloscope and mixed-signals analysis.

Key concepts: Multithreading,I2C, SPI, UART,RTOS, RTCS, Scheduling, DMA,ADC,DAC,Mixed signals.
