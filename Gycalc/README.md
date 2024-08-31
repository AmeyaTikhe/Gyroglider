# Throttle vs Pitch Analysis Tool

## Overview

This C program reads data from a file (`readings1.txt`) containing throttle and pitch values from an experiment. The program calculates a theoretical throttle value based on the pitch angle and then compares it with the real throttle value to determine a correction factor (`fac`). This correction factor is averaged across all readings, and the most frequent correction factor is identified. This information can be used to calibrate the Gyroglider setup by aiding in finding the ratio of change in angle to the change in throttle value with further improvements.

### Key Features:
- Reads and parses real throttle and pitch data from a file.
- Calculates theoretical throttle values using trigonometric equations.
- Computes a correction factor to account for real-world discrepancies.
- Calculates the average correction factor and identifies the most frequent correction factor.
- Outputs the average and the most frequent correction factor for further analysis.

## Prerequisites

To compile and run this program, you will need:
- A C compiler, such as GCC.
- A text file named `readings1.txt` containing the input data in a specific format.


