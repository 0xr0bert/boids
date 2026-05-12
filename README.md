# Boids Simulation

A C++ implementation of the Boids flocking algorithm.

## Features

- Command-line arguments for configuration
- Random initialization of boids
- Simple flocking behavior simulation
- Output of simulation data to CSV

## Builds

Build with Meson:

```shell
meson setup buildDir
mseon compile -C buildDir
```

Requires a C++20 compiler.

## Run

```shell
./boids > output.csv
```

This runs the simulation and writes the output to `output.csv` (redirected from stdout).

## Implementation notes

- Brute-force O(N^2) implementation
- World wraps at edges.