# Dynamic Cloth Subdivision
Advanced Computer Graphics Final Project 2025

# Building
To buid DynCloth, make sure that you have CMake 3.10 or newer installed.

- navigate to `dynamic-cloth/build`
- run `cmake ../src`
- run `make`
- MacOS
    - run `./render.app/Contents/MacOS/render --cloth ../inputs/[filename]`
- Other
    - run `./render --cloth ../inputs/[filename]`

# Running
To begin a simulation, press `a`. 
To stop the simulation, press `x`.
To iterate the simulation by one step, press `<space>`.

# Cloth File
A Cloth file (extension arbitrary) defines a cloth to simulate and takes the
following form:
```
k_structural [double, defines spring constant for structural springs]
k_shear [double, defines spring constant for shear springs]
k_bend [double, defines spring constance for bend springs]
damping [double, defines damping force to apply (simulates friction)]

correction [double, see Provot et. al. for Provot cloth correction]

m [int, number of masses in x direction] [int, number of masses in y direction]

[4 instances of corners that take the following form]
p [double, x] [double, y] [double, z]

fabric_weight [double, defines mass per node in simulation, distributed over nodes and area]

timestep [double, timestep to iterate over]

[arbitrary number of fixed points specific as the following form]
[each fixed point marks an existing point in the grid as fixed]
f [int, x value of mass] [int, y value of mass] [double, x] [double, y] [double, z]
```
