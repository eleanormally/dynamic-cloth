# Dynamic Cloth Subdivision
Advanced Computer Graphics Final Project 2025

# Building
To buid DynCloth, make sure that you have CMake 3.10 or newer installed.

- navigate to `dynamic-cloth/build`
- run `cmake ../src`
- run `build`
- MacOS
    - run `./render.app/Contents/MacOS/render --cloth ../inputs/[filename]`
- Other
    - run `./render --cloth ../inputs/[filename]`

# Running
To begin a simulation, press `a`. 
To stop the simulation, press `x`.
To iterate the simulation by one step, press `<space>`.

# Cloth File
NOTE: provot_shear_correction is now ignored, and shear corrections use the same factor as structural corrections

