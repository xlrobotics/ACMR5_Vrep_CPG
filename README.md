# ACMR5_Vrep_CPG
A CPG controller to generate serpentine locomotion on ACMR5 robot in Vrep.
Use Genetic Algorithm to optimize the parameters of the CPG network, some functions and libraries are referred from the work in https://github.com/sayantanauddy/hierarchical_bipedal_controller. 

## Requirements
- deap 1.2.2 
- vrep pro-edu_3.4.0
- python 3.5.5

## Run
- Open Vrep first, load the scene and run it.
- run ga.py to carry out genetic algorithm (GA) for parameter optimization. See deap documentation to modify the configurations of the algorithm. 

## Tips

- Vrep:
  - Make sure you have following files in your directory, in order to run the various examples:
    1. vrep.py
    2. vrepConst.py
    3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)
    4. simpleTest.py (or any other example file)
  - Add scene-disabled.ttt to your Vrep repository.

- deap
