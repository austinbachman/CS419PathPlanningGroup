## University of Nevada, Reno
## CS491 Aerial Robotics, Dr. Kostas Alexis
## Path Planning Project Group
### Austin Bachman, Jeff Williams, David Hull, David Nielsen, Robert Trimble

# Quadrotor Path Planning Algorithm Comparison and Analysis

### Setup:
The OMPL app must be installed. (http://ompl.kavrakilab.org/installation.html)

Place `SE3QuadcopterPlanningBenchmark.cpp` into `omplapp-1.x.x-Source/demos/SE3RigidBodyPlanning`.

Add the line
```
add_omplapp_demo(demo_SE3QuadcopterPlanningBenchmark SE3RigidBodyPlanning/SE3QuadcopterPlanningBenchmark.cpp)
```
to `omplapp-1.x.x-Source/demos/CMakeLists.txt`.

Run cmake, then `make` in `omplapp-1.x.x-Source/build/Release`.

Place `input` folder into `omplapp-1.x.x-Source/build/Release/bin`.

### Input data:
Input files take the form:
```
<environment file name>
<start X position>
<start Y position>
<start Z position>
<goal X position>
<goal Y position>
<goal Z position>
```

### Running:
Run planning benchmark with:
```
$ ./demo_SE3QuadcopterPlanningBenchmark input/<input file name>
```

### Output data:
This will output data to the terminal, and create a .log file.

To process the log file, follow instructions at: http://ompl.kavrakilab.org/benchmark.html

Each planner being benchmarked will be run once extra and will output the solution path and metrics for that solution to the screen.

Each path point takes the form:
```
<X position> <Y position> <Z position> <unit quaternion component 1> <component 2> <component 3> <component 4>
```

Each environment that the quadrotor will work in has been tested, with the results placed in the `output` folder. 
