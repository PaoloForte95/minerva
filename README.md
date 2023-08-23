# Minerva: A Framework for Multi-Robot Task Assignment

This software implements a loosely-coupled framework for _online task assignment, motion planning, coordination and control of multiple heterogeneous robots_. The approach accounts for the important real-world requirement that tasks can be posted asynchronously. 


Its main features are:

* Goals can be posted and paths computed online.
* The coordination method is not specific to a particular motion planning technique.

The software includes a basic 2D robot simulation and a simple built-in motion planner (which depends on the <a href="http://ompl.kavrakilab.org/">OMPL</a> and <a href="http://www.mrpt.org/">MRPT</a> libraries).

# Overview
The algorithm provided by this implementation is detailed in:

* Paolo Forte, Anna Mannucci, Henrik Andreasson and Federico Pecora, <a href="https://ieeexplore.ieee.org/abstract/document/9387084"> "Online Task Assignment and Coordination in Multi-Robot Fleets </a>," in IEEE Robotics and Automation Letters, vol. 6, no. 3, pp. 4584-4591, July 2021, doi: 10.1109/LRA.2021.3068918.


[![Examples usages of minerva](https://img.youtube.com/vi/HCi1M1h7TCE/0.jpg)](https://www.youtube.com/watch?v=HCi1M1h7TCE "Examples usages of minerva")

The systematic algorithm exploits systematic search for optimal task assignment, where interference is considered as a cost and estimated with knowledge of the kinodynamic models and current state of the robots. Safety is guaranteed by an online coordination algorithm, where the absence of collisions is treated as a hard constraint.


# Installation

To install a coordination framework, follow the instruction provided <a href="https://github.com/FedericoPecora/coordination_oru.git">here</a>. 

To install the assignment framework, clone this repository and compile the source code with gradle (redistributable included):
```
$ git clone https://github.com/PaoloForte95/assignment_oru.git
$ cd assignment_oru
$ ./gradlew build

```


# Running an example
A number of examples are provided. Issue the following command from the source code root directory for instructions on how to run the examples:
```
$ ./gradlew run
```

# Optimization Problems
The API provides two types of optimization problems, implemented using the Google <a href="https://developers.google.com/optimization?hl=en">Ortools</a> library:
* <a href="https://developers.google.com/optimization/lp?hl=en">Linear Optimization</a>
* <a href="https://developers.google.com/optimization/cp?hl=en">Constraint Optimization</a>

Both extend the abstract ```AbstractOptimizationProblem``` class, which can be used as a basis to create your own optimization problem. The ```AbstractOptimizationProblem``` class has no dependency on the  Google Ortools library, thus, you can rely on any library to create your own optimization problem.

## Create your own optimization problem
To create your own optimization problem, the following methods need to be implemented: 
* ```evaluateBFunction```: define how to evaluate the B(⋅) function. The B(⋅) function reflects user-defined costs, each depending on the single robot state, path, and dynamics. 
* ```evaluateInterferenceCost```: define how to evaluate the F(⋅) function The F(⋅) function considers the additional cost associated with interference and coordination between the trajectories that robots follow to reach their goals. This cost derives from the imposition of precedences among robots in critical sections. 
* ```solve```: Get and save the current optimal solution of the optimization problem into the variable ```currentAssignment```.

# Algorithms
Minerva includes two algorithm to find the optimal solution:
* ```Systematic Algorithm```.
* <a href="https://en.wikipedia.org/wiki/Simulated_annealing">Simulated Annealing Algorithm</a>.


Both extend the abstract ```AbstractOptimizationAlgorithm``` class, which can be used as a basis to create your own algorithm.



# Visualizations
The API provides three visualization methods:

* ```BrowserTaskVisualization```: a browser-based visualization.
* ```JTSDrawingPanelTaskVisualization```: a Swing-based visualization.
* ```RVizTaskVisualization```: a visualization based on the ROS visualization tool <a href="http://wiki.ros.org/rviz">RViz</a>.


All three visualizations implement the abstract ```TaskFleetVisualization``` class, which can be used as a basis to create your own visualization.

Most examples use the ```BrowserTaskVisualization```. The state of the fleet can be viewed from a browser at <a href="http://localhost:8080">http://localhost:8080</a>. The image below shows this visualization for the ```TaskAssignmentWithoutMap``` example:

![BrowserVisualization GUI](images/browser-gui.png "Browser-based visualization")

Each task is characterized by a start and goal position. A blue hexagon indicates the task's start position, while a red one indicates the goal position. Each robot will navigate from its current position to the goal location of the task by passing through the start location of the task.

The a Swing-based GUI provided by class ```JTSDrawingPanelVisualization``` looks like this:

![Swing-based GUI](images/coord.png "Swing-based visualization")

This GUI allows to take screenshots in SVG, EPS and PDF formats by pressing the ```s```, ```e``` and ```p``` keys, respectively (while focus is on the GUI window). Screenshots are saved in files named with a timestamp, e.g., ```2017-08-13-11:13:17:528.svg```. Note that saving PDF and EPS files is computationally demanding and will temporarily interrupt the rendering of robot movements; SVG screenshots are saved much quicker.

The ```RVizVisualization``` visualization publishes <a href="http://wiki.ros.org/rviz/DisplayTypes/Marker">visualization markers</a> that can be visualized in <a href="http://wiki.ros.org/rviz">RViz</a>. The class also provides the static method ```writeRVizConfigFile(int ... robotIDs)``` for writing an appropriate RViz confiuration file for a given set of robots. An example of the visualization is shown below.

![RVizVisualization GUI](images/rviz-gui.png "RViz-based visualization")

The visualization with least computational overhead is the ```RVizVisualization```, and is recommended for fleets of many robots. The ```BrowserVisualization``` class serves an HTML page with a Javascript which communicates with the coordinator via websockets. Although rendering in this solution is less efficient than in RViz, the rendering occurs on the client platform (where the browser is running), so its computational overhead does not necessarily affect the coordination algorithm. The ```JTSDrawingPanelVisualization``` is rather slow and not recommended for fleets of more than a handful of robots, however it is practical (not requiring to start another process/program for visualization) and relatively well-tested.


# Sponsors
This project is supported by

* The <a href="http://semanticrobots.oru.se">Semantic Robots</a> Research Profile, funded by the <a href="http://www.kks.se/">Swedish Knowledge Foundation</a>
* The <a href="https://iliad-project.eu/">ILIAD Project</a>, funded by the <a href="https://ec.europa.eu/programmes/horizon2020/">EC H2020 Program</a>
* The iQMobility Project, funded by <a href="https://www.vinnova.se/">Vinnova</a>

* The <a href="https://www.more-itn.eu/">MORE Project</a> Research Profile, funded by the European Union’s Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement. 

# License
minerva - Robot-agnostic online task assignment for multiple robots

Copyright &copy; 2017-2023 Paolo Forte

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
