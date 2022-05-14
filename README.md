# IntersectionControl

[![test](https://github.com/julesdehon/IntersectionControl/actions/workflows/python-app.yml/badge.svg)](https://github.com/julesdehon/IntersectionControl/actions/workflows/python-app.yml)

An environment-agnostic framework for comparing intersection control algorithms

## Getting Started

### Installation

Clone the repository:

`git clone git@github.com:julesdehon/IntersectionControl.git`

`cd IntersectionControl`

It is recommended to create a new Anaconda environment to work in this project. This project was written using python
3.8. Other versions have not been tested:

`conda create --name <environment-name> python=3.8`

`conda activate <environment-name>`

Then install the dependencies:

`pip install -r requirements.txt`

#### Sumo

In order to be able to use the Sumo simulation environment, you must have Sumo installed. The installation instructions
for Sumo can be found [here](https://sumo.dlr.de/docs/Installing/index.html).

#### Checking everything works

You can make sure everything was installed correctly by running `./main.py` or `python main.py` which start up an
experiment using the Sumo environment, and the QBIM intersection control algorithm.

Note that in its current state, the QBIM implementation has some bugs so the simulation may crash after some time.

### Exploring the code

A more detailed and high-level explanation of the various components is being written and will be referred to here.

The directory structure is as follows:

```python
IntersectionControl
├── docs  # Documentation images and files
├── intersection_control  # The main source code package
│   ├── core  # Defines all interfaces and defines the component structure
│   │   ├── environment  # Provides an interface for any environment to implement
│   │   │   ├── environment.py  # Defines the base Environment class
│   │   │   ├── intersectiont_handler.py  # Defines the base IntersectionHandler class 
│   │   │   └── vehicle_handler.py  # Defines the base VehicleHandler class
│   │   ├── communication.py  # Provides an interface for communication - V2V or V2I is possible. Specifically, defines the base MessagingUnit class
│   │   ├── intersection_manager.py  # Defines the base IntersectionManager class
│   │   ├── performance_indicator.py  # Defines the base PerformanceIndicator class (Not yet implemented)
│   │   └── vehicle.py  # Defines the base Vehicle class
│   ├── algorithms  # A collection of intersection control algorithm implementations (for now only QBIM). These are implementations of core.Vehicle and core.IntersectionManager
│   ├── environments  # A collection of environment implementations (for now only SUMO). These are implementations of core.Environment
│   ├── communication  # A collection of communication implementations (for now only DistanceBasedUnit). These are implementations of core.MessagingUnit
│   └── test  # unit tests for various components
└── main.py  # Main script to run an experiment - for now simply runs the QBIM algorithm in the SUMO environment
```