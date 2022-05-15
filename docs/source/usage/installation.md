# Installation

## Prerequisites

### Python
The project requires python3 (>= 3.7)

### Sumo
In order to be able to use the SumoEnvironment (intersection_control.environments.sumo.SumoEnvironment), you will 
need to have [SUMO installed](https://sumo.dlr.de/docs/Installing/index.html), and the `SUMO_HOME` environment 
variable should be set correctly as described [here](https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html)

### Anaconda
It is recommended to create a new [Anaconda](https://www.anaconda.com/) environment when using this project:
```shell
$ conda create --name <environment-name> python=3.8
$ conda activate <environment-name>
```

## Installing IntersectionControl

There are a couple of ways to install this library, and you should choose the way you install it depending on how 
you plan on using it and what you would like to do with it.

The reason for suggesting the 2nd and 3rd option below is that this is a first version of the library, and so I 
assume that some APIs will need modifications and so most users of the library at this stage will likely want to be 
modifying its source code while using it.

1. [Via pip](install-via-pip): If you plan to define your own control algorithms and environments entirely from 
   scratch - and/or to use the algorithms/environments from the library out-of-the box.
2. [Clone and install in editable mode](clone-and-install-in-editable-mode): If you would like the flexibility of 
   being able to modify the library's source code - which may be necessary, for example if you are defining a new 
   algorithm and feel like the existing API is not sufficient and needs to be extended, but you will be writing your 
   own code in a separate project space.
3. [Work in the repository directly](work-in-repo-directly): If you want the same flexibility as in (2), but want 
   to work directly in the library's source files.

(install-via-pip)=
### Install via pip

The library is available to install via pip
```shell
$ pip install intersection-control
```

You will then be able to make use of the library in any python source file:
```python
from intersection_control.core import Vehicle
```

(clone-and-install-in-editable-mode)=
### Clone and install in editable mode
```shell
$ git clone git@github.com:julesdehon/IntersectionControl.git
$ cd IntersectionControl
$ pip install -e .  # (install in editable mode)
```

You can then create a new separate project and be able to make use of the library in any python source file:
```python
from intersection_control.core import Vehicle
```

And any modifications that you make to source files in the `IntersectionControl` project will be reflected directly 
in your pip installation

(work-in-repo-directly)=
### Work in the repository directly
```shell
$ git clone git@github.com:julesdehon/IntersectionControl.git
$ cd IntersectionControl
$ pip install -r requirements.txt
```