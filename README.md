# PerchDetector
Raspberry pi stereo vision code for thesis project


## Project Folders

### Helper Libraries

#### ADS1X15
This folder contains an API for Adafuits ADS1x15 ADC products. It is mostly copied from 
[Adafruits own driver](https://github.com/adafruit/adafruit_ads1x15), with changes made to support
the Linux I2C device driver.

#### STSW_IMG040
This folder contains an API for the STM VL53L8CX tim-of-flight lidar
sensor. The code is copied from STMs [STSW-IMG040 package](https://www.st.com/en/embedded-software/stsw-img040.html),
with updates made to the Package folder to support the Linux SPI device driver.

#### hough3d
This folder contains an implementation of the iterative 3-dimensional hough transform proposed by Dalitz et al. (citation below).
It is based on the [code provided by the authors](https://doi.org/10.5201/ipol.2017.208), with changes made in order to:

* Reject lines that make too steep an angle with the X-Y plane.
* Reject lines with a width that is outside a given range.
* Reject lines with a ratio of length to width below a given value.
* Improve performance by removing unnecessary copies and conversions between custom types and Eigen matrices. 

[1] C. Dalitz, T. Schramke, and M. Jeltsch, “Iterative Hough Transform for Line Detection in 3D Point Clouds,” Image Processing On Line, vol. 7, pp. 184–196, Jul. 2017, doi: 10.5201/ipol.2017.208.


### Core Utilities

The "util" folder contains code for a number of base functions and interfaces, such as:

* Base task API definition and tools for managing task lifecycle and execution.
* Memory management for shared data.
* Tools for loading settings from JSON files. 
* Logging utility.

### Task Types

The "tasks" folder contains all specific task implementations. This is where each step in the image processing pipeline is performed,
as well as IO management and data reporting and visualization helpers.


## Program Architecture

### User Interface

The interface to the program is defined in "main_cmd.cpp" and "main_exe.cpp", by the functions that get mapped to command names in 
`void make_commands(std::map<std::string, cli_cmd_executor>& commands)`. Each of these mappings defines a command that can be entered
from the command line once `main` is started. These commands operate on the global scope; they have to do with:

* Managing lifecycles of individual tasks (such as `start` and `stop`).
* Gathering data and status from a group of tasks (such as `capture`).
* Managing and inspecting overall program state (such as `status` and `tick`).

There are also task-specific commands, defined in each tasks constructor with a call to the `void Task::declare_cli_command(const char* cmd, command_executor_t executor)`
function. This similarly maps a command name to a function that executes it, but the command is used to operate on task-specific states and data; in coding terms
it acts like a method bound to an object while the previously described global commands act like a C-style function. An example of where task-specific commands are 
used is in the `DataMapper` class, where the `map` command tells the task to open a memory mapped file and write the data from its source task to the file for sharing with other processes.

The global commands are called in the form `<command name> <args...>`, while task-specific commands are called in the form `<task name> <command name> <args...>`.


### Tasks

The program is divided into individual types of `Task`, each instance of which executes in its own thread. This multi-threaded approach
is intended to solve a few problems:

* Allow computationally intensive stages of the image processing pipeline to run in parallel.
* Allow IO-bound tasks, such as image acquisition or state-based grasper control, to run asyncronously without blocking each other.
* Allow the program to be configurable, by swapping out different variants of task types in the program initialization (such as when the `WSL_SIM` flag is defined).
* Allow the user to start and stop individual parts of the program, to diagnose or avoid exceptions.
* Allow the user to inspect intermediate data values without interrupting the processing pipeline, such as with the `DataMapper` class.

#### Execution

Tasks are constructed at program initialization in the `void make_tasks(program_context& context)` function. Each task is assigned to
the programs `TaskManager`, which is responsible for creating a function that executes the task in its own thread. The `TaskManager` is 
also responsible for tracking statistics about execution rate for each task and controlling rate limits.

#### Communication

Tasks communicate by two methods:

* For small, single configuration values where direct copying is OK, tasks are passed to eachother as `shared_ptr`s and data accessed by getter/setter methods.
* For large data that should not be copied and has a lifetime unknown to the producer task, tasks derive the `DataSource` task type to use its data interface.

The `DataSource` class provides helpers to atomically update a tasks latest data value, and for consumer tasks to atomically access that latest value, as well
as check if a past update has gone stale (a newer data value is available). This process uses `shared_ptr`s to manage lifetimes. For efficient memory 
management, the `BlockPool` and `BlockAllocator` classes allocate and free updates from a stack of fixed-size blocks. The `BLOCK_ALLOC` flag can be un-defined
to switch to default `malloc` based `shared_ptr` allocation.

Tasks deriving from `DataSource` can only produce one type of update. In other words, each `DataSource` has only one channel (though `struct` datatypes are supported, so multiple data values can be
mixed into one channel).

