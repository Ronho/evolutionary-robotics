# Evolutionary Robotics

This is a repository containing code! Besides that, it also contains files without code.

## Setup

**Requirements**

In order to setup the project, you should have installed the following
- `cmake>=3.14`
- `gnuplot` (Required for visualization using matplotplusplus)

For `gnuplot` make sure that you can reach it from your console. In case of windows, make sure that you restarted your system after installing gnuplot and adding it to PATH.


**Building the Project**

Go into the `build` directory and run `cmake ..`. Once finished, run `cmake --build .`.

Depending on your system, you can run the code using one of the following commands from the `build` directory:

```console
// Windows
Debug\EvoRob.exe
```

```console
// MacOS
./EvoRob
```