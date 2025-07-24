# Chrono_data

This repository contains utilities for mechanism kinematics.

## Four-bar linkage solver

`four_bar.py` provides a function `four_bar_angles` that computes the coupler and rocker angles of a planar four-bar linkage for a given set of link lengths and input crank angle. The solver uses purely geometric methods and only relies on the Python standard library.

Run the demo from the command line:

```bash
python3 four_bar.py
```

## Four-bar linkage simulation

`chrono_four_bar.py` builds the same mechanism using the PyChrono physics
engine. The script requires the PyChrono Python bindings, which must be
installed separately. During the simulation it prints the instantaneous
angles of the moving links.

```bash
python3 chrono_four_bar.py
```
