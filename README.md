# SLTS Control

Implements the exponentially stable robust following controller for an upcoming paper to Scitech 2023.

## Dependencies

1. Eigen: Install with `sudo apt-get install libeigen3-dev`

2. Rapidjson: For unit tests, install with `sudo apt-get install rapidjson-dev`

Additionally, you need cmake >= 3.14, such that `FetchContent` is supported. `FetchContent` is used to pull in googletest on-the-fly.

## Building

At the root of this package, run

``` bash
cmake -S . -B build
cmake --build build
sudo cmake --install build
```

## Formula Reference

Refer to `docs/formula_reference.pdf`
