# Testing

We test this package by comparing controller input/output behavior against that obtained from numerical simulation via Simulink.

To simply rerun the simulation AND generate test data to the appropriate location, run `tests/generate_test_data.m`. To introspect the simulation package, read on.

## Numerical Simulation Sub-package

The numerical simulation sub-package is contained in `tests/slts_control_matlab`. The Simulink model is `test_controller.slx`, created and saved in MATLAB R2020a. It depends on the `DSP System Toolbox` for the LDL-factorization-based matrix inversion block.

In the numerical simulation, the SLTS travels along a trajectory that is defined in `tests/slts_control_matlab/params.json` in the `"path"` array

``` json
{ 
  "mission" : {
    "path": [
      "Array of 3-arrays defining the path"
    ]
  }
}
```

By convention, the simulation saves output data to the `tout` and `yout` structure. Invoke the simulation by calling the `SimulationRunner(path_to_save)` function will additionally save data in `yout` to a `test_controller.json` file.


## C++ testing

To build the C++ tests, pass `-DBUILD_TESTING=ON` to `cmake` when configuring, i.e.

``` bash
cmake -S . -B build -DBUILD_TESTING=ON
```

Then run the appropriate executable

``` bash
./build/tests/test_controller
```

Or use CTest

``` bash
cd build # Run tests INSIDE the build folder!! 
ctest -j4 -C Debug -T test --output-on-failure
```

The test fixture loads `tests/slts_control_matlab/test_controller.json` to obtain expected values of SLTS states, inputs and outputs, while running the controller


