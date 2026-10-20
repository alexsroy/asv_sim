ASV Collision-Avoidance Simulation

Building:
   rm -rf build
   cmake -S . -B build -DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
   cmake --build build

Tests:
    ctest --test-dir build --output-on-failure
Running simulation:
./build/viewer/sim_viewer

