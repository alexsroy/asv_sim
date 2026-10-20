ASV Collision-Avoidance Simulation

Commands:

Make commands executable:
chmod +x build.sh test.sh run.sh

then

./build.sh            # configure + build
./build.sh --clean    # wipe build/ first, then build
./test.sh             # run the unit tests
./run.sh              # launch the viewer


or manually:

Building:
   rm -rf build
   cmake -S . -B build -DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
   cmake --build build

Tests:
    ctest --test-dir build --output-on-failure
Running simulation:
./build/viewer/sim_viewer

