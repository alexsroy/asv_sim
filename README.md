ASV Collision-Avoidance Simulation

Commands:

Make commands executable:
chmod +x build.sh test.sh run.sh

then

./build.sh            # configure + build
./build.sh --clean    # wipe build/ first, then build
./test.sh             # run the unit tests
./run.sh              # launch the viewer,  speed 5, crossing course
./run.sh 8                  # faster, still crossing
./run.sh 5 patrol           # shorthand for scenario/patrol.course
./run.sh 5 scenario/my.course   # or a full path
./run.sh 5 crossing parallel    # two targets (GNN association)
./run.sh 5 none             # ASV alone, like the old behavior


or manually:

Building:
   rm -rf build
   cmake -S . -B build -DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
   cmake --build build

Tests:
    ctest --test-dir build --output-on-failure
Running simulation:
./build/viewer/sim_viewer




