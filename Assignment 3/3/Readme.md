
==libcontrollDLL==

This directory contains code to create a shared library called libcontrollDLL which provides the realtime controllers for the simulator.

To compile your code:

    mkdir build
    cd build
    cmake ..
    make   

To make the simulator load the file libcontrolDLL.so, simply start it in the build/ directory:

    pumasim
