Roblib Planning Tutorial

Installation - Ubuntu:
- sudo apt-add-repository ppa:roblib/ppa
- sudo apt-get update
- sudo apt-get install librl-dev
- extract tutorialPlan.zip to your favourite working directory

Build:
- cd tutorialPlan/build
- cmake ..
- make

Execution:
- ./tutorialPlan


Installation - Windows & VS2010:

Prerequisites:
- Visual Studio 2010 (Express is ok)
- CMake (at least 2.8.11)
- Qt (at least 4.8.5) (http://download.qt-project.org/archive/qt/4.8/4.8.5/qt-win-opensource-4.8.5-vs2010.exe)
- Installed Roblib (follow the instructions at http://www.roboticslibrary.org/tutorials/install-windows)
- extract tutorialPlan.zip to your favourite working directory

Build:
- Open tutorialPlan/CMakeLists.txt in CMake - generate Visual Studio project files in the build directory
- Open build/tutorialPlan.sln in Visual Studio 2010
- Compile RelWithDebInfo (Debug is not supported - sorry)

Execution:
press F5 or open build/RelWithDebugInfo/tutorialPlan.exe

