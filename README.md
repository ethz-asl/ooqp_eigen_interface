OOQP-Eigen Interface
======================

Overview
---------------

This C++ software provides a simple interface to use [OOQP] with the [Eigen] linear algebra library.

*The Object Oriented Quadratic Programming solver package ([OOQP]) is written by Mike Gertz and Steve Wright from the University of Chicago ([copyright notice](http://pages.cs.wisc.edu/~swright/ooqp/COPYRIGHT.html)). This software is an independent interface to the OOQP library and is not affiliated with OOQP in any way.*

**Authors: Péter Fankhauser, Christian Gehring, Stelian Coros**<br />
Contact: Péter Fankhauser, pfankhauser@ethz.ch<br />
Affiliation: Autonomous Systems Lab, ETH Zurich

### Links

* [OOQP User Guide](http://pages.cs.wisc.edu/~swright/ooqp/ooqp-userguide.pdf)
* [OOQP Paper](http://pages.cs.wisc.edu/~swright/ooqp/ooqp-paper.pdf)
* The MA27 documentation is under `ma27-1.0.0.tar.gz -> /doc/ma27_Fortran.pdf`


Installation
------------

### Dependencies

- [OOQP]: Object-oriented software for quadratic programming,
- [Eigen]: Linear algebra library.

### Building

This version uses [Catkin] as build system. There is a [CMake] version in the [cmake](https://github.com/ethz-asl/ooqp_eigen_interface/tree/cmake) branch.

To build, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone https://github.com/ethz-asl/ooqp_eigen_interface.git
	cd ../
	catkin_make

### Building using Findooqp.cmake
Since ooqp_catkin can't be distributed, I modified this package to add a find_package command to search for ooqp on the system in the standard directories `usr/local/include` and `/usr/local/lib`. You need to build ooqp and ma27 with the `-fPIC` option. To do so run in the ma27 folder:

	./configure FFLAGS="-O -fPIC"
	make
	sudo make install

And run in the ooqp folder

	export CXXFLAGS="-O -fPIC"
	make -j4
	sudo make install

After that you can build using `catkin_make` or `catkin build`. If using the former change CMakeLists.txt

	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake")

Otherwise leave `CMAKE_MODULE_PATH` alone.

Usage
------------

TODO: Add an example project.


Unit Tests
------------

Run the unit tests with

	catkin_make run_tests_ooqp_eigen_interface


Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ooqp-eigen_interface/issues).


[OOQP]: http://pages.cs.wisc.edu/~swright/ooqp/
[Eigen]: http://eigen.tuxfamily.org
[Catkin]: http://wiki.ros.org/catkin
[CMake]: http://www.cmake.org/
