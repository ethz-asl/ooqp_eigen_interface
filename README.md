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

### Building and Installing

To build and install the OOQP-Eigen interface:

	mkdir build
	cd build
	cmake ..
	sudo make install
	
	
Usage
------------

Add the following to the CMakeLists.txt of your project:

	find_package(OOQPEI REQUIRED)
	
Then, add `${OOQPEI_INCLUDE_DIRS}` this to the *include_directories* and `${OOQEI_LIBRARIES}` to the *add_libraries*.


Unit Tests
------------

Unit tests are built as soon as the folder gtest exists in the root folder.

Download Google Test:

	wget http://googletest.googlecode.com/files/gtest-1.7.0.zip
	unzip gtest-1.7.0.zip
	ln -s gtest-1.7.0 gtest
	rm gtest-1.7.0.zip
	
To build and run the unit tests:

	mkdir build
	cd build
	cmake ..
	make
	make check


Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ooqp-eigen_interface/issues).


[OOQP]: http://pages.cs.wisc.edu/~swright/ooqp/
[Eigen]: http://eigen.tuxfamily.org
