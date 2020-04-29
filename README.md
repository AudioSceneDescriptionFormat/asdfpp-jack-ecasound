C++ Implementation of the Audio Scene Description Format (ASDF)
===============================================================

This is incomplete and obsolete and has been superseded by
https://github.com/AudioSceneDescriptionFormat/asdf-rust.

Dependencies
------------

* A C++ compiler supporting C++17 (including `std::filesystem`)
* JACK
  * http://jackaudio.org/
  * `libjack-dev` or `libjack-jackd2-dev`
  * `jackd` or `jackd1` or `jackd2`
* Ecasound
  * http://www.eca.cx/ecasound/
  * `libecasoundc-dev` *and* `ecasound`
* pugixml
  * https://pugixml.org/
  * `libpugixml-dev`
* GML
  * https://github.com/ilmola/gml
* {fmt}
  * http://fmtlib.net/
  * `libfmt-dev`

API Documentation
-----------------

Run `doxygen` in the main directory to create the documentation.
The generated HTML documentation can be accessed via `html/index.html`.
