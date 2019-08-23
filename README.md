# Particles
[![Build Status](https://travis-ci.org/dfridovi/dumbo.svg?branch=master)](https://travis-ci.org/dfridovi/particles)
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

## Why *particles*?
*Particles* is a lightweight, quick-and-dirty C++ library for particle filtering. It should be easy to use and efficient to customize.

## Overview
This repository is entirely written in C++, and is structured around the `cmake` cross-compilation paradigm.

## Dependencies
The main external dependencies are just for logging and command line flags. Installation should be available from your operating system's package manager.
* `eigen3` (header-only linear algebra library)
* `glog` (Google's C++ logging tools)
* `gflags` (Google's C++ command line flags)
* `boost` (general C++ toolchain)

## Documentation
Documentation is auto-generated using Doxygen and may be found [here](https://dfridovi.github.io/particles/documentation/html/).

## Build instructions
As with any other `cmake` build, just download the repository and from the top directory type:

```bash
mkdir build && cd build && cmake .. && make
```
