#!/usr/bin/env bash
cd build && \
source install/setup.bash && \
nosetests pyros_setup.tests
