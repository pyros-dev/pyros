#!/usr/bin/env bash
cd build && \
source devel/setup.bash && \
make -j1 tests && \
make -j1 run_tests && \
catkin_test_results .
