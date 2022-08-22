#!/bin/bash
rm -rf build

pushd libopencm3 > /dev/null && \
    make -j8 clean > /dev/null && \
    popd > /dev/null
