#!/bin/bash
cd mod-metric-ff
make clean
make depend
make
cd ..
make clean
make depend
make
