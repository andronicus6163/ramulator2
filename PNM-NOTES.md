# ramulator2 — build & run notes

Verified 2026-09-08 on Ubuntu 22.04, kernel 5.15, 80 cores.
Status: **build OK / run OK**. No patches needed.

## Prerequisites
System `g++` 11 is enough despite the README asking for g++-12 (C++20 subset used
compiles fine). CMake fetches yaml-cpp, fmt and nanobind itself — no manual deps.

The Python bindings are built against the **system** interpreter that CMake finds
(`/usr/bin/python3.10` here), *not* against a conda python. Use that same
interpreter to run the examples or the module will not import.

## Build
```bash
mkdir -p build && cd build
cmake ..
make -j16
cd ..
```
Produces `libramulator.so` in the repo root and
`python/ramulator/_ramulator.cpython-310-x86_64-linux-gnu.so`.

## Run
`examples/` is on the repo root path, and the `ramulator` package lives in
`python/`, so `PYTHONPATH` must point there:
```bash
PYTHONPATH=$PWD/python /usr/bin/python3.10 examples/example_config.py
```
Expected tail: DDR4 timing table, then `Controller cycles: 81302`,
`Avg read latency: 45.2 cycles`, `Read requests: 6`.

## Data prep
None. `examples/traces/example_inst.trace` ships with the repo.
