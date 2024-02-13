# score-rs

Find `n` points out of a set of possible tens of thousands of GPS points, such that the straight distance between them is maximized.
There is one constraint: The finish altitude must not be more than 1000 m less than the start altitude.

The algorithm does the same optimization that [WeGlide](https://www.weglide.org) does to assign a distance to every flight:

1. Build a distance matrix between all points
2. Build a graph with backward min-marginals
3. Traverse this graph to find a solution (path)
4. If this solution does comply with the 1000 m rule, the problem is solved.
5. If not, all possible start candidates and their maximum achievable distances (without the 1000m rule) are calculated and they are checked one by one (with many optimizations) until the maximum achievable distance of all the remaining ones is lower than the current best valid (complying with 1000m rule) solution

The code is based on the excellent [aeroscore-rs](https://github.com/glide-rs/aeroscore-rs) library, but differs in three points:

1. Instead of the full distance matrix, only the triangular half matrix is calculated, minimizing the memory footprint. This comes with the caveat of only allowing for backward min-marginals (optimization is only possible in the backward direction).
2. If the 1000 m altitude is satisfied by the best result, the optimization is similar. If not, this library uses a caching system to quickly determine if start candidates can give a better solution than the current best without traversing the whole graph.
3. Also look for potential solutions by adjusting the start- and end points of a given solution and keeping the middle points constant. This is not used to find the actual solution (as it does not guarantee optimality), but it speeds up the optimization by helping to find better intermediate results and discard candidates that do not offer a better solution

## Develop

Python bindings are generated with [maturin](https://github.com/PyO3/maturin/). Create a virtual env first with

```bash
python -m venv ./env && source .env/bin/activate
```

and install numpy in your virtual env:

```bash
pip install numpy
```

To develop, run

```bash
maturin develop
``` 

or for a (faster) release version

```bash
maturin build --release
pip install .
```

## Test

You can run the tests with

```bash
cargo test
python -m pytest
```