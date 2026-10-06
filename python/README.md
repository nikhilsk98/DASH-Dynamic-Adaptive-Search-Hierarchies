# Python reference

A readable port of the C++ implementation in `../cpp`. It uses the same rules, neighbour order, and tie-breaking, so global expansions, probe expansions, probe counts, and costs match the C++ build exactly. It is slower by roughly 20 to 25 times, so use the C++ build for timing.

Requires Python 3.10 or newer. No third-party packages.

```bash
python3 dash_reference.py ../data/synthetic_rooms.map 3 510 490 496 --k 50 --probe-budget 500
python3 -m unittest discover -s tests -v
```
