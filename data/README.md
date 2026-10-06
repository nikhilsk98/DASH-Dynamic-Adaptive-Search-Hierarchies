# Benchmark data

Map files are not committed (see `.gitignore`). Two options:

## Real benchmark: MovingAI

Download maps from the MovingAI grid benchmarks:
https://www.movingai.com/benchmarks/grids.html

The course report used `64room_005.map` from the 512 by 512 room set. Save it as:

```text
data/64room_005.map
```

Expected format:

```text
type octile
height <H>
width <W>
map
<H rows of W characters>
```

Traversable cells are `.`, `G`, and `S`. Every other symbol is treated as blocked.
Movement in this project is 4-connected, so the `octile` type in the header is ignored.

## Synthetic room map

If you do not have the benchmark file, generate a similar one (it is not the benchmark):

```bash
python3 scripts/make_room_map.py --out data/synthetic_rooms.map
```

`make bench-synthetic` does this automatically.
