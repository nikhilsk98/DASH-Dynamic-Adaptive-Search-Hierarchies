#!/usr/bin/env python3
"""Generate a room-style grid map in MovingAI .map format.

The map is a square grid divided into square rooms by one-cell walls. Every
pair of adjacent rooms is joined by a single door at a random position, so the
map is connected and has many narrow passages, similar in spirit to the
MovingAI 64room family. It is a synthetic stand-in, NOT the benchmark file.

Example:
    python3 scripts/make_room_map.py --out data/synthetic_rooms.map
"""

import argparse
import random
from pathlib import Path


def build(size: int, room: int, seed: int) -> list[list[str]]:
    if size % room != 0:
        raise SystemExit("--size must be a multiple of --room")
    rng = random.Random(seed)
    grid = [["."] * size for _ in range(size)]
    rooms = size // room

    # Walls on the boundary between neighbouring rooms.
    for k in range(1, rooms):
        for t in range(size):
            grid[t][k * room] = "#"
            grid[k * room][t] = "#"

    # One door per pair of adjacent rooms, never on a wall intersection.
    for ry in range(rooms):
        for rx in range(rooms):
            if rx < rooms - 1:
                grid[ry * room + rng.randrange(1, room)][(rx + 1) * room] = "."
            if ry < rooms - 1:
                grid[(ry + 1) * room][rx * room + rng.randrange(1, room)] = "."
    return grid


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--size", type=int, default=512, help="grid width and height (default 512)")
    p.add_argument("--room", type=int, default=64, help="room width and height (default 64)")
    p.add_argument("--seed", type=int, default=5, help="random seed (default 5)")
    p.add_argument("--out", type=Path, default=Path("data/synthetic_rooms.map"))
    args = p.parse_args()

    grid = build(args.size, args.room, args.seed)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    header = f"type octile\nheight {args.size}\nwidth {args.size}\nmap\n"
    args.out.write_text(header + "\n".join("".join(r) for r in grid) + "\n", encoding="utf-8")
    print(f"wrote {args.out} ({args.size}x{args.size}, room size {args.room}, seed {args.seed})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
