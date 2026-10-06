# Results

- `synthetic_rooms.txt`: output of `make bench-synthetic`, the reproducible run on a generated 512 by 512 room map. The file header records the map parameters, compiler, and machine type.

The table from the course report on the MovingAI `64room_005` map is in the main [README](../README.md#course-report-original-run-on-64room_005). Those figures came from the original implementation and have not been regenerated with the code in this repository.

Costs and expansion counts are deterministic. Timings depend on hardware and load, so compare ratios, not absolute milliseconds. To refresh the real-benchmark numbers, follow "Reproduce on the real benchmark" in the main README and save the output here, for example as `results/64room_005.txt`.
