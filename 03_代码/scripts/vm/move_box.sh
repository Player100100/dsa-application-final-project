#!/bin/bash
for i in $(seq 1 160); do
  y=$(python3 -c "import math; print(round(0.30*math.sin($i*0.9),3))")
  ign service -s /world/maze_6x5_v1/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 200 \
    --req "name: \"dyn_box\", position: {x: 1.2, y: $y, z: 0.2}" >/dev/null 2>&1
  sleep 0.5
done
