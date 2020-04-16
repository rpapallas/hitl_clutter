This work is to appear in the proceedings of *IEEE International Conference on Robotics and Automation (ICRA) 2020, Paris, France*.
More details [here](https://pubs.rpapallas.com/icra2020/).

# Non-Prehensile Manipulation in Clutter with Human-In-The-Loop

# Installation
- Ubuntu 14.04
- You need ROS (Indigo).
- Boost 1.64 (locally installed on Ubuntu 14.04).
- OMPL installed from source.
- MuJoCo 200

# Configuration
- You need to change in `MujocoGlobal.cpp` the path to `PROJECT_ROOT_PATH` and `MJ_KEY_PATH` to your paths.
- You need to obtain a MuJoCo license.

# Demos
To run the demos on specific scenes use the following commands:

**GRTC-HITL:**

```shell script
rosrun nonprehensile_planning grtc_hitl <scene_name>.xml <number_of_objects> <planner_name>
```

**GRTC-Heuristic:**

```shell script
rosrun nonprehensile_planning grtc_heuristic <scene_name>.xml <number_of_objects> <planner_name>
```

**RRT/KPIECE:**

```shell script
rosrun nonprehensile_planning rtc <scene_name>.xml <number_of_objects> <planner_name>
```

Where `<scene_name>` could be `s1`, `s2`, ..., `s10` or `parallel_1`, `parallel_2`, ..., `parallel_20`. The 
`<number_of_objects>` should be the number of objects in the scene, which should be 10 for all the scenes provided. The
value for `<planner_name>` can be either `RRT` or `KPIECE` (case-sensitive).

# License
This work is licensed under GPLv3. The full license can be found 
[here](https://github.com/rpapallas/hitl_clutter/blob/master/LICENSE).

# Citation

If you used this work or part of this work in your work, please consider citing
the paper below.

### Bibtex
```
@inproceedings{papallas2019non,
  title={Non-Prehensile Manipulation in Clutter with Human-In-The-Loop},
  author={Papallas, Rafael and Dogar, Mehmet R},
  booktitle={{IEEE} International Conference on Robotics and Automation},
  year={2020}
}
```

### Plain
```
Papallas, R. and Dogar, M.R., 2020. Non-Prehensile Manipulation in Clutter with Human-In-The-Loop. IEEE International Conference on Robotics and Automation.
```
