# Report - Robotics Engineer Intern Assessment (Rameshwar Khokle)

Repository: https://github.com/rameshwarkhokle557-oss/ROS_assignment (branch `rameshwar-assessment`).
Build and run instructions: `README_ASSESSMENT.md`. Test outputs: `evidence/`.

## 1. Status

| Task | Status | Main result |
|---|---|---|
| S1 resize | done (with test dimensions) | `/odom` 0.610 m vs ground truth 0.609 m; turn 63.64 vs 63.66 deg |
| S2 sensors | done | two configs; topics, frames and TF tree checked |
| B1 nearest obstacle | done | 2.497 m at -0.5 deg to the test_box face at 2.5 m |
| B3 emergency stop | done | true gap 0.41 m, no collision |
| B2 square | done | closing error 2.8 cm (open) vs 0.0 cm (closed), ground truth |
| S3 bonus | done | parking_area scenario; LiDAR reading matches the layout |

## 2. What I changed
- `rb_description/urdf/common.xacro`: `wheel_r` 0.10 -> 0.12, `body_l` 0.60 -> 0.80 (test values, see section 5).
- `rb_control/config/diff_2w.yaml`, `skid_4w.yaml`: `wheel_radius` 0.12, so the controller matches the model.
- `rb_description/config/robots/dual_lidar_imu.yaml` and `rgb_camera.yaml`: new sensor configurations.
- `rb_assignment`: new nodes `nearest_obstacle`, `emergency_stop`, `drive_square` (entry points in `setup.py`,
  dependencies in `package.xml`).
- `rb_gazebo/scenarios/parking_area.yaml`: new scenario.
- `README_ASSESSMENT.md`, `REPORT.md`, `evidence/`.

## 3. Design choices
- B1: invalid readings (`inf`, `NaN`, outside the sensor range) are skipped; the bearing is `angle_min + i * angle_increment`.
- B3: the LiDAR is at the chassis centre, so the stop range is `front_offset (0.40 m) + stop_gap (0.45 m)`. Only +-20 deg
  ahead are checked. Missing scans, mostly invalid scans and readings below `range_min` lead to zero velocity; a stop is latched.
- B2: timing uses the simulation clock because the simulation ran at 55-75 % real time on WSL2. Closed loop uses `/odom`
  distance and yaw; the real error is measured with `/ground_truth/odom`.

## 4. Problems and debugging
1. **Robot did not load after the S1 edit.** `/odom` and `/ground_truth/odom` did not exist. `git diff` showed two stray
   characters (`SS`) after `</robot>` in `common.xacro`, which made the XML invalid. I removed the line, rebuilt, and the
   robot spawned normally.
2. **Which controller file matters.** Two controller files exist (`diff_2w`, `skid_4w`). `default.yaml` showed that the default
   robot uses `diff_2w`, so I updated both files for consistency but verified the change with the default robot.
3. **Stray files in a commit.** An accidental shell redirect created three oddly named files, and `git add -A` committed them.
   Before pushing I undid the commits (`git reset --soft HEAD~2`), removed the files (`git rm`) and committed again;
   since then I add files by name.
4. **Wrong IMU topic name.** My first evidence run for config A used `/imu`; the real topic is `/imu/data`.
   I re-ran the check and corrected the evidence file.
5. **Slow simulation.** The simulation ran at about 55-75 % real time on WSL2, so topic rates were lower than configured
   (the 10 Hz LiDAR measured about 5.3 Hz) and wall-clock timing was unreliable. B2 therefore runs with `use_sim_time:=true`.
6. **Stop distance.** A LiDAR range of 0.45 m would leave only 0.05 m between the chassis front and the box, because the LiDAR
   is 0.40 m behind the front. I added the `front_offset` parameter and verified the true gap with ground truth (0.41 m).
7. **Git push.** GitHub rejected my password (passwords are no longer accepted for git); a token without enough permissions
   gave HTTP 403. A classic personal access token with the `repo` scope worked.

## 5. Incomplete work and limitations
- The reviewer's target dimensions for S1 were not available to me. I used 0.12 m wheel radius and 0.80 m chassis length as
  test values; changing them needs the Xacro properties, the YAML `wheel_radius`, and `front_offset` in B3.
- `skid_4w` was edited for consistency but not launched, and its `wheel_separation_multiplier` (1.34) was not re-measured.
- B2 has one run per mode. Gazebo models little wheel slip, so the open-loop error is smaller than it would be on hardware.
- The wall thickness in the parking scenario was inferred (about 0.2 m) from the LiDAR reading, not measured.
- Sensor topic rates were measured only briefly.

## 6. Possible improvements
- Repeat B2 several times and report the mean and spread; tune the heading gain of the closed-loop controller.
- B3 with a slowing zone before the stop distance and a recovery behaviour.
- One launch file per task and an automated test that checks the B3 gap using the ground-truth topic.

## 7. AI assistance
I used an AI assistant (Claude) for guidance, code drafting and explanations. I ran and tested everything myself in the
simulation, and I can explain the code and the configuration changes.
