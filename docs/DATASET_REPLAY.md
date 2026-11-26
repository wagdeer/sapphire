# Dataset replay

Method B uses the `sapphire::OdometryPipeline` sensor and map interfaces. Use the calibration for the LiDAR and IMU that recorded the data.

| Rig | Profile | LiDAR topic | IMU topic |
|---|---|---|---|
| NCLT | `cfg/cfg_b/nclt.toml` | Raw Velodyne hits | Microstrain MS25 |
| HILTI 2022 handheld | `cfg/cfg_b/hilti_2022_handheld.toml` | `/hesai/pandar` | `/alphasense/imu` |
| S3E Alpha, Bob, Carol | `cfg/cfg_b/s3e_alpha.toml`, `s3e_bob.toml`, `s3e_carol.toml` | `/<Robot>/velodyne_points` | `/<Robot>/imu/data` |

Use the [NCLT replay instructions](../benchmarks/README.md) for a standalone run. When feeding sensor messages through a ROS wrapper, pass the matching profile as `config_file` and remap the LiDAR and IMU topics to the recording. Check that timestamps are expressed on the same sensor clock.

Set `methodB.map_directory` to a fresh destination for an empty-map run. After the final sensor callback, request the needed map or snapshot, then call `finishMapping()` before reading the final exports.
