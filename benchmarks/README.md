# NCLT A/B 回放

在仓库根目录构建回放程序，然后指定同一条 NCLT 序列的原始雷达和 IMU 文件。`main` 和 `methodB` 在同一构建里顺序运行，使用相同的输入、外参和回放节奏，各自从空图启动。配置分别是 `cfg/nclt.toml` 和 `cfg/cfg_b/nclt.toml`。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSAPPHIRE_BUILD_BENCHMARKS=ON
cmake --build build --target sapphire_nclt_replay -j4
benchmarks/run_nclt_ab.sh build/sapphire_nclt_replay /data/hits.bin /data/ms25.csv /tmp/nclt-ab
```

提供同一序列的 TUM 真值可以生成对照表。评分环境需要 NumPy、SciPy 和 evo。

```sh
python3 benchmarks/compare_nclt.py /tmp/nclt-ab /data/groundtruth.tum
```

`events.csv` 保存逐帧耗时，`online.tum` 与 `mapped.tum` 分别是在线与最终轨迹。`memory.csv` 记录在线结束、地图导出前后和轨迹导出结束时的 RSS 与累计峰值。`revisits.log` 记录 `methodB` 采用的历史约束与验证结果。
