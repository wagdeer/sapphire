# NCLT 回放和 A/B

在 `method_b` 工作区构建并独立运行 Method B。最后一个参数是传感器回放时长，示例覆盖 NCLT 2012-01-08 的完整序列。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSAPPHIRE_BUILD_BENCHMARKS=ON
cmake --build build --target sapphire_nclt_replay -j4
mkdir -p /tmp/nclt-ab
REPLAY_RATE=12 OMP_NUM_THREADS=2 OMP_THREAD_LIMIT=2 build/sapphire_nclt_replay cfg/cfg_b/nclt.toml \
  /data/hits.bin /data/ms25.csv /tmp/nclt-ab/methodB 5634
```

从 `main` 的独立工作区构建基准，用链接到该分支的外部回放程序记录同样的输入和传感器时间范围，并把输出放到 `/tmp/nclt-ab/main`。两组各自从空图启动，采用相同的外参、回放节奏与计时范围。Method B 的 NCLT 回放程序只链接本分支的 Sapphire。

两组结果目录需要有 `online.tum`、`mapped.tum`、`events.csv` 和 `memory.csv`。事件表包含 `stamp,ms,fresh,converged`，内存表包含 `phase,rss_kib,peak_kib`，记录 `online_end` 和 `map_end`。用同一份 TUM 真值评分，环境需要 NumPy、SciPy 和 evo。

```sh
python3 benchmarks/compare_nclt.py /tmp/nclt-ab /data/groundtruth.tum
```

Method B 把封存子图与回环检索数据写入其回放结果的 `submaps/`，`revisits.log` 记录历史约束和验证结果。
