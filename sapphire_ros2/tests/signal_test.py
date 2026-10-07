#!/usr/bin/env python3
"""Real-process checked control-path test; guard timeout is test failure, never clean drain."""
import pathlib
import signal
import subprocess
import sys
import tempfile

with tempfile.TemporaryDirectory(prefix="sapphire-b3-signal-") as directory:
    path = pathlib.Path(directory)
    config = path / "empty.toml"
    config.write_text(f'[general]\nsave_map = 0\nsave_path = "{directory}/"\n[pose_graph]\nenabled = false\n')
    for request in (signal.SIGINT, signal.SIGTERM, None):
        args = [sys.argv[1], "--ros-args", "-p", f"algorithm_config:={config}"]
        if request is None:
            args += ["-p", "finish:=true"]
        process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        output = []
        try:
            for line in process.stdout:
                output.append(line)
                if "lidar_mode=" in line:
                    if request is not None:
                        process.send_signal(request)
                    break
            tail, _ = process.communicate(timeout=30)
            output.append(tail)
            text = "".join(output)
            print(f"REQUEST {request}\n{text}")
            assert process.returncode == 0, f"checked process exit {process.returncode}"
            assert "B3 checked finish success=1" in text, "missing checked result"
        except BaseException:
            process.kill()
            process.wait()
            raise
print("PASS SIGINT, SIGTERM, finish parameter: checked finish precedes context shutdown")
