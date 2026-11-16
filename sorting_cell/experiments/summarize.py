"""Tổng hợp lại results/<dir>/scene_*.json thành summary.json (thêm độ trễ lập kế hoạch lại nếu có)."""
import glob
import json
import sys
from pathlib import Path

import numpy as np

from sortcell.eval.metrics import summarize

for d in sys.argv[1:]:
    rs = [json.loads(Path(f).read_text()) for f in sorted(glob.glob(f"{d}/scene_*.json"))]
    s = summarize(rs)
    if "replan_times" in rs[0]:
        rt = np.concatenate([r["replan_times"] for r in rs])
        s["replan_latency_ms"] = {"median": float(np.median(rt) * 1000), "p95": float(np.percentile(rt, 95) * 1000),
                                  "max": float(rt.max() * 1000), "n": int(rt.size)}
        s["wait_local_s_per_scene"] = float(np.mean([r["wait_local_time"] for r in rs]))
    else:
        rt = np.concatenate([r["plan_times"] for r in rs])
        s["plan_latency_ms"] = {"median": float(np.median(rt) * 1000), "p95": float(np.percentile(rt, 95) * 1000),
                                "max": float(rt.max() * 1000), "n": int(rt.size)}
    Path(d, "summary.json").write_text(json.dumps(s, indent=1))
    print(d, json.dumps(s, indent=1))
