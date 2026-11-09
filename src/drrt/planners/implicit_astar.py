"""A* trên Ĝ biểu diễn ngầm: mốc tối ưu để kiểm chứng sự hội tụ của dRRT* (Mục 8.1)."""
from __future__ import annotations

import heapq
import itertools
import time

from ..tensor_graph import TensorGraph
from .base import Result


def implicit_astar(graph: TensorGraph, time_limit: float | None = None) -> Result:
    t0 = time.perf_counter()
    result = Result()
    tick = itertools.count()
    g_cost = {graph.start: 0.0}
    parent = {graph.start: None}
    closed = set()
    heap = [(graph.h(graph.start), next(tick), graph.start)]
    expansions = 0
    while heap:
        if time_limit is not None and time.perf_counter() - t0 > time_limit:
            break
        _, _, v = heapq.heappop(heap)
        if v in closed:
            continue
        closed.add(v)
        expansions += 1
        if v == graph.target:
            path = [v]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            result.path = path[::-1]
            result.cost = g_cost[v]
            result.first_time = time.perf_counter() - t0
            result.first_cost = result.cost
            break
        base = g_cost[v]
        adjacency = [rm.adj[a].tolist() for rm, a in zip(graph.roadmaps, v)]
        for u in itertools.product(*adjacency):
            if u == v or u in closed:
                continue
            c = base + graph.edge_cost(v, u)
            if c < g_cost.get(u, float("inf")) - 1e-12 and graph.edge_free(v, u):
                g_cost[u] = c
                parent[u] = v
                heapq.heappush(heap, (c + graph.h(u), next(tick), u))
    result.iterations = expansions
    result.elapsed = time.perf_counter() - t0
    result.collision_checks = graph.collision_checks
    return result
