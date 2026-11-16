# Multi-arm task allocation / scheduling coupled with motion planning in a shared workspace — how inter-arm conflict cost is modelled

Research date: 2026-11-16. Scope: 2016–2026, emphasis 2019–2026. "Verified" = read from the paper's own full text (PDF/HTML) in this session; "abstract only" = from the arXiv/publisher abstract page; "snippet only" = from a search-engine snippet of the publisher page (full text was blocked) and should be re-checked.

## Q0. Annotated paper set (15 entries)

### Takeaway
Conflict models used in the literature fall into five families: (a) 2D geometric zones/hitboxes (Oberthür 2025); (b) voxel/incidence-cube occupancy from simulated trajectories, time-slotted (El Ghazi 2024/2026, Chen 2022); (c) precomputed pairwise coordination diagrams or swept volumes (Kimmel & Bekris 2016, Google/X patent); (d) a multi-robot motion planner in the loop that evaluates every candidate assignment (Hartmann & Toussaint 2023, cuArms 2026); (e) learned predictors or end-to-end learned policies (Li et al. 2024, RoboBallet 2025, MultiPnP, Jermann 2024). Almost all allocation is offline. No paper validates a cheap conflict proxy against the delay that a closed-loop multi-robot planner actually produces.

### Cited Findings

**1. Oberthür, El Ghazi, Laurent, Cardin — "Scheduling of Pick-and-Place Tasks in Shared Workspaces Using a Trajectory-Free Approximation Method", IFAC-PapersOnLine 59-10 (2025) 1384–1389, DOI 10.1016/j.ifacol.2025.09.233** — [PDF (ORBi UMONS, "Version publiée")](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf) (verified, full text)
- Summary: the paper addresses "four heterogeneous robot arms operating within a limited shared workspace, which was introduced by El Ghazi et al. (2024)". El Ghazi's MILP uses trajectory simulations and gets low makespans "at the cost of high computational time". This paper proposes an approximation that uses "only … the durations of the trajectories and the relative positions of robots and objects", with hitboxes that predict collisions. [PDF](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- Conflict model: the work is in 2D. Trajectories (r1,o1) and (r2,o2) conflict if (1) the segments robot→object cross (tensor C), (2) o2 lies in the hitbox of (r1,o1), or (3) o1 lies in the hitbox of (r2,o2). The elliptic hitbox has its foci at the robot base and the object, with the focus-to-vertex distance d_E as its parameter. A parabolic hitbox is also tested. The hitbox zone "is reserved for all the duration of the task", and robots "cannot stop in their trajectory (only between two trips)". [PDF](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- Allocation: **offline** MILP ("unrelated-machine single-stage job scheduling") minimising C_max, solved with Gurobi 10. Each object goes to exactly one robot (x_ro). Precedence variables a_oo′ are big-M disjunctive. Constraints (7)/(8) forbid concurrent execution of conflicting pairs. [PDF](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- Validation: the hitbox is **only scored against El Ghazi's coarse simulation** (workspace cut into q=6 incidence cubes per dimension, 100 ms steps, ROS-computed fastest trajectories). Best ellipse d_E*=0.162: F1 0.931, precision 0.914, recall 0.949. Parabola d_P*=0.232: F1 0.926, P 0.920, R 0.934. The schedules were **not executed**. The authors list "the schedules returned by our model could be tested with El Ghazi's simulations" as future work. They also note that the voxel reference over-detects collisions, so the hitbox is "a farther approximation of the reality". [PDF](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- Instances: 4 robots, m = 4/6/8/10 objects, workspace side Δ = 0.55 ("sparse") or 0.35 ("tight"). The paper prints "cm", which is almost certainly a typo for m (inference). 10 instances per cell. [PDF](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- Metrics: build+solve time for the hitbox model M_S is always ≤ 0.12 s, against 786–8004 s for El Ghazi's M_EG (1 h solver cap). Average RPD of makespan vs M_EG: −59 % (m=4 tight), −30 % (m=4 sparse), −68 %/−26 % (m=6), −16 %/−9 % (m=8), **+16 %/+17 % (m=10)**. Here, positive means the hitbox model is better, because M_EG's duality gap reaches 77–80 % at m=10. [PDF](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)

**2. El Ghazi, Subrin, Levilly, Mouchère, Cardin — "Task Allocation and Motion Planning Strategies for Multi-robot Cooperation", Springer Studies in Computational Intelligence vol. 1136 (2024), DOI 10.1007/978-3-031-53445-4_32** — [Springer](https://link.springer.com/chapter/10.1007/978-3-031-53445-4_32), [ResearchGate](https://www.researchgate.net/publication/377950937_Task_Allocation_and_Motion_Planning_Strategies_for_Multi-robot_Cooperation) (snippet only, publisher page blocked)
- Per the search snippet, the chapter proposes a *predictive* approach (simulation-optimisation that anticipates collisions during allocation) and a *reactive* approach (multi-agent control), with a case study for industrial robots. It also mentions real-time reallocation under intermittent robot failures. Affiliation is LS2N / Nantes. Exact venue: the snippet indicates the SOHOMA/IWSOHMAS proceedings (not verified). [Springer](https://link.springer.com/chapter/10.1007/978-3-031-53445-4_32)
- Oberthür 2025 cites the MILP baseline as El Ghazi, Cardin, Laurent, Subrin, Levilly, Mouchère (2024), "Optimizing Task Allocation and Movement Coordination in Autonomous Robotic Systems", SSRN 4964992. [PDF ref list](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf). The SSRN page could not be fetched.

**3. El Ghazi, Laurent, Subrin, Levilly, Mouchère, Cardin — "High-level multi-robot coordination in a shared workspace for pick-and-place operations using MILP optimization and heuristic for conflict resolution", Robotics and Autonomous Systems (Elsevier), 2026 (PII S0921889026001387)** — [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S0921889026001387) (snippet only: HTTP 403 on fetch. This is most likely the journal version of SSRN 4964992, an inference)
- Per the snippets: a MILP jointly optimises allocation, sequencing and motion coordination. The scalable "Detect-and-Pause" heuristic splits into MILP-based allocation, SPT-rule sequencing, and iterative conflict resolution that inserts pauses without changing the geometric paths. Both methods use a **voxel-based occupancy matrix** of the shared workspace. Validation is in **ROS-Gazebo** on 240 instances with "100% collision-free success", and robots operate simultaneously 89.4 % of the time. [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S0921889026001387)
- This is the main 2025–2026 follow-up of the UMONS/LS2N line. It is still offline and voxel-based, as far as the snippet shows.

**4. Hartmann & Toussaint — "Towards computing low-makespan solutions for multi-arm multi-task planning problems", ICAPS 2023 PlanRob workshop, arXiv 2305.17527** — [arXiv](https://arxiv.org/abs/2305.17527), [PDF](https://arxiv.org/pdf/2305.17527), [code](https://github.com/vhartman/planrob-23-low-makespan) (verified, full text)
- Summary: greedy descent with random restarts over task assignment *and* sequence. Every candidate sequence is evaluated by a multi-agent path planner that plans robot/task pairs one by one with **time-embedded ST-RRT\***, treating earlier paths as fixed (prioritised planning), plus an "escape" path to a non-blocking pose. [PDF](https://arxiv.org/pdf/2305.17527)
- Conflict model: **motion planner in the loop**, with no proxy. An optimistic makespan lower bound is used for early stopping, and the authors say it "tends to be optimistic, and does not help much". [PDF](https://arxiv.org/pdf/2305.17527)
- Offline. Up to 4 arms. Scenarios: stippling and bin picking. Videos are "in simulation and on real robots".
- Planning time per candidate sequence: ~180 s (2-arm grid), 543 s (4-arm grid, 33 s/task), ~110 s for bin picking (6 tasks). Total search runs for hours.
- Makespan: up to 30 % lower than greedy alternating assignment, and up to 50 % lower than a single robot. When arms "completely block each other" (small LIS logo), a single arm beats two arms. [PDF](https://arxiv.org/pdf/2305.17527)

**5. Hartmann, Orthey, Driess, Oğuz, Toussaint — "Long-Horizon Multi-Robot Rearrangement Planning for Construction Assembly", IEEE T-RO 39(1):239–252, 2023, DOI 10.1109/TRO.2022.3198020** — [arXiv 2106.02489](https://arxiv.org/abs/2106.02489) (abstract only)
- Decomposes long-horizon multi-robot TAMP into subproblems over subsets of robots and tasks. Uses time-embedded optimisation for manipulation constraints plus a bidirectional space-time path planner, and synchronises the subproblems. Plans were executed on **two real arms**. The abstract names choosing the right task assignment as a core difficulty. [arXiv](https://arxiv.org/abs/2106.02489)

**6. Chen, Li, Huang, Garrett, Sun, Fan, Hofmann, Mueller, Koenig, Williams — "Cooperative Task and Motion Planning for Multi-Arm Assembly Systems", arXiv 2203.02475 (2022)** — [arXiv](https://arxiv.org/abs/2203.02475), [HTML](https://arxiv.org/html/2203.02475) (verified, HTML). Venue is not stated on arXiv; Oberthür cites it only as "Chen et al. (2022)".
- High level: a MILP assigns robots to tasks under precedence. Low level: the MAPF solver PBS-AT plans collision-free roadmap motions. [HTML](https://arxiv.org/html/2203.02475)
- Conflict model: **precomputed pairwise collision annotation** over per-robot roadmaps. Each roadmap edge (swept area), waiting vertex and object occupancy is checked pairwise. The MILP uses only the annotated collisions of mode-changing edges (3–22 % of all annotations). Edge durations come from maximum joint velocities. Annotation is the dominant cost, "up to about 1000 hours of compute" on AWS. [HTML](https://arxiv.org/html/2203.02475)
- Strict hierarchy with no feedback from MAPF to the MILP. Offline. 2–3 KUKA KR6-R900 arms, heterogeneous (gripper/suction), up to 23 objects, **simulation only**. MILP first-feasible solution in 3–90 s. No numeric makespan comparison is reported. [HTML](https://arxiv.org/html/2203.02475)

**7. Li, Zhang, Dai, Huang, Hu, Chen, Xu — "Synchronized Dual-arm Rearrangement via Cooperative mTSP", arXiv 2403.08191 (Mar 2024)** — [arXiv](https://arxiv.org/abs/2403.08191), [HTML](https://arxiv.org/html/2403.08191v1) (verified, HTML; no venue on arXiv)
- Formulates the problem as a cooperative mTSP. An attention-based RL (PPO) scheduler takes a task-state graph plus a **cooperative cost matrix** over the joint actions of the two arms. A learned **cost predictor** (3-layer 512-d MLP trained on 10⁶ random transit/transfer tasks) replaces trajectory planning, with >98 % mask accuracy and <0.5 % cost error. [HTML](https://arxiv.org/html/2403.08191v1)
- Conflict model: infeasible joint actions are masked, and a fixed 0.1 m safety rule stops both arms from picking or placing closer than 0.1 m. **Waiting between arms is not explicitly modelled.** Two Kawasaki RS050N arms, synchronous and monotone, PyBullet, no real robot. [HTML](https://arxiv.org/html/2403.08191v1)
- Metrics are cumulative cost (sum of transit and transfer times), not makespan. n=10: cost 27.29 (exhaustive 25.81, perfect matching 28.87). Planning 6.27 vs 49.2 (units not given). Using Euclidean distance instead of the predictor raises cost to 32.45. [HTML](https://arxiv.org/html/2403.08191v1)

**8. Kimmel & Bekris — "Scheduling Pick-and-Place Tasks for Dual-arm Manipulators using Incremental Search on Coordination Diagrams", ICAPS PlanRob 2016** — [PDF](https://people.cs.rutgers.edu/~kb572/pubs/kimmel_schedule.pdf) (verified, abstract and intro; seminal, pre-2019)
- A "batch" mode chooses the order first, then velocity-tunes over a matrix of **coordination diagrams for all pairs of pick-and-place trajectories**. An "incremental" mode discovers the order on the fly and "can be used effectively when objects appear dynamically". Each arm has its own object set, so this is sequencing, not allocation. Coordination diagrams can be precomputed. Simulated Baxter. [PDF](https://people.cs.rutgers.edu/~kb572/pubs/kimmel_schedule.pdf)
- Oberthür 2025 notes that this method needs coordination-diagram computation "often exceeding one hour". [PDF](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)

**9. Shome, Solovey, Yu, Bekris, Halperin — "Fast, High-Quality Two-Arm Rearrangement in Synchronous, Monotone Tabletop Setups", IEEE T-ASE 18(3):888–901, 2021** — [arXiv 1810.12202](https://arxiv.org/abs/1810.12202), [NSF PAR](https://par.nsf.gov/biblio/10219059) (record/abstract only)
- An optimal MIP model plus a scalable method that resolves transfer costs first and uses a lazy strategy to cut motion-planning calls. Synchronous two-arm execution adds only small overhead. Near-optimal solutions are found quickly. [record](https://katalog.lib.cas.cz/KNAV/EdsRecord/edsair,edsair.doi...........200555246d71cf68699382a2dfd37648)

**10. Gao & Yu (Rutgers) — dual-arm tabletop rearrangement: "Toward Efficient Task Planning for Dual-Arm Tabletop Object Rearrangement" (arXiv 2207.08078, 2022) and MODAP "Toward Holistic Planning and Control Optimization for Dual-Arm Rearrangement" (Gao, Ye, Zhang, Huang, Yu, arXiv 2404.06758, 2024)** — [2207.08078](https://arxiv.org/pdf/2207.08078), [2404.06758](https://arxiv.org/abs/2404.06758) (abstract only)
- 2022: dual arms with partially overlapping workspaces, minimising makespan, with lazy buffer allocation. 2024: task-planning sampling is tightly integrated with trajectory optimisation that respects acceleration and jerk limits, on "a real dual-arm system", with "significantly improved execution time" (no numbers on the abstract page). [2404.06758](https://arxiv.org/abs/2404.06758)

**11. Behrens, Lange, Mansouri — "A Constraint Programming Approach to Simultaneous Task Allocation and Motion Scheduling for Industrial Dual Arm Manipulation Tasks", ICRA 2019, pp. 8705–8711, DOI 10.1109/ICRA.2019.8794022** — [arXiv 1901.07914](https://arxiv.org/abs/1901.07914) (abstract only)
- A CP model (Google OR-Tools + ROS) that integrates task and robot motion models. "Ordered Visiting Constraints" specify tasks. The solver produces makespan-optimised programs and computes usable plans for 200 objects in under a minute. Offline. The conflict model is not in the abstract (gap). [arXiv](https://arxiv.org/abs/1901.07914)

**12. Lai, Go, Li, Kröger, Schaal, Allen, Scholz — "RoboBallet: Planning for Multi-Robot Reaching with Graph Neural Networks and Reinforcement Learning", Science Robotics 10, eads1204 (Sep 2025)** — [arXiv 2509.05397](https://arxiv.org/abs/2509.05397v1), [DeepMind](https://deepmind.google/research/publications/roboballet-planning-for-multi-robot-reaching-with-graph-neural-networks-and-reinforcement-learning/) (abstract only)
- A GNN policy trained with RL jointly solves allocation, scheduling and motion for **8 arms and 40 reaching tasks** in a shared workspace with obstacles, and generalises zero-shot. This is reaching only, not pick-and-place. Conflict avoidance is learned implicitly. [arXiv](https://arxiv.org/abs/2509.05397v1)

**13. Huang, Ramos, Li, Garrett, Johannsmeier — "cuArms: GPU-Accelerated Conflict-Based Planning for Multi-Arm Manipulation", arXiv 2610.36250 (26 Oct 2026)** — [arXiv](https://arxiv.org/abs/2610.36250), [HTML](https://arxiv.org/html/2610.36250v1) (verified, HTML; very recent, no venue)
- Uses per-arm reusable multi-modal roadmaps, a GPU-computed **Boolean inter-robot conflict table** (sphere models, bounding volumes for skills), and LaCAM/PIBT-style lazy search. A high-level scheduler proposes skeletons: for RoboBallet it is greedy Hungarian matching on relaxed travel distance; for tabletop TAMP it is ScheduleStream. It backtracks over up to 16 skeletons. [HTML](https://arxiv.org/html/2610.36250v1)
- RoboBallet benchmark: 4 arms/10 targets, t0 0.20 s, final makespan 5.64 s; 8 arms/40 targets, t0 0.87 s, 14.87 s; 150/150 solved, where the RL RoboBallet policy solved 29/150 and 50/150. Two Panda arms in tabletop TAMP. Real system: two Flexiv Rizon4, qualitative only. **The allocation cost itself is conflict-agnostic.** Conflicts are resolved downstream by the planner. [HTML](https://arxiv.org/html/2610.36250v1)

**14. "MultiPnP: Learning Task Allocation, Scheduling, and Motion Planning for Multi-Robot Pick-and-Place" (anonymous, under review; venue unknown)** — [project page](https://multipnp.github.io/) (project page only, no arXiv ID found)
- Centralised GNN RL policy with two-stage hindsight experience replay, plus an IK "docking" controller for the final grasp. **4 arms, 8 objects, MuJoCo**, and a physical 4-arm cell. 92.5 % success in sim, ~15 s per scene vs 53 s for a serial planner (3.5×). The parallel planner baseline shows more "blocked motions". Real cell: 92 % of 50 scenes cleared, 97.9 % of 480 objects placed, zero executed collisions. Generalises to 2–4 robots and 12–32 objects. Sorting needs fine-tuning. [project page](https://multipnp.github.io/)

**15. Jermann, Kolvenbach, Esquivel Estay, Kramer, Hutter (ETH) — "An Efficient Multi-Robot Arm Coordination Strategy for Pick-and-Place Tasks using Reinforcement Learning", arXiv 2409.13511 (Sep 2024)** — [arXiv](https://arxiv.org/abs/2409.13511) (abstract only)
- Online waste sorting on a stream of incoming objects. A deep RL allocator in a Gym environment gets up to 16 % higher picking rate than a combinatorial-game-theory baseline in simulation. Hardware tests on a 2-robot station. The conflict model is not in the abstract. Per a search snippet, the observations include each agent's busy flag and the estimated time until it is free. [arXiv](https://arxiv.org/abs/2409.13511), [HTML](https://arxiv.org/html/2409.13511v1)

**Also relevant (shorter notes):**
- Guo, Kingston, Hang, Kavraki — "Efficient Multi-Robot Motion Planning for Manifold-Constrained Manipulators by Randomized Scheduling and Informed Path Generation" (StAC), IEEE RA-L 11(4):4385–4392, 2026, DOI 10.1109/LRA.2026.3662639. This is **motion planning only, with no task allocation**. It is a hybrid MRMP that composes individual paths by scheduling (adding stops and coordination motion) with bidirectional scheduler↔planner feedback. It uses "10 to 100 times fewer paths" than hybrid baselines and notes that "dRRT often scales poorly". This is probably the "Guo et al. RA-L 2026" the brief meant: its title says "randomized scheduling", not "conflict-aware scheduling". [Kavrakilab](https://www.kavrakilab.org/publications/weihang-manifoldpath2026.html), [PDF](http://kavrakilab.org/publications/weihang-manifoldpath2026.pdf) (verified, abstract and intro)
- Toumi, Orthey, von Rohr, Vien — "Multi-Arm Bin-Picking in Real-Time: A Combined Task and Motion Planning Approach", arXiv 2211.11089 (2022). A geometry-aware policy decides which object to remove online, and BIT* plans the motion. Two arms, scaled to four. The abstract reports a 28 % improvement over the baseline (the abstract's wording is "increasing bin-picking time by 28%"). [arXiv](https://arxiv.org/abs/2211.11089)
- US 11,216,009 B2 "Robot coordination in a shared workspace" (X Development → Intrinsic; inventor J.-F. Dupuis; filed 2019-06-25, granted 2022-01-04). Tasks are clustered and assigned to robots (approximate TSP). Several candidate motion plans per robot (RRT/PRM) each get a **3D swept-volume model**. Plans are scored on duration, energy, swept volume and a congestion cost map. Swept-volume intersections between robots flag conflicts, and colliding plans are pruned. The claim language was not read (only the description). [Google Patents](https://patents.google.com/patent/US11216009B2/en)
- DG-MAP "Diffusion-Guided Multi-Arm Motion Planning", arXiv 2509.08160: 4 UR5 arms in a shared bin, 89 % success over 100 trials. Motion only, no allocation (per a search snippet). [arXiv](https://arxiv.org/pdf/2509.08160)

### Inferences
Summary table (O = offline, On = online; "exec" = how the result is validated):

| # | Paper | Conflict / coordination-cost model | Alloc. | Arms | Exec. validation | Metrics |
|---|---|---|---|---|---|---|
| 1 | Oberthür 2025 IFAC | 2D elliptic/parabolic hitbox + segment crossing; zone reserved for whole task | O (MILP) | 4 heterogeneous | none; F1 vs coarse voxel sim | makespan RPD, solve time |
| 2–3 | El Ghazi 2024 / RAS 2026 | voxel incidence cubes × 100 ms time slots (time-window reservation); Detect-and-Pause | O | 4 heterogeneous | ROS-Gazebo (2026, snippet) | makespan, collisions, solve time |
| 4 | Hartmann & Toussaint 2023 | ST-RRT* prioritised planner in the loop | O (local search) | ≤4 | sim + real videos | makespan vs comp. time |
| 5 | Hartmann et al. T-RO 2023 | space-time planner in the loop, subproblem decomposition | O | many; 2 real | real arms | scalability |
| 6 | Chen et al. 2022 | precomputed pairwise swept-edge collision annotation → MILP | O | 2–3 KUKA | sim only | solve time |
| 7 | Li et al. 2024 | learned cost and feasibility predictor; 0.1 m rule; no waiting | O (RL policy, replans) | 2 | PyBullet | cumulative cost, plan time |
| 8 | Kimmel & Bekris 2016 | precomputed pairwise coordination diagrams | On (incremental sequencing) | 2 (Baxter) | sim | makespan, comp. time |
| 9–10 | Shome 2021; Gao & Yu 2022/24 | synchronous/monotone MIP, lazy MP; trajectory optimisation | O | 2 | real dual-arm (MODAP) | makespan/cost |
| 11 | Behrens 2019 | CP with motion models (detail not verified) | O | 2 | ROS | makespan, solve time |
| 12 | RoboBallet 2025 | implicit (learned GNN policy) | On (policy) | 8 | sim (+ real per press, unverified) | success, makespan |
| 13 | cuArms 2026 | GPU conflict table + lazy search; scheduler conflict-agnostic | O / anytime | 2–8 | sim + real (qual.) | t0, makespan, solve rate |
| 14 | MultiPnP (anon.) | implicit (learned GNN policy) | On (policy) | 4 | MuJoCo + real 4-arm cell | success, time/scene |
| 15 | Jermann 2024 | learned RL allocator (busy/time-to-free features) | On | 2 | sim + 2-robot hardware | picking rate |

- The only explicitly *geometric, trajectory-free* conflict proxy in the set is Oberthür's hitbox. Its quality was measured only against another approximation (6×6×6 voxels), never against executed motions. The user's own measurement (F1 0.888 vs exact collision of executed reach trajectories) therefore goes beyond what the paper reports.
- MultiPnP's setup (4 arms, 8 objects, MuJoCo, sorting) is almost identical to the user's cell. It is the closest competitor and the most likely reviewer comparison.

### Gaps
- Exact venue of Chen et al. 2203.02475 is unverified (arXiv lists none).
- El Ghazi RAS 2026 full text was not accessible (HTTP 403). Robot types, online/offline status and numbers beyond the snippet are unverified.
- MultiPnP authors, venue and arXiv ID are unknown (anonymous project page).
- Behrens 2019 conflict model: not read beyond the abstract.
- Google Scholar "cited by" for Oberthür 2025 / El Ghazi 2024 could not be queried. Apart from El Ghazi RAS 2026, no other 2025–2026 citing work was found.

## Q1. El Ghazi et al. 2024 and Oberthür et al. 2025: exact problem, model, validation, follow-ups

### Takeaway
The problem: a fixed set of objects in a shared square workspace, 4 heterogeneous arms, each object picked by exactly one arm into its own drop-off, minimise makespan. It is solved **offline** by MILP. The hitbox was validated only statistically (F1 vs a coarse voxel simulation). Hitbox schedules were never executed. The 2026 RAS follow-up stays voxel-based and offline, with ROS-Gazebo validation.

### Cited Findings
- Problem data: robot positions, the object set, and for every (r,o) "the fastest trajectory for which r picks up o and places it in its drop-off area", computed with ROS. Unreachable pairs get t_ro = +∞. — [Oberthür 2025](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- El Ghazi's MILP: each trajectory is cut into t-ms steps and assigned to time slots. Two steps at the same time may not share an incidence cube. Robots may pause mid-trajectory. Building and solving often takes "more than an hour". — [Oberthür 2025](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- Oberthür explains the hitbox shape choice with visualised trajectories: "No pattern seems to emerge … sometimes the robot makes a curve to the left … to the right … or goes straight". Hence the zone has to cover widely around the robot→object segment. — [Oberthür 2025](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- Stated limitation and future work: "our approximation (the hitboxes) is therefore a farther approximation of the reality … the schedules returned by our model could be tested with El Ghazi's simulations". — [Oberthür 2025](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- 2026 follow-up: MILP + Detect-and-Pause, voxel occupancy matrix, ROS-Gazebo, 240 instances, 100 % collision-free, 89.4 % simultaneous operation (snippet). — [RAS 2026](https://www.sciencedirect.com/science/article/pii/S0921889026001387)

### Inferences
- Oberthür's model is more conservative than reality in one respect (no mid-trajectory pauses; the zone is blocked for the whole task) and more optimistic in another (2D; recall < 1). So the makespan its MILP predicts is not the makespan a reactive planner would achieve. The user's observation fits this: with dRRT* and event-driven replanning, conflicts cost only ~0.1 s waits, so the hitbox's binary "cannot run concurrently" penalty mis-prices coordination.
- In a sparse layout, the hitbox's high F1 for *collision* prediction does not imply good *delay* prediction. A collision that a planner resolves with a 0.1 s wait and one that forces a 3 s detour look the same to a binary classifier. This distinction is not made anywhere in the UMONS/LS2N line.

### Gaps
- Whether El Ghazi 2026 evaluates the hitbox model inside Gazebo (i.e. closes Oberthür's future-work item) could not be verified.
- No other 2025–2026 citers were found (Scholar not accessible).

## Q2. Hartmann & Toussaint: how assignment and motion planning interact

### Takeaway
Hartmann uses the planner as the cost oracle. Every candidate assignment and sequence is fully planned with prioritised ST-RRT*, which is accurate but takes minutes per candidate and runs offline. The authors explicitly ask for a better task-time lower bound and for "more informed ways of generating candidate sequences that take into account the geometry".

### Cited Findings
- Greedy descent + random restarts over sequences. Neighbour moves are TSP-style mutations. Each sequence is evaluated by PlanGivenSequence (prioritised, time-embedded ST-RRT\*, escape poses). — [arXiv 2305.17527](https://arxiv.org/pdf/2305.17527)
- 33 s per task in the 4-arm grid. "planning time increases considerably, as the space becomes more occupied by the robots". — [arXiv 2305.17527](https://arxiv.org/pdf/2305.17527)
- The benefit of multiple arms is larger when tasks parallelise. When arms "completely block each other", a single arm is faster. — [arXiv 2305.17527](https://arxiv.org/pdf/2305.17527)
- Future work: "a more accurate lower bound for the time it takes to complete a task", and "more informed ways of generating candidate sequences that take into account the geometry of the scene". — [arXiv 2305.17527](https://arxiv.org/pdf/2305.17527)
- T-RO 2023: decomposition into robot/task subsets, with space-time planning and synchronisation. Executed on 2 real arms. — [arXiv 2106.02489](https://arxiv.org/abs/2106.02489)

### Inferences
- Hartmann's result that the gain depends on the regime (parallelisable vs mutually blocking) is the closest prior evidence for Angle A's "in which regimes" question. It is anecdotal, though: a few scenarios, offline, with no proxy evaluated.
- A planner-grounded but cheap coordination cost (precomputed from the roadmap) is exactly the "more accurate lower bound / geometry-informed" component Hartmann lists as future work.

### Gaps
- Hartmann's later work (2024–2026, e.g. on multi-robot multi-modal planning) was not surveyed in this session.

## Q3. Other allocation + motion planning works (Chen 2022, Li 2024, Guo 2026, Behrens 2019, Shome/Bekris, Gao/Yu, Pan & Kavraki)

### Takeaway
The dual-arm rearrangement line (Shome/Bekris, Gao/Yu, Li et al.) assumes synchronous or monotone execution and models conflict as feasibility masks or MIP constraints, not delay. Chen 2022 precomputes swept-edge collisions offline at huge cost. Guo 2026 (StAC) is motion planning only. Nothing here is online allocation for more than 2 arms with a validated delay model.

### Cited Findings
- Chen 2022: pairwise collision annotation over roadmaps feeds the MILP, which never gets feedback from MAPF. ~1000 CPU-h annotation. Sim only. — [arXiv HTML](https://arxiv.org/html/2203.02475)
- Li 2024: learned cost and feasibility predictor, waiting not modelled, 0.1 m safety rule, PyBullet. — [arXiv HTML](https://arxiv.org/html/2403.08191v1)
- Guo 2026 StAC: scheduling = inserting stops and coordination motion into individual paths, with scheduler↔planner feedback. No allocation. — [PDF](http://kavrakilab.org/publications/weihang-manifoldpath2026.pdf)
- Behrens 2019: CP simultaneous allocation + motion scheduling, 200 objects in <1 min. — [arXiv](https://arxiv.org/abs/1901.07914)
- Shome 2021: MIP + lazy MP, synchronous two-arm. — [arXiv](https://arxiv.org/abs/1810.12202)
- Gao/Yu 2022/2024: makespan-optimised dual-arm planning, real dual-arm system (MODAP). — [arXiv](https://arxiv.org/abs/2404.06758)

### Inferences
- Synchronous, monotone dual-arm models side-step the asynchronous "who frees up when" online setting of a 4-arm cell with event-driven replanning.

### Gaps
- "Pan & Kavraki on multiple manipulators": no matching paper was found in this session. The closest Kavraki-lab item is Guo et al. RA-L 2026 (motion only). It needs a targeted search if it matters.
- A general MRTA-with-motion-planning survey was not located. A TechRxiv 2025 multi-robot collaboration survey appeared in results but was not read: [TechRxiv](https://www.techrxiv.org/doi/pdf/10.36227/techrxiv.176045766.60277537/v2).

## Q4. Learned or precomputed pairwise conflict / delay from planner data used in scheduling; patents

### Takeaway
Precomputed pairwise geometric conflicts exist: coordination diagrams (Kimmel 2016), roadmap collision annotations (Chen 2022), swept volumes (US 11216009), and a GPU conflict table (cuArms 2026). Li et al. 2024 learns a cost and feasibility predictor. **No work found learns or precomputes the expected *delay* (waiting/detour time) between two arm tasks under a given multi-robot planner and uses it as an allocation cost.** All existing items predict binary conflict or single-arm cost.

### Cited Findings
- Kimmel & Bekris: coordination diagrams "for all pairs of pick-and-place trajectories". Velocity tuning resolves collisions. They can be precomputed. — [PDF](https://people.cs.rutgers.edu/~kb572/pubs/kimmel_schedule.pdf)
- US 11216009: swept-volume intersection between candidate plans of different robots, plus a congestion cost map used in plan scoring. — [Google Patents](https://patents.google.com/patent/US11216009B2/en)
- cuArms: a Boolean conflict table on GPU. The scheduler uses relaxed travel distance, so it is conflict-agnostic. — [arXiv HTML](https://arxiv.org/html/2610.36250v1)
- Li 2024: predictor trained on 10⁶ planned transit/transfer tasks. Euclidean distance instead of the predictor gives worse cost (32.45 vs 27.29). — [arXiv HTML](https://arxiv.org/html/2403.08191v1)

### Inferences
- A coordination diagram is in principle a pairwise delay oracle: the minimum delay through the CD is the coordination cost. Kimmel used it for velocity tuning and sequencing for 2 arms, not as an allocation cost for N arms, and not against a composite-space planner like dRRT*. Angle A must cite it and position itself against it.

### Gaps
- The patent search was limited to US 11216009. Another USPTO hit, "Motion planning for multiple robots in shared workspace" (US 12358140), appeared but was not read: [USPTO](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12358140).

## Q5. Works comparing allocation strategies inside a closed loop with a real multi-robot planner and physics

### Takeaway
Few exist. MultiPnP (MuJoCo plus a real 4-arm cell) compares a learned joint policy against serial and parallel planners, not allocation cost models. Hartmann compares greedy vs search with the planner in the loop, offline. Jermann compares RL vs game-theory allocators with hardware tests. El Ghazi 2026 validates in Gazebo. None ablates the *conflict-cost term* of an online allocator in closed loop.

### Cited Findings
- MultiPnP: 15 s vs 53 s (serial planner). The parallel planner has more blocked motions. Real: 97.9 % of 480 objects placed, 0 collisions. — [project page](https://multipnp.github.io/)
- Hartmann: greedy vs search, up to 30 % makespan gain. — [arXiv](https://arxiv.org/pdf/2305.17527)
- Jermann: RL allocator +16 % picking rate vs combinatorial baseline (sim). Hardware on 2 robots. — [arXiv](https://arxiv.org/abs/2409.13511)
- El Ghazi 2026: ROS-Gazebo, 240 instances (snippet). — [RAS](https://www.sciencedirect.com/science/article/pii/S0921889026001387)

### Inferences
- The specific experiment "same online greedy allocator, with conflict term ∈ {none, hitbox, planner-grounded, learned}, executed with a composite multi-robot planner in physics, swept over density / object count / object proximity" was not found in any source.

### Gaps
- RoboBallet's real-robot results and any allocation ablations were not read (abstract only).

## Q6. Novelty judgement for Angle A

### Takeaway
**Partially open.** The ingredients exist: planner-in-the-loop allocation (Hartmann), precomputed pairwise conflicts (Kimmel, Chen, the patent, cuArms), a learned cost predictor (Li 2024), learned end-to-end policies (RoboBallet, MultiPnP), and the hitbox (Oberthür). But no published work (i) measures how well a geometric conflict proxy predicts the *actual coordination delay* under a closed-loop multi-robot planner, or (ii) maps the regimes where coordination-aware *online* allocation pays off in throughput. Oberthür et al. explicitly leave execution-based validation as future work. Hartmann leaves "more accurate task-time bounds / geometry-informed sequences" as future work.

### Cited Findings
- Hitbox validated only vs a coarse voxel sim, with execution-based testing left as future work. — [Oberthür 2025](https://orbi.umons.ac.be/bitstream/20.500.12907/55073/1/Version%20publi%C3%A9e.pdf)
- Regime dependence (parallelisable vs blocking) is observed but not characterised. Planner-in-the-loop costs minutes per candidate. — [Hartmann 2023](https://arxiv.org/pdf/2305.17527)
- Learned cost predictor exists for 2 synchronous arms, without waiting time. — [Li 2024](https://arxiv.org/html/2403.08191v1)
- The scheduler in the newest multi-arm planner (cuArms) uses conflict-agnostic relaxed travel distance and leaves conflicts to the planner. — [cuArms](https://arxiv.org/html/2610.36250v1)
- Nearly identical cell (4 arms, 8 objects, MuJoCo, real) solved end-to-end with RL. — [MultiPnP](https://multipnp.github.io/)

### Inferences
- **Already done:** coordination-aware allocation in general; offline planner-in-the-loop assignment; binary pairwise conflict precomputation; learned pick/transfer cost for dual-arm.
- **Partially done:** online incremental scheduling with precomputed pairwise coordination diagrams (Kimmel 2016, 2 arms, fixed partition, no allocation); learned allocation policies that implicitly price conflict (MultiPnP, RoboBallet), which are black boxes and do not explain *when* conflict-awareness matters.
- **Open (defensible contribution):** (1) A quantitative study showing that collision-prediction accuracy (F1) of geometric proxies does not translate into delay prediction or throughput when a composite planner such as dRRT* with replanning resolves most conflicts with short waits. The user's ~0.1 s waits and null throughput gain are the first data point. (2) A planner-grounded pairwise *delay* cost, e.g. precomputed from roadmap/dRRT* coordination or learned from logged executions, as a regression target rather than a binary label. (3) A regime map (table density, object count, inter-object distance, arm reach overlap) showing where any conflict term changes throughput.
- **Risks:** a pure negative result ("hitbox does not help") is weak unless the regime map shows where it *does* help. Reviewers will expect baselines: conflict-agnostic greedy, hitbox, planner-in-the-loop (Hartmann-style, at least offline as an upper bound), a learned predictor (Li-style), and ideally a comparison or discussion against MultiPnP/RoboBallet-type learned policies. The 1.3 m table with 8 cubes may sit in the "sparse" regime where Oberthür's tight/sparse distinction (Δ 0.35 vs 0.55) already shows different behaviour. Dense layouts must be added to see any effect.

### Gaps
- Unpublished or concurrent work (e.g. MultiPnP's final version, or any El Ghazi 2026 follow-up that runs hitbox schedules in Gazebo) could close part of the gap. Check before submission.
- No Scholar citation crawl of Oberthür 2025 was possible. This should be done manually.
