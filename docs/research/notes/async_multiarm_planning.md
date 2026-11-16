# Asynchronous / event-driven / dynamic motion planning and replanning for multiple arms sharing a workspace (dRRT*-family and composite roadmaps)

Research done 2026-11-16. Sources: arXiv abstract pages, arXiv HTML full text, PDF text extraction (pdftotext on the downloaded PDFs) and one workshop PDF. Each claim below is tagged with the paper it comes from. Anything I only saw in another paper's reference list or in a search snippet is marked **[not read]** or **[unverified]**.

Each paper card answers the brief's six questions:
- **Q1** sync/async: do arms start and finish together, or finish at different times, with replanning while other arms move?
- **Q2** replanning from moving (non-rest) states?
- **Q3** how arms doing manipulation are treated
- **Q4** carried objects modelled?
- **Q5** execution and time parameterization
- **Q6** evaluation

---

## KQ1. dRRT / dRRT* variants since 2019 (Fast-dRRT*, handoffs, MR-TAMP with dRRT*, 2026 asymptotically optimal MR-TAMP)

### Takeaway
All dRRT/dRRT*-family papers I found plan **offline, one-shot, from a static start state**. None replans online from mid-execution states, and none adds temporary vertices for the current configurations of moving arms. Two papers come closest. Fast-dRRT* (Solano et al. 2023, Realtime Robotics) handles "partial" problems in which idle, goal-less robots are moved out of the way, and uses precomputed per-edge swept volumes. Sung, Shome & Stone (ICRA 2024) extend dRRT to asynchronous multi-modal plans with held objects.

### Cited Findings

**P1. dRRT* (the base method)**
- Citation: R. Shome, K. Solovey, A. Dobson, D. Halperin, K. E. Bekris, "dRRT*: Scalable and Informed Asymptotically-Optimal Multi-Robot Motion Planning," *Autonomous Robots* (online 2019; the reference lists in later papers give 2020), DOI 10.1007/s10514-019-09832-9. arXiv 1903.00994. — [arXiv](https://arxiv.org/abs/1903.00994)
- Summary: dRRT* extends the dRRT planner, which searches an implicit tensor-product of per-robot roadmaps, adding asymptotic optimality and informed (heuristic) guidance. It scales to more robots than the alternatives it was compared with. — [arXiv](https://arxiv.org/abs/1903.00994)
- Q1–Q6: I did not re-read the full text in this pass. It is a single-mode, single-query planner from a start composite configuration to a goal composite configuration, and the later papers treat it as such ([Fast-dRRT*](https://arxiv.org/abs/2309.10665); [Sung et al.](https://arxiv.org/abs/2309.08897)). Fast-dRRT* states that "dRRT* does not support deadlock avoidance" for robots without goals, and that "collision detection is computed on-the-fly". — [Fast-dRRT* PDF](https://arxiv.org/pdf/2309.10665)

**P2. Fast-dRRT***
- Citation: A. Solano, A. Sieverling, R. Gieselmann, A. Orthey, "Fast-dRRT*: Efficient Multi-Robot Motion Planning for Automated Industrial Manufacturing," arXiv 2309.10665 (Sep 2023). The arXiv comment says "submitted to ICRA 2024". Final publication status: **[unverified]**. Affiliation footnote: Realtime Robotics Inc. — [arXiv](https://arxiv.org/abs/2309.10665)
- Summary (abstract): Fast-dRRT* builds on dRRT* "by using pre-computed swept volumes for efficient collision detection, deadlock avoidance for partial multi-robot problems, and a simplified rewiring strategy". It is evaluated on five scenarios with 2–4 industrial arms. It "outperform[s] dRRT* by up to 94% in terms of finding solutions within given time limits, while only sacrificing up to 35% on initial solution cost", and solves "welding, and pick and place tasks". — [arXiv PDF](https://arxiv.org/pdf/2309.10665)
- Q1: Plans are one-shot and composite. The paper defines **partial multi-robot problems**, in which "one or more robots have no specified goal, but could impede the others". These robots get an "any-goal". The informed expansion "keeps the robots without a defined goal at their place and only moves them after an iteration fails to expand the tree". So idle robots are part of the search state as movable but goal-less, not as fixed obstacles. The paper does not address asynchronous task completion. — [PDF](https://arxiv.org/pdf/2309.10665)
- Q2: No online replanning. It shows "resilience against noise in target configurations" (goal perturbation) and reports first solutions in about 100 ms ("first solutions in 100ms with one exception at 150ms" in the perturbed-target experiment). — [PDF](https://arxiv.org/pdf/2309.10665)
- Q3: Swept volumes are precomputed **per roadmap edge** by voxelization ("approximating the swept volumes using voxels"). "An edge in the combined graph is collision free if there is no intersection between the swept volumes of the edges that compose it." This is a collision-checking speedup, not a reservation of a working arm's region. Process robots (welding guns) appear only as attached tools. — [PDF](https://arxiv.org/pdf/2309.10665)
- Q4: A welding gun is attached to the robots. The text I extracted does not say how carried parts in the pick-and-place scenario are modelled. **[unverified]**
- Q5: The extracted text does not describe time parameterization or execution. **[gap]**
- Q6: Simulation, 2–4 arms "from various manufacturers". Five scenarios: Deadlock Aisles, Deadlock Table, Narrow Passage, Welding (targets "taken from a robot cell used in an industrial car manufacturing task"), and Pick and Place. Metrics: time to first solution, cost at first solution, cost after 100 ms. Per-scenario results include 22% and 63% faster first solutions with 17% and 9% higher initial cost. — [PDF](https://arxiv.org/pdf/2309.10665)

**P3. Shome & Bekris, anytime multi-arm TAMP via handoffs**
- Citation: R. Shome, K. E. Bekris, "Anytime Multi-arm Task and Motion Planning for Pick-and-Place of Individual Objects via Handoffs," arXiv 1905.03179 (May 2019). The cuArms reference list gives the venue as MRS 2019. — [arXiv](https://arxiv.org/abs/1905.03179); venue per [cuArms refs](https://arxiv.org/html/2610.36250)
- Summary: A multi-modal extension of dRRT* that plans pick-and-place sequences with arm-to-arm handoffs. It finds initial solutions fast and improves them anytime. — [arXiv](https://arxiv.org/abs/1905.03179)
- Q1: Sung et al. cite this line of work (their ref [6], R. Shome & K. E. Bekris, "Synchronized multi-arm rearrangement…") as an example of **synchronous actions**, "where robots simultaneously initiate and complete action execution". — [Sung et al. PDF](https://arxiv.org/pdf/2309.08897). I did not read the handoff paper's own full text, so its exact Q2–Q6 answers are **[not read]**.

**P4. Sung, Shome, Stone, asynchronous task plan refinement (dRRT extended)**
- Citation: Y. Sung, R. Shome, P. Stone, "Asynchronous Task Plan Refinement for Multi-Robot Task and Motion Planning," ICRA 2024 (Yokohama), arXiv 2309.08897. Sony AI's page lists AAAI-24, while Stone's page and the ANU record list ICRA 2024. — [arXiv](https://arxiv.org/abs/2309.08897); [Stone pubs](https://www.cs.utexas.edu/~pstone/Papers/bib2html/b2hd-yoonchang_sung_ICRA2024.html)
- Summary: The paper formulates MR-TAMP plan refinement (given a task plan) as a hybrid CSP that "is inherently asynchronous and does not require" complex scheduling. It solves it by sequential decomposition: object placements, then transition configurations and grasps, then individual PRMs, then composite dRRT search. Features include "implicit time and roadmap representations" and "minimally necessary coordination-induced constraints". — [PDF](https://arxiv.org/pdf/2309.08897)
- Q1: **Asynchronous at the plan level.** The formulation "strictly generalizes the synchronous formulation". Synchronous means "all robots either leave and arrive at" transitions together. Ordering constraints (≺) between robots' abstract actions are enforced inside the dRRT search ("no adjacent edges connected to a goal configuration ... are used until another robot's roadmap, as determined by ≺, is reached"). The composite edge definition allows "some robots to remain stationary". — [PDF](https://arxiv.org/pdf/2309.08897)
- Q2: Offline. The paper has no mention of online or mid-execution replanning. dRRT has a time limit, and on failure the method backtracks to earlier subproblems. — [PDF](https://arxiv.org/pdf/2309.08897)
- Q3: Grasping robots are part of the composite search. Their transition (grasp/place) configurations are the goals of per-action PRMs. "Time is implicitly revealed as a byproduct". There are no explicit time windows or reserved regions. — [PDF](https://arxiv.org/pdf/2309.08897)
- Q4: Yes. dRRT is extended so that it "accommodates robots holding objects, which affects the collision-checking process" (Hold constraint). — [PDF](https://arxiv.org/pdf/2309.08897)
- Q5: No time parameterization is described. Makespan is reported in "simulation steps". — [PDF](https://arxiv.org/pdf/2309.08897)
- Q6: Simulation only. The example task (Fig. 1) has 3 robots. "Base poses and grasp poses are all sampled". Makespan: **5118.3 ± 148.4 (asynchronous) vs 7432.1 ± 211.8 (synchronous)** simulation steps, which is about 31% lower (my arithmetic). The paper calls the synchronous approach "impractical". The exact robot model and count across all experiments are **[unverified]**. — [PDF](https://arxiv.org/pdf/2309.08897)

**P5. Duong, Yiu, Shome, Sung, asymptotically optimal MR-TAMP (2026)**
- Citation: T. T. N. Duong, C. T. S. Yiu, R. Shome, Y. Sung, "Asymptotically Optimal Multi-Robot Task and Motion Planning," arXiv 2609.18813, 16 Sep 2026, preprint. — [arXiv HTML](https://arxiv.org/html/2609.18813)
- Summary: The paper analyses the geometry of multi-robot mode transitions, where different subsets of robots induce transition sets of different dimension. It proves global asymptotic optimality under conditions on transition coverage, persistent exploration and asymptotically improving motion within orbits. The algorithm combines evolving per-robot roadmaps, a **dRRT*-based implicit tensor-product search tree**, lazy collision checking, and mode- and orbit-level guidance. — [arXiv HTML](https://arxiv.org/html/2609.18813)
- Q1: Not explicitly sync/async. Transitions can involve "any nonempty subset of robots" and occur "at a configuration shared by the closures of two incident orbits". — [HTML](https://arxiv.org/html/2609.18813)
- Q2: No. It is an anytime offline search. — [HTML](https://arxiv.org/html/2609.18813)
- Q3/Q4: Modes are "represented by the robot–object attachment relations", for example "Robot 1 carries Object 1", so carried objects are modelled. — [HTML](https://arxiv.org/html/2609.18813)
- Q5: Trajectories are "transferred to the physical robots for execution". No time parameterization is described. — [HTML](https://arxiv.org/html/2609.18813)
- Q6: PyBullet with Franka Panda arms, 2–4 robots, plus 2 real Franka Research 3 arms. Metrics: success rate, time to first solution, initial and final cost. In the 2-robot/6-object case success was 100%, against 40% for Composed PRM*. — [HTML](https://arxiv.org/html/2609.18813)
- Its related work cites only the original dRRT* among dRRT* extensions. It cites Sung et al. 2024 and ScheduleStream (Garrett & Ramos, ICRA 2026) for asynchrony and scheduling. — [HTML](https://arxiv.org/html/2609.18813)

### Inferences
- In the dRRT* line, "partial problems / idle robots" (Fast-dRRT*) and "asynchronous ordering inside dRRT" (Sung et al.) exist, but both are **offline planners solved once from a static start**.
- Running dRRT* repeatedly online during execution, re-rooted at the current joint configurations of only the moving arms with non-moving arms frozen as obstacles, does not appear in any of P1–P5.
- Fast-dRRT*'s ~100 ms first-solution times show that latencies well below a second are achievable for dRRT*-type planners on 2–4 industrial arms. The project's 0.54 s median and 1.4 s p99 replanning latency is therefore not by itself a strong claim. A reviewer may ask for a comparison to Fast-dRRT*-style precomputed swept volumes.

### Gaps
- I could not confirm whether Fast-dRRT* was published at ICRA 2024 or elsewhere.
- I found no paper explicitly on kinodynamic or time-augmented dRRT* for arms.
- I did not open the full text of the Shome & Bekris handoff paper or of dRRT*. Their Q2–Q5 answers rest on how later papers characterise them.
- Not read: Pan, Wells, Shome, Kavraki, "A general task and motion planning framework for multiple manipulators," IROS 2021 (cited in [cuArms](https://arxiv.org/html/2610.36250)).

---

## KQ2. Asynchronous multi-arm planning and execution (TAMP, reservation, scheduling, CBS/LaCAM-type)

### Takeaway
Asynchronous multi-arm execution, where arms are dispatched independently and finish tasks at different times, is **well established** in four papers:
- trajectory reservation: Meehan et al. 2022
- MoveIt2 scheduler: Stoop et al. 2023/ICRA 2024
- offline TPG post-processing: APEX-MR 2025
- GPU conflict-based search with skill time windows and bounding volumes: cuArms 2026

The specific "working arm = reserved region" idea has close precedents in three of them. Meehan reserves the trajectory as collision objects. APEX-MR requires all poses of a skill's reference path to be collision-free simultaneously. cuArms uses a skill bounding volume over a time window. What I did **not** find is an *online, event-driven, composite-roadmap (dRRT*) re-solve* of the moving subset from current states.

### Cited Findings

**P6. Meehan, Roberts, Hiatt, trajectory reservation for a dual-arm robot**
- Citation: C. A. Meehan, M. Roberts, L. M. Hiatt (U.S. Naval Research Laboratory), "Asynchronous Motion Planning and Execution for a Dual-Arm Robot," ICAPS 2022 PlanRob workshop. — [PDF](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)
- Summary: The method uses "a trajectory reservation component that, when combined with off-the-shelf motion planning, prevents collisions between the arms. This approach reserves that projected space for each arm by creating collision objects in the environment that correspond to planned trajectories." — [PDF](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)
- Q1: Asynchronous. It "opportunistically dispatch[es] an arm's actions for execution even if the other arm is currently executing a task". — [PDF](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)
- Q2: Event-driven retry, from rest. A blocked arm's planner returns failure, the central controller "continues to ask for another motion plan ... until a success". This "resulted in the left arm holding its current position until the right arm moved out of the way". The moving arm is never replanned. — [PDF](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)
- Q3: **Reserved region = swept trajectory.** "For each waypoint in the trajectory, it creates a set of collision objects that match the size and pose of certain links of the robotic arm at that waypoint". "When the planned trajectory is executed, the collision objects are then removed as the arm passes them." APEX-MR later criticises this as treating the arm's whole path as a static obstacle, which "can be too conservative and cause deadlock". — [PDF](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf); [APEX-MR](https://arxiv.org/html/2503.15836)
- Q4: Cans are "attached ... to the hands when in close proximity" (no real grasping). — [PDF](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)
- Q5: MoveIt time parameterization; OMPL planners (RRT-Connect, PRM, LazyPRM). — [PDF](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)
- Q6: Gazebo with a DRC-Hubo dual-arm robot. A push scenario (16 tasks) and a pick-and-place scenario (57 tasks). Zero collisions with reservation, and a "major collision" without it. The asynchronous run took on average 1 min 19.017 s ± 2.013 s including planning. The sequential figure was cut off in my extraction (**[gap]**), but the paper says async is "more efficient than executing motions sequentially". — [PDF](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)

**P7. Stoop, Ratnayake, Toffetti, asynchronous execution in MoveIt2**
- Citation: P. Stoop, T. Ratnayake, G. Toffetti, "A Method for Multi-Robot Asynchronous Trajectory Execution in MoveIt2 (Extended Abstract)," arXiv 2310.08597 (Oct 2023). It appeared at the IROS 2023 workshop (dyalab.mines.edu hosts it), and dblp lists it at ICRA 2024 (conf/icra/StoopRT24). Code: github.com/stooppas/moveit2. — [arXiv](https://arxiv.org/abs/2310.08597); [workshop PDF](https://dyalab.mines.edu/2023/iros-workshop/stoop.pdf); [dblp](https://dblp.dagstuhl.de/rec/conf/icra/StoopRT24.html)
- Summary: The paper extends MoveIt2, which "only allows synchronous execution". It adds (1) collision detection between multiple trajectories, (2) a central execution scheduler, and (3) online collision detection. A new trajectory is checked against all running ones and executed immediately or put in a backlog. A "backlog timeout" aborts the trajectory "allowing the user to re-plan". — [PDF](https://arxiv.org/pdf/2310.08597)
- Q1: Asynchronous. Motivation: "tasks vary in duration". — [PDF](https://arxiv.org/pdf/2310.08597)
- Q2: Planning is per arm from its own state. Replanning is left to the user after a timeout. No planning from moving states is described. — [PDF](https://arxiv.org/pdf/2310.08597)
- Q3: The authors **explicitly reject** swept reservation: "'sweeping' methods that reserve all the space occupied by an entire trajectory ... reduces the motion space available for planning. For this reason, we chose to check for collision in a time dependent manner." Trajectories are discretized at a configurable step. CCD is left as future work. — [PDF](https://arxiv.org/pdf/2310.08597)
- Q4: Not discussed. **[gap]**
- Q5: MoveIt time-parameterized trajectories, checked at discrete time steps. — [PDF](https://arxiv.org/pdf/2310.08597)
- Q6: Simulated pick-and-place with two 7-DoF Panda arms. The authors say experiments "are currently running", and the extended abstract reports no numbers. — [PDF](https://arxiv.org/pdf/2310.08597)

**P8. APEX-MR (CMU, Huang, Liu, Aggarwal, Liu, Li)**
- Citation: P. Huang, R. Liu, S. Aggarwal, C. Liu, J. Li, "APEX-MR: Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly," arXiv 2503.15836 (v3, Aug 2025). The cuArms and Schnyder et al. reference lists cite it as RSS 2025. — [arXiv HTML](https://arxiv.org/html/2503.15836); venue per [cuArms refs](https://arxiv.org/html/2610.36250)
- Summary: Task assignment by ILP, single-robot RRT-Connect motion planning per task, then a multi-modal **Temporal Plan Graph (TPG)** post-processing step. The TPG converts a sequential or synchronous plan into one that executes asynchronously, with precedence edges inferred from collision checks. Shown on LEGO assembly with two arms. — [arXiv HTML](https://arxiv.org/html/2503.15836)
- Q1: Asynchronous execution, and the TPG "is robust to execution delay and uncertainty". Planning is offline: "both the TPG processing and motion planning in APEX-MR are performed offline before real execution." — [HTML](https://arxiv.org/html/2503.15836)
- Q2: No replanning. The system "cannot easily adapt to changes in the collision environment"; windowed replanning is future work. — [HTML](https://arxiv.org/html/2503.15836)
- Q3: Skills are "skill nodes" with a reference path, and "all robot poses on the reference path have to be collision-free simultaneously". In effect, the skill's whole path is reserved against the other robot's poses, very close to "reserved region = all configurations of the descent sweep". — [HTML](https://arxiv.org/html/2503.15836)
- Q4: Yes. Collision checks include "the robot links and attached objects". — [HTML](https://arxiv.org/html/2503.15836)
- Q5: RRT-Connect paths time-stamped at maximum velocity, then shortcut. Timestamps are "only used for TPG construction and is ignored during execution". A central server releases actions when TPG dependencies are satisfied. — [HTML](https://arxiv.org/html/2503.15836)
- Q6: Simulation (up to 258 objects) plus two real Yaskawa GP4 arms (real build up to 47 objects). TPG post-processing "reduces the makespan by 36% and wait time by 77% on average" against synchronous plans, and the abstract reports 48% against sequential planning. — [HTML](https://arxiv.org/html/2503.15836)
- Conflict: one search snippet claimed that Schnyder et al. 2026 call APEX-MR "turn-based, so only one robot moves at a time". The APEX-MR text describes asynchronous concurrent execution. I did not see that claim in my fetch of Schnyder et al. **[unverified snippet]** — [search snippet source](https://arxiv.org/html/2610.07784v1)

**P9. VAMP-MR (Huang, Gao, Li)**
- Citation: P. Huang, C. Gao, J. Li, "VAMP-MR: Vector-Accelerated Motion Planning and Execution for Multi-Robot-Arms," arXiv 2607.13478; IROS 2026 per the search summary and the cuArms references. — [arXiv PDF](https://arxiv.org/pdf/2607.13478); [project](https://vamp-mr.github.io/vamp-mr)
- Summary (from search result and project page; full text **not read**): CPU-SIMD collision checking, built on vamp and APEX-MR, with "up to two orders of magnitude speedup in both planning and execution postprocessing for multi-arm tasks". — [project](https://vamp-mr.github.io/vamp-mr)
- Q1–Q6: **[not read]**. It inherits APEX-MR's asynchronous TPG execution according to the project page.

**P10. cuArms (NVIDIA/CMU, Huang, Ramos, Li, Garrett, Johannsmeier)**
- Citation: P. Huang, F. Ramos, J. Li, C. Garrett, L. Johannsmeier, "cuArms: GPU-Accelerated Conflict-Based Planning for Multi-Arm Manipulation," arXiv 2610.36250 (26 Oct 2026), preprint. — [arXiv HTML](https://arxiv.org/html/2610.36250)
- Summary: Per-robot GPU PRMs with mode-validity masks (no rebuild per object or attachment state). A LaCAM-style lazy search over joint robot+action states, with PIBT conflict resolution using a batch-precomputed GPU conflict table. A scheduler supplies schedule skeletons, followed by batched multi-arm shortcutting. — [HTML](https://arxiv.org/html/2610.36250)
- Q1: Asynchronous. It finds "asynchronous trajectories and skills jointly", and unordered actions "may execute asynchronously". A robot's completion time is when it "finishes all assigned moves and skills and thereafter remains stationary". — [HTML](https://arxiv.org/html/2610.36250)
- Q2: Not addressed. Planning runs from the initial state, and failure backtracks to the scheduler. — [HTML](https://arxiv.org/html/2610.36250)
- Q3: **Skill = time window + bounding volume**: "a bounding volume enclosing the collision geometry of all participating arms and manipulated objects during execution", "active during its scheduled interval". — [HTML](https://arxiv.org/html/2610.36250)
- Q4: Yes. Modes encode "grasp, attachment, or placement state of every object", and robots may wait "while maintaining their object attachments". — [HTML](https://arxiv.org/html/2610.36250)
- Q5: Durations are rounded "up to a whole-number multiple of the max edge interval", and the output is a timed plan. Controller-level details are not given. — [HTML](https://arxiv.org/html/2610.36250)
- Q6: RoboBallet welding benchmark with 4/8 arms and 10/40 targets; tabletop TAMP with 2 Pandas and 2–8 objects; real: two Flexiv Rizon4 arms. Median time to first solution 0.20 s (4 arms/10 targets), 0.41 s (4/40) and 0.87 s (8 arms). All 150 episodes per setting solved, against RoboBallet RL's 19.3% and 33.3%. 46.4% faster than ScheduleStream, with 42–57% makespan reductions. — [HTML](https://arxiv.org/html/2610.36250)

**P11. Hartmann & Toussaint, low-makespan multi-arm multi-task planning**
- Citation: V. N. Hartmann, M. Toussaint, "Towards computing low-makespan solutions for multi-arm multi-task planning problems," PlanRob workshop @ ICAPS 2023, arXiv 2305.17527. — [arXiv](https://arxiv.org/abs/2305.17527)
- Summary: Greedy descent with random restarts over task assignment and sequence. Each candidate is evaluated by a multi-agent path planner that solves one-robot/one-task subproblems and combines them. Demonstrated on stippling and bin picking with **up to four arms**, with makespans "up to 30% lower" than a greedy approach. — [arXiv](https://arxiv.org/abs/2305.17527)
- Q1–Q5: The abstract does not detail asynchrony or time parameterization. **[not read in full]**. The earlier T-RO 2023 paper by Hartmann, Orthey, Driess, Oguz & Toussaint (arXiv 2106.02489) "split[s] task and motion problems into subproblems and synchroniz[es] them", with hardware validation on two arms. — [arXiv](https://arxiv.org/abs/2106.02489)

**P12. Hartmann, Heinle, Huang, Coros, multi-modal multi-robot multi-goal (MRMG) path planning**
- Citation: V. N. Hartmann, T. Heinle, Y. Huang, S. Coros, "Sampling-Based Multi-Modal Multi-Robot Multi-Goal Path Planning," arXiv 2503.03509 (v1 Mar 2025 as "A Benchmark for Optimal ... with Given Robot Assignment"; v2 Sep 2025). Cited as WAFR 2026. — [arXiv](https://arxiv.org/abs/2503.03509); venue per [cuArms refs](https://arxiv.org/html/2610.36250)
- Summary: Standard sampling-based planners are modified for the multi-modal multi-robot multi-goal setting on a composite space. Open-source benchmark of 21 base scenarios with up to 74 subgoals. Claim: "there are no multi-modal multi-robot multi-goal planners that are complete and optimal in the general setting". — [arXiv](https://arxiv.org/abs/2503.03509)
- Q1–Q6: **[not read in full]**.

**P13. Schnyder, Hartmann, Coros, MRMG with stochastic skills (2026)**
- Citation: W. Schnyder, V. N. Hartmann, S. Coros, "Multi-Robot Multi-Goal Motion Planning with Stochastic Skills," arXiv 2610.07784 (6 Oct 2026), preprint. — [arXiv HTML](https://arxiv.org/html/2610.07784v1)
- Summary: Builds on MRMG. Manipulation skills are modelled as autonomous closed-loop controllers with stochastic duration and outcome. Planning is an MDP over a multi-modal composite roadmap solved by value iteration. The resulting **reactive policy** is compared with a conservative open-loop baseline. — [HTML](https://arxiv.org/html/2610.07784v1)
- Q1: The policy "conditions its actions directly on the real-time observed state of both" robots. Skill termination is handled through exit values. — [HTML](https://arxiv.org/html/2610.07784v1)
- Q2: A feedback policy over observed states. Off-support states are "projected onto the nearest visited state". The policy is precomputed, not replanned online. — [HTML](https://arxiv.org/html/2610.07784v1)
- Q3: The skill controller "does _not_ account for the controllable robots". The conservative baseline checks against Monte Carlo skill realizations. The reactive planner uses a precomputed Boolean inter-robot collision mask. A single static envelope is rejected as overly conservative. — [HTML](https://arxiv.org/html/2610.07784v1)
- Q4: Only briefly: modes include "attached to a gripper". — [HTML](https://arxiv.org/html/2610.07784v1)
- Q5: Time-based stage cost, with edge length limited to v_max·Δt. — [HTML](https://arxiv.org/html/2610.07784v1)
- Q6: Simulation only (hardware and **MuJoCo** listed as future work). Two 6-DoF arms in four environments, and stacking with 2–4 arms. Mean makespan reduced 8.1–43.4%, and median reduced 19.0–24.1% in stacking. 1000 s planning limit. — [HTML](https://arxiv.org/html/2610.07784v1)

**Other works seen only in reference lists [not read]**
- Zhang & Pecora, online motion coordination with a pairwise collision matrix — cited in [APEX-MR](https://arxiv.org/html/2503.15836)
- Shaoul, Mishani, Likhachev, Li, xECBS (ICAPS 2024) — [cuArms refs](https://arxiv.org/html/2610.36250)
- Garrett & Ramos, ScheduleStream (ICRA 2026) — [cuArms refs](https://arxiv.org/html/2610.36250)
- Kimmel & Bekris 2017, velocity tuning for dual-arm — [Meehan refs](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)
- Mirrazavi Salehian et al., IJRR 2018, unified coordinated multi-arm framework — [Stoop refs](https://arxiv.org/pdf/2310.08597)

### Inferences
- **Asynchronous execution alone is not novel**: Meehan 2022, Stoop 2023/24, Sung 2024, APEX-MR 2025, cuArms 2026, Schnyder 2026.
- **Reported gains match the project's +30%.** Async against sync is about 31% (Sung, my arithmetic), 36% (APEX-MR TPG), and up to 30% (Hartmann against greedy). A +30% throughput gain over turn-based planning is in the expected range, not surprising.
- **Reservation of a working arm's region is closely precedented.**
  - Meehan reserves per-waypoint link volumes along the trajectory.
  - APEX-MR requires a skill's whole reference path to be simultaneously collision-free.
  - cuArms uses a skill bounding volume active for a time window.
  - Stoop argues against swept reservation as too conservative.
  - Defining the reservation as the *set of configurations of the descent sweep*, checked in C-space against a composite dRRT* expansion, is a minor variant of these. It is not a new idea.
- **The open part is the online loop inside dRRT*.** That means event-driven re-solves in which only currently-moving arms are re-rooted at their *current* configurations (temporary roadmap vertices), while working arms are frozen as reserved obstacles. Every composite-roadmap method I found (P2, P4, P5, P10, P12, P13) plans offline from a static start.

### Gaps
- No full text read for VAMP-MR, Hartmann & Toussaint 2023, MRMG 2025, Zhang & Pecora, xECBS, ScheduleStream, or CBS-MP (Solis et al.). Any of them could contain online replanning. ScheduleStream and VAMP-MR are the most important to check.
- I could not get Meehan's sequential baseline time from the PDF extraction.
- I found no industrial case study (Siemens or automotive welding lines) with published online replanning details. This was not searched exhaustively.

---

## KQ3. Decentralized / learned multi-arm planners

### Takeaway
The learned planners are closed-loop, so they react from moving states by construction. They target reaching (end-effector pose) tasks without dRRT*-style guarantees. cuArms reports they solve far fewer RoboBallet episodes than a search-based planner.

### Cited Findings

**P14. Ha, Xu, Song, decentralized multi-arm motion planner**
- Citation: H. Ha, J. Xu, S. Song, "Learning a Decentralized Multi-arm Motion Planner," CoRL 2020, arXiv 2011.02608. — [arXiv](https://arxiv.org/abs/2011.02608)
- Summary: A "closed-loop and decentralized" per-arm policy trained with multi-agent RL (Soft Actor-Critic) plus BiRRT expert demonstrations. Observations are the arm's workspace state and its target end-effector pose. It is trained on 1–4 arms and generalizes to 5–10 arms, with over 90% success for a 10-arm system with "dynamic moving targets". — [arXiv](https://arxiv.org/abs/2011.02608)
- Q1/Q2: Closed-loop, so each arm acts from its current state at every step. Asynchronous targets are possible. — [arXiv](https://arxiv.org/abs/2011.02608)
- Q3–Q5: Reaching only, per the abstract. Grasping and carried objects are not mentioned in the abstract. **[gap]**
- Q6: The abstract does not say whether real robots were used. **[gap]**

**P15. RoboBallet (DeepMind/Intrinsic)**
- Citation: M. Lai, K. Go, Z. Li, T. Kröger, S. Schaal, K. Allen, J. Scholz, "RoboBallet: Planning for multirobot reaching with graph neural networks and reinforcement learning," *Science Robotics*, 3 Sep 2025, DOI 10.1126/scirobotics.ads1204; arXiv 2509.05397. — [arXiv](https://arxiv.org/abs/2509.05397); [DeepMind](https://deepmind.google/research/publications/roboballet-planning-for-multi-robot-reaching-with-graph-neural-networks-and-reinforcement-learning/)
- Summary: A graph policy network trained with RL jointly handles task allocation, scheduling and motion for multiple robots. Evaluated with 8 robots and 40 reaching tasks in an obstacle-rich shared workspace, with zero-shot transfer to new layouts. — [arXiv](https://arxiv.org/abs/2509.05397)
- Q3–Q6 beyond this: **[not read]**. cuArms reports that RoboBallet RL solved 19.3% (10 targets) and 33.3% (40 targets) of episodes on its benchmark. — [cuArms](https://arxiv.org/html/2610.36250)

### Inferences
- Learned planners offer reactivity from moving states but no completeness. They have not been shown on pick-and-place with carried objects, at least in the abstracts I read. A sampling-based online replanner with collision guarantees stays a reasonable position against them.

### Gaps
- I did not verify multi-arm safe-RL work or industrial deployments.

---

## KQ4. Continuous replanning from moving (non-rest) states for manipulators

### Takeaway
None of the composite-roadmap multi-arm papers I read replans from moving states. The closest designs to "non-stop" behaviour are:
- closed-loop learned policies (Ha et al. 2020)
- a precomputed reactive MDP policy over a composite roadmap (Schnyder et al. 2026)
- execution-time monitoring with discrete-time collision checks (Stoop et al.)
- TPG execution, which is robust to delays but planned offline (APEX-MR)

### Cited Findings
- APEX-MR names "windowed replanning" as future work and says it "cannot easily adapt to changes". — [APEX-MR](https://arxiv.org/html/2503.15836)
- cuArms has no mid-execution replanning; it backtracks to the scheduler on failure at planning time. — [cuArms](https://arxiv.org/html/2610.36250)
- Meehan: a blocked arm holds position and re-requests a plan. Replanning happens only from rest. — [Meehan](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf)
- Stoop: online collision checks during execution, with abort and user re-plan after a backlog timeout. — [Stoop](https://arxiv.org/pdf/2310.08597)
- Schnyder et al.: the policy is conditioned on observed states of both robots, and off-support states are projected to the nearest visited state. — [Schnyder](https://arxiv.org/html/2610.07784v1)
- Ha et al.: a closed-loop policy that handles moving targets. — [Ha](https://arxiv.org/abs/2011.02608)

### Inferences
- Continuous non-stop dRRT* replanning seems **open**. It would need a commit horizon (keep executing the prefix while planning), a start state that is predicted rather than current, and time-parameterized blending into the new path with velocity continuity.

### Gaps
- Classic single-arm real-time replanning with commit horizons (Hauser's "responsiveness, safety and completeness" line, receding-horizon trajectory optimisation) was **not searched** in this pass for lack of budget. A related-work section on Angle B must cover it.

---

## KQ5. Synthesis: novelty of "Angle B"

### Takeaway
**Partially done.** The individual ingredients exist:
- asynchronous multi-arm execution
- reserved regions or time windows for working arms
- carried objects in collision checking
- dRRT extended to asynchronous multi-modal plans
- dRRT* with idle-robot handling and swept volumes

The *combination* I did not find in 2019–2026 work is an **online, event-driven, dRRT*-based re-solve of only the currently moving subset, from their current configurations inserted as temporary vertices, with working arms frozen as configuration-sweep reservations, on a ≥4-arm pick-and-place cell**. **Continuous replanning from moving states inside dRRT*** appears open.

### Cited Findings

| Angle B component | Status | Evidence |
|---|---|---|
| Asynchronous arms, tasks finishing at different times | **Done** | Meehan 2022, Stoop 2023/24, Sung 2024, APEX-MR 2025, cuArms 2026 ([Meehan](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf), [Stoop](https://arxiv.org/pdf/2310.08597), [Sung](https://arxiv.org/pdf/2309.08897), [APEX-MR](https://arxiv.org/html/2503.15836), [cuArms](https://arxiv.org/html/2610.36250)) |
| dRRT/dRRT* used for asynchronous multi-arm plans | **Partially** (offline) | Sung et al. extend dRRT with ordering and held objects ([PDF](https://arxiv.org/pdf/2309.08897)); Duong et al. 2026 use a dRRT*-based search for MR-TAMP ([HTML](https://arxiv.org/html/2609.18813)) |
| Plan only a subset of arms; the others idle | **Partially** | Fast-dRRT* "partial multi-robot problems" with any-goal robots kept in place but movable ([PDF](https://arxiv.org/pdf/2309.10665)) |
| Event-driven online replanning from current configurations (temporary roadmap vertices) | **Not found** for composite roadmaps. Meehan replans single arms from rest. | [Meehan](https://icaps22.icaps-conference.org/workshops/PlanRob/PlanRob_2022_paper_11.pdf); APEX-MR lists it as future work ([HTML](https://arxiv.org/html/2503.15836)) |
| Working arm = reserved region / swept volume / time window | **Done in variants** | Meehan per-waypoint link objects; APEX-MR skill path simultaneously collision-free; cuArms skill bounding volume + time window; Fast-dRRT* edge swept volumes (for speed); Stoop argues against sweeps |
| Carried object modelled in collision checks | **Done (standard)** | APEX-MR, cuArms, Sung, Duong, Meehan |
| Total planning-budget cap | Engineering detail. dRRT already has time limits with backtracking. | [Sung](https://arxiv.org/pdf/2309.08897) |
| Continuous (non-stop) replanning from moving states, dRRT* | **Open** as far as found. Closest are learned or precomputed reactive policies. | [Ha](https://arxiv.org/abs/2011.02608), [Schnyder](https://arxiv.org/html/2610.07784v1) |

### Inferences
- **Novelty risk is high** if the paper is framed as "asynchronous multi-arm pick-and-place". Reviewers will cite APEX-MR, cuArms, Sung et al. and Meehan.
- **The defensible novelty is in three points:**
  - (a) *online, event-driven* re-solving within the dRRT* composite roadmap. This needs anytime and asymptotic-optimality properties preserved per re-solve, and a measured latency distribution.
  - (b) *continuous replanning from moving states* (commit horizon plus velocity-continuous splicing), which removes the current stop-at-segment-boundary behaviour.
  - (c) a 4-arm UR10e cell with physics simulation (MuJoCo), real gripper pick success, and 400-pick statistics. Most of the cited works use 2 arms on real hardware or kinematic simulation (Sung uses "simulation steps", and Schnyder lists MuJoCo as future work).
- **Expected comparisons:**
  - APEX-MR-style offline planning + TPG asynchronous execution (open source via VAMP-MR / APEX-MR)
  - Meehan-style trajectory reservation with a single-arm planner
  - Fast-dRRT*-style swept volumes for latency
  - possibly cuArms if code is released
- The project's current numbers (+30% throughput against turn-based, 0.54 s median and 1.4 s p99 replanning latency, 0 collisions in 400 picks) are in the same range as the literature's async-vs-sync gains. The latency is slower than Fast-dRRT*'s ~100 ms first solutions and cuArms' 0.2–0.9 s. The latency should be presented with that context.

### Gaps
- Not checked: ScheduleStream (Garrett & Ramos, ICRA 2026), VAMP-MR full text, xECBS, CBS-MP, Zhang & Pecora. Any of them could already do online subset replanning. Checking them is the main remaining due-diligence item before claiming novelty.
- The single-arm real-time replanning literature (commit horizon / receding horizon) was not surveyed here.
