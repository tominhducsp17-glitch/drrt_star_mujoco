# Verification of closest competing works (multi-arm pick-and-place, online allocation + dRRT* replanning)

Checked 2026-11-16. Every claim below links to the page it came from. "UNVERIFIED" marks claims I could not confirm from a primary source. Where WebFetch only gave a paraphrase of a page (arXiv abstract pages limit quoting), the claim is a paraphrase of that page.

Our candidate contributions, for reference: (A) a planner-grounded or learned inter-arm delay cost for online allocation, plus a regime map; (B) online, event-driven, asynchronous dRRT* with reserved regions and replanning from moving states; (C) a learned grasp skill confined to a reserved region inside a multi-arm planner.

## 1. MultiPnP — RL graph-network policy for allocation, scheduling and motion

### Takeaway
MultiPnP exists only as an anonymous project page (BibTeX `anonymous2026multipnp`, venue "[venue]"), and its Paper and arXiv buttons are disabled. It is a single centralized GNN RL policy that runs closed-loop by construction. It does not replan with a motion planner. Conflicts are handled by rejecting commands that would collide, not by reserving space. Its baselines are naive serial and parallel planners plus an ablated policy. It does not compare against MAPF or dRRT-type planners, or against conflict-aware allocation.

### Cited Findings
- **Identity.** Title: "MultiPnP: Learning Task Allocation, Scheduling, and Motion Planning for Multi-Robot Pick-and-Place". The BibTeX lists author `Anonymous`, journal `[venue]` and key `anonymous2026multipnp`. The Paper and arXiv buttons are rendered `disabled aria-disabled="true"`, and the only working link is the repo github.com/multipnp/multipnp.github.io — [MultiPnP page](https://multipnp.github.io/) (raw HTML checked).
- An arXiv title search and an "MultiPnP" arXiv/OpenReview web search found no paper — [arXiv API query run 2026-11-16](https://export.arxiv.org/api/query?search_query=all:pick%20AND%20all:place%20AND%20all:multi-robot%20AND%20all:graph); [MultiPnP page](https://multipnp.github.io/).
- **Method.** One RL policy jointly learns task allocation, scheduling and motion planning. A GNN runs over a single graph of robots, objects, bins and obstacles, and training uses "two-stage hindsight experience replay". A pick-only "reaching policy" is trained first as a prior — [MultiPnP page](https://multipnp.github.io/).
- **Action interface.** The policy "commands a coarse standoff pose above an object", and "a classical inverse-kinematics solver then takes over for the last few centimeters". A pick registers "only if every waypoint of that descent has a collision-free arm pose — otherwise the approach is refused and the object stays available to try again" — [MultiPnP page](https://multipnp.github.io/).
- **Conflict handling.** The GNN reasons "how to move without colliding with the others". In the parallel baseline, "blocked motions — motion commands from the planner that would cause a collision if executed, and are rejected instead" — [MultiPnP page](https://multipnp.github.io/).
- **Baselines.** (i) SinglePnP + a naive serial planner. (ii) SinglePnP + a naive parallel planner. (iii) The reaching policy, which completes no scene. MultiPnP takes "15 s on average, 3.5× faster than the serial planner (53 s), which only succeeds when allowed a 3× longer time limit" — [MultiPnP page](https://multipnp.github.io/).
- **Results.** 92.5% success with 4 robots and 8 objects in simulation (MuJoCo, per the figure caption). Zero-shot transfer to 2–4 robots and 12–32 objects. Sorting needs "a brief fine-tuning". On hardware: "a physical cell of 4 robot arms, clears 92% of 50 real-world scenes and places 97.9% of 480 objects with no executed collisions" — [MultiPnP page](https://multipnp.github.io/).
- **Ablations.** Without the docking mechanism the success rate "roughly halves". Without two-stage HER or the place-phase carry signal, the policy "fails to complete the task at all" — [MultiPnP page](https://multipnp.github.io/).

### Inferences
- **Online replanning.** It is a reactive policy queried at every step, so it is "online" in the RL sense. It never replans paths with a sampling-based motion planner, and it has no explicit event-driven replanning. This leaves contribution B distinct from it.
- **Conflict handling.** Conflicts are avoided implicitly by the learned policy, with a hard refusal at the IK descent step. There is no reserved region and no conflict cost in an allocator. This leaves contributions A and C distinct, though MultiPnP is the strongest "learned end-to-end" competitor on the same 4-arm / 8-object / MuJoCo setup.
- **Threat to positioning.** Its setup closely matches ours (4 arms, 8 objects, MuJoCo, sorting into bins). Reviewers will expect a comparison or a clear differentiation, such as completeness or collision guarantees from the planner, no training, and interpretability. Its serial baseline (53 s) is weak, which opens room for a strong planner baseline.

### Gaps
- Authors, venue, arXiv ID and arm model could not be found; the page does not name the hardware. The page is probably under double-blind review (2026 submission).
- Reward terms, the policy's control rate, and whether the arm motion between standoffs comes from the policy or a controller are not stated.

## 2. ScheduleStream (Garrett & Ramos)

### Takeaway
Verified: arXiv 2511.04758 (v1 6 Nov 2025, v2 28 May 2026), ICRA 2026, NVIDIA. It does asynchronous scheduling (hybrid durative actions) and automatic arm–object allocation, but offline: it returns a complete schedule. No online replanning or execution monitoring is described. Experiments are mainly bimanual.

### Cited Findings
- **Identity.** "ScheduleStream: Temporal Planning with Samplers for GPU-Accelerated Multi-Arm Task and Motion Planning & Scheduling", by Caelan Garrett and Fabio Ramos. Journal ref: 2026 IEEE ICRA. Project: schedulestream.github.io — [arXiv 2511.04758](https://arxiv.org/abs/2511.04758); [NVIDIA SRL page](https://research.nvidia.com/labs/srl/publication/ramos-2026-schedule-stream/).
- It models "hybrid durative actions, which can be started asynchronously and persist for a duration that's a function of their parameters", with GPU-accelerated samplers — [project page](https://schedulestream.github.io).
- **Allocation.** "No prior information is provided regarding which arms should or even can hold which object… ScheduleStream automatically allocates objects to arms" — [project page](https://schedulestream.github.io).
- **Collisions.** An `ArmCollision` condition checks whether "the swept volume induced by trajectory ?t1 intersects with the swept volume induced by trajectory ?t2". On GPU, this runs as a batch sphere-sphere check — [arXiv HTML v2](https://arxiv.org/html/2511.04758v2).
- **Experiments.** Simulated N-Franka / N-SO100 tasks (2 Frankas for Pack/Stack). The real robot has two Kinova Gen3 arms. The simulator is not named — [arXiv HTML v2](https://arxiv.org/html/2511.04758v2); [project page](https://schedulestream.github.io).
- **Replanning.** Neither the paper nor the project page describes online replanning, closed-loop reaction to delays, or future work on it. The output is a complete schedule — [arXiv HTML v2](https://arxiv.org/html/2511.04758v2); [project page](https://schedulestream.github.io).

### Inferences
- ScheduleStream's pairwise swept-volume collision condition on concurrent durative actions is conceptually close to our "reserved region = descent sweep". It is offline, though, and lives inside a temporal planner. Our contribution B must stress runtime replanning from moving states.

### Gaps
- The full v2 text was only read through a summarizing fetch. A statement on execution monitoring could still be in an appendix (UNVERIFIED negative).

## 3. VAMP-MR (IROS 2026)

### Takeaway
Verified: arXiv 2607.13478 by Philip Huang, Chenrui Gao and Jiaoyang Li (CMU), "To appear in the Proceedings of IROS 2026". It is a SIMD-vectorized collision-checking suite for composite RRT-Connect and CBS-style planners, plus faster execution postprocessing (TPG construction and shortcutting). Neither the abstract nor the project page describes online replanning. Asynchrony comes from the separate APEX-MR execution framework.

### Cited Findings
- **Identity.** "VAMP-MR: Vector-Accelerated Motion Planning and Execution for Multi-Robot-Arms", v1 15 Jul 2026. Comment: "8 pages, 6 figures, 3 tables. To appear in the Proceedings of IROS 2026" — [arXiv 2607.13478](https://arxiv.org/abs/2607.13478).
- **Planners.** "composite RRT-Connect and CBS-style multi-robot planning". Environments: "Panda Two Rod", "Panda Four", "Panda Four Bins", and dual-arm LEGO assembly (11–258 bricks). Collision checking is 11.7×–148.3× faster than FCL, and shortcutting converges up to 50× faster — [project page](https://vamp-mr.github.io/vamp-mr).
- **Execution.** It mentions "building execution graph" and "TPG construction". Online replanning and asynchronous execution are not described for VAMP-MR; APEX-MR ("Multi-Robot Asynchronous Planning and Execution") is cited as a separate project — [project page](https://vamp-mr.github.io/vamp-mr).
- APEX-MR (arXiv 2503.15836) "provides a systematic approach to post-process multi-robot tasks and motion plans to enable robust asynchronous execution under uncertainty" — [arXiv 2503.15836](https://arxiv.org/abs/2503.15836).
- **Not to be confused with:** a separate multi-robot VAMP-style paper, "Multi-Robot Motions in Milliseconds: Vector-Accelerated Primitives for Sampling-Based Planning" (Motes, Morales, Amato), arXiv 2604.23960 — [arXiv PDF](https://arxiv.org/pdf/2604.23960).

### Inferences
- VAMP-MR fits as a speed backbone or a planning baseline, not as a competitor on online replanning. Its asynchrony means post-processing an offline plan into a TPG/ADG (APEX-MR). That differs from replanning only the moving arms from their current states (contribution B), though reviewers may see it as "execution-time robustness already handled".

### Gaps
- The VAMP-MR full text was not read; only the abstract and project page. APEX-MR's exact mechanism (ADG vs. TPG, whether it replans) is not verified.

## 4. xECBS (arXiv 2404.00143) and CBS-MP

### Takeaway
xECBS (Shaoul, Mishani, Likhachev, Li; ICAPS 2024) is an offline planner. Its "online-generated experiences" are reused during search, not during execution. It does no execution-time replanning for a subset of arms, and no code or scenes are linked on arXiv. CBS-MP appears as a baseline in xECBS and is not a Shaoul paper. The most likely source is Solis et al., RA-L 2021 (mapping UNVERIFIED).

### Cited Findings
- **Identity.** "Accelerating Search-Based Planning for Multi-Robot Manipulation by Leveraging Online-Generated Experiences", by Yorai Shaoul, Itamar Mishani, Maxim Likhachev and Jiaoyang Li. Submitted 29 Mar 2024; "Accepted to ICAPS 2024". Up to 10 arms. No code link on the abstract page — [arXiv 2404.00143](https://arxiv.org/abs/2404.00143).
- **Planning.** The planner returns the full set of per-arm paths once a conflict-free node is found. "Online" refers to reusing experience within the search. No execution-time replanning or delay handling is described. Evaluation is in simulation with 7-DoF Franka Pandas (8-arm shelf rearrangement, 4-arm bin picking, 2–10 arms in free or lightly cluttered scenes). MoveIt and Isaac Sim are used for environment handling and rendering — [arXiv HTML v1](https://arxiv.org/html/2404.00143v1).
- **Baselines.** PRM, RRT-Connect, dRRT*, PP, CBS, ECBS and CBS-MP — [arXiv HTML v1](https://arxiv.org/html/2404.00143v1).
- **Follow-up.** "Unconstraining Multi-Robot Manipulation: Enabling Arbitrary Constraints in ECBS with Bounded Sub-Optimality" (Shaoul, Veerapaneni, Likhachev, Li), arXiv 2405.01772 — [arXiv HTML](https://arxiv.org/html/2405.01772v4).
- **CBS-MP candidate.** "Representation-Optimal Multi-Robot Motion Planning Using Conflict-Based Search" (Solis Vidana, Motes, Sandström, Amato), RA-L 2021, DOI 10.1109/lra.2021.3068910 — [OpenAlex](https://api.openalex.org/works?search=Representation-optimal%20multi-robot%20motion%20planning%20using%20conflict-based%20search).

### Inferences
- Neither xECBS nor CBS-MP does online replanning for a subset of arms during execution, so contribution B holds against this line of work. cuArms (item 7) is the newest CBS-line descendant and is also offline.

### Gaps
- Benchmark scene release: no repository link found for xECBS (UNVERIFIED negative). The authors' GitHub was not checked directly.
- That "CBS-MP" names exactly Solis et al. 2021 is inferred, not read from the xECBS reference list.

## 5. Zhang & Pecora — online coordination of manipulators

### Takeaway
Two verified papers by Shiyu Zhang and Federico Pecora are directly relevant:
- **TASE 2023/24:** online, scalable motion coordination with local trajectory replanning while robots move.
- **RA-L 2021:** online sequential task assignment that accounts for interference among concurrent motions, with motion coordination integrated into a branch-and-bound assignment.

The RA-L paper is the closest prior art to contribution A (conflict-aware online allocation). "Critical sections" is terminology from Pecora's mobile-robot ICAPS 2018 work. I could not verify that the manipulator papers use critical sections equivalent to our reserved regions.

### Cited Findings
- **TASE paper, identity.** "Online and Scalable Motion Coordination for Multiple Robot Manipulators in Shared Workspaces", IEEE T-ASE, DOI 10.1109/TASE.2023.3266889 (OpenAlex year 2023; vol. 21(3) 2024 per search snippet) — [OpenAlex record](https://api.openalex.org/works/W4366310718); [IEEE Xplore](https://ieeexplore.ieee.org/document/10103893/).
- **TASE paper, method.** The abstract says it "separates the problem into a global coordination stage and a local trajectory replanning stage". "Online reactivity and scalability are achieved by means of coordination in a reduced space and efficient trajectory replanning". It works with "given targets", "allows to adjust robot trajectories while robots move", and "provides information for switching targets if solutions cannot be found". It scales quadratically, with up to 3 real 7-DoF robots and 10 simulated. The motivation covers uncertain task durations and "unexpected delays in motion or stops" — [OpenAlex abstract](https://api.openalex.org/works/W4366310718).
- A citing paper reportedly describes it as paths planned offline plus a pairwise collision matrix between robots — [search summary](https://www.researchgate.net/publication/370114687_Online_and_Scalable_Motion_Coordination_for_Multiple_Robot_Manipulators_in_Shared_Workspaces) (secondary, UNVERIFIED).
- **RA-L paper, identity.** "Online Sequential Task Assignment With Execution Uncertainties for Multiple Robot Manipulators", IEEE RA-L 2021, DOI 10.1109/LRA.2021.3093874 — [OpenAlex](https://api.openalex.org/works/W3186042173).
- **RA-L paper, method.** The abstract describes "constantly assigning tasks to robots, while the interference among concurrent robot motions is account for". It "decouples the time-extended problem into a sequence of synchronous and asynchronous instantaneous assignment sub-problems". It uses "one-step-ahead simulation", and each sub-problem is "solved by a branch-and-bound algorithm, with which multi-robot motion coordination is integrated". Tested on 3 Franka Panda arms, with linear scaling in simulation — [OpenAlex abstract](https://api.openalex.org/works/W3186042173).
- **Critical sections (mobile robots).** Pecora, Andreasson, Mansouri, Petkov, "A Loosely-Coupled Approach for Multi-Robot Coordination, Motion Planning and Control", ICAPS 2018: critical sections plus online-revised precedence constraints. The `coordination_oru` implementation supports online goal posting — [ICAPS PDF](http://iliad-project.eu/wp-content/uploads/papers/PecoraEtAlICAPS2018.pdf); [coordination_oru](https://github.com/FedericoPecora/coordination_oru).
- **Follow-up citing both Zhang & Pecora papers.** Qin, Liao, Liu, Xiong, "Online task allocation and scheduling in multi-manipulator system considering collision constraints and unknown tasks", Robotics and Computer-Integrated Manufacturing 2024, DOI 10.1016/j.rcim.2024.102808. Per its abstract it does pre-allocation, then a greedy online allocation within the "current feasible set of manipulators" under collision constraints, plus online addition of unknown tasks, evaluated in simulation — [OpenAlex citing list](https://api.openalex.org/works?filter=cites:W3186042173); [x-mol abstract](https://x-mol.com/paper/1812712343199084544).
- **Other online multi-arm coordination citing TASE.** Sun & Liao, "Real-Time Coordination of Multiple Robotic Arms With Reactive Trajectory Modulation", T-RO 2024 (learned movement primitives plus reactive avoidance). Also "A Method for Multi-Robot Asynchronous Trajectory Execution in MoveIt2", ICRA 2024 — [OpenAlex citing list](https://api.openalex.org/works?filter=cites:W4366310718).

### Inferences
- Contribution B: Zhang & Pecora TASE already claims online coordination with replanning while robots move, for manipulators with uncertain task durations and delays. The difference must be framed concretely. Our approach replans with a sampling-based composite roadmap (dRRT*) from current states for the moving arms only, with reserved descent sweeps for arms doing local work. Theirs coordinates fixed paths in a reduced (coordination) space with time-optimal trajectory replanning. Whether their method can change geometric paths is UNVERIFIED.
- Contribution A: Zhang & Pecora RA-L 2021 and Qin et al. RCIM 2024 already do online greedy or branch-and-bound allocation that accounts for inter-arm interference. A novelty claim for A must rest on a planner-grounded or learned delay cost and the regime map, not on "conflict-aware online allocation" in itself.

### Gaps
- No full text of either Zhang & Pecora paper was available (IEEE Xplore and AMiner returned empty pages). Whether they use "critical sections" for manipulators, and whether those equal our reserved regions, is UNVERIFIED.

## 6. Cited-by for Oberthür et al. 2025 and El Ghazi et al. 2024

### Takeaway
Both papers are verified, and both have essentially no follow-ups. Oberthür 2025 has 0 citations in Semantic Scholar and OpenAlex. El Ghazi 2024 has 1 citation in OpenAlex (an agricultural review). Neither has a follow-up that executes the schedules with real motion planning. The niche of executing these hitbox-based schedules with an actual multi-arm motion planner is open.

### Cited Findings
- **Oberthür et al. 2025, identity.** "Scheduling of Pick-and-Place Tasks in Shared Workspaces Using a Trajectory-Free Approximation Method" (Oberthür, El Ghazi, Laurent, Cardin), IFAC-PapersOnLine 2025, DOI 10.1016/j.ifacol.2025.09.233. citationCount 0 with an empty citations list — [Semantic Scholar API](https://api.semanticscholar.org/graph/v1/paper/DOI:10.1016/j.ifacol.2025.09.233?fields=title,authors,year,venue,citationCount,citations.title); cited_by 0 — [OpenAlex](https://api.openalex.org/works/W4414568256).
- **Oberthür et al. 2025, abstract.** Four heterogeneous arms in a limited shared workspace. The method uses only trajectory durations and relative positions, "without prior knowledge of the space taken by the robots", with "hitboxes to approximate the space occupied by robots". It is faster than El Ghazi's MILP and often achieves a better makespan at 10 objects — [OpenAlex abstract](https://api.openalex.org/works/W4414568256). (The abstract does not say the hitboxes are elliptic; that detail comes from the user's brief and is UNVERIFIED here.)
- **El Ghazi et al. 2024, identity.** "Task Allocation and Motion Planning Strategies for Multi-robot Cooperation" (El Ghazi, Subrin, Levilly, Mouchère, Cardin), Studies in Computational Intelligence vol. 1136 (SOHOMA 2023), DOI 10.1007/978-3-031-53445-4_32. OpenAlex cited_by_count 1 — [OpenAlex](https://api.openalex.org/works/W4389128334).
- **El Ghazi et al. 2024, only citing work.** "Intelligent robotic systems for flower harvesting: A review…", Computers and Electronics in Agriculture 2026 — [OpenAlex cites filter](https://api.openalex.org/works?filter=cites:W4389128334).
- Per the Oberthür abstract, El Ghazi uses MILP "using trajectory simulations" — [OpenAlex abstract](https://api.openalex.org/works/W4414568256).

### Inferences
- No follow-up executes these schedules with real motion planning or replanning, so contribution A (grounding the allocation cost in a real planner and measuring when the hitbox penalty helps) does not overlap this line. Google Scholar may index a few more items (theses, HAL preprints).

### Gaps
- Google Scholar "cited by" could not be queried (no API, and it blocks scraping). The Semantic Scholar query for El Ghazi returned HTTP 429, so only OpenAlex was used for it.

## 7. Schnyder, Hartmann & Coros (2610.07784) and cuArms (2610.36250): what they do not do

### Takeaway
Both are verified.
- **Schnyder et al.** computes a reactive policy offline over a multi-modal composite roadmap (an MDP with stochastic skills) and looks it up at runtime. It has no online replanning, no learned skills, no physics simulation and no hardware; all three are listed as future work. Task assignment appears to be given.
- **cuArms** does GPU CBS/LaCAM-style planning from schedule skeletons. Its replanning happens at planning time only. It has no learned skills, no named physics simulator (geometric sphere checks), and only qualitative real-robot runs (2 Flexiv arms). It does not compare to MultiPnP.

### Cited Findings
- **Schnyder et al., identity.** "Multi-Robot Multi-Goal Motion Planning with Stochastic Skills", by William Schnyder, Valentin N. Hartmann and Stelian Coros (ETH Zürich). Submitted 6 Oct 2026; 8 pages. Planner code is public — [arXiv 2610.07784](https://arxiv.org/abs/2610.07784).
- **Schnyder et al., replanning.** At runtime "the action prescribed by π⋆ is retrieved in 𝒪(1) time". If the state is off the roadmap, it "is projected onto the nearest visited state within the current epoch layer k". There is no online replanning — [arXiv HTML](https://arxiv.org/html/2610.07784).
- **Schnyder et al., skills.** Skills are stochastic disturbance models ("supports arbitrary, multimodal skill distributions"), not learned. "Learned skill policies" are future work — [arXiv HTML](https://arxiv.org/html/2610.07784).
- **Schnyder et al., simulation and hardware.** Physics validation "such as MuJoCo" and "physical hardware" are both future work — [arXiv HTML](https://arxiv.org/html/2610.07784).
- **Schnyder et al., skill space.** Collisions are checked against sampled skill rollouts at matching time steps. A static envelope is rejected as "overly conservative" — [arXiv HTML](https://arxiv.org/html/2610.07784).
- **Schnyder et al., scale and results.** Up to 4 six-DoF arms and 8 objects (stochastic stacking; pick duration 1 s or 3 s). The reactive planner cuts median makespan by 19.0–24.1% vs. the conservative open-loop planner. Success drops to 60% at 4 robots — [arXiv HTML](https://arxiv.org/html/2610.07784).
- **cuArms, identity.** "cuArms: GPU-Accelerated Conflict-Based Planning for Multi-Arm Manipulation", by Philip Huang, Fabio Ramos, Jiaoyang Li, Caelan Garrett and Lars Johannsmeier. Submitted 26 Oct 2026 — [arXiv 2610.36250](https://arxiv.org/abs/2610.36250).
- **cuArms, replanning.** "cuArms then plans the manipulation trajectory from the given schedule skeleton". It "can iteratively replan with different schedule skeletons", but at planning time. No runtime delay handling or ADG/TPG is described. Unordered actions "may execute asynchronously" — [arXiv HTML](https://arxiv.org/html/2610.36250).
- **cuArms, skills, simulation and hardware.** The skill sampler is GPU-based, not learned. Collisions use "unions of collision spheres". No physics engine is named. There are qualitative real runs on "two Flexiv Rizon4s arms" — [arXiv HTML](https://arxiv.org/html/2610.36250).
- **cuArms, allocation and baselines.** Allocation comes from a high-level scheduler: ScheduleStream, an optimization program, or a fixed order. For RoboBallet it is greedy Hungarian matching each round. Baselines are RoboBallet RL (4 and 8 arms, 10 and 40 targets) and ScheduleStream (2-Panda TAMP). xECBS, dRRT* and APEX-MR appear only in related work, and MultiPnP is not mentioned — [arXiv HTML](https://arxiv.org/html/2610.36250).

### Inferences
- Both leave open: event-driven online replanning from moving states (B), learned skills inside a multi-arm planner (C, which Schnyder et al. explicitly name as future work), and physics-simulated execution. Schnyder et al. is the closest conceptual neighbour for B and C: it does reactive coordination around stochastic skills over a composite roadmap, from the dRRT* family. A paper must contrast online replanning with an offline-computed MDP policy.

### Gaps
- cuArms: the episode counts are inconsistent across sections (as fetched). The real-robot setup is described only qualitatively.

## Verdict list

### Takeaway
No verified work combines all three of our elements: online event-driven dRRT* replanning of moving arms only, with reserved descent-sweep regions; planner-grounded conflict cost in online allocation; and learned skills inside the planner in a physics simulator. The two biggest threats are Zhang & Pecora (for A and B) and MultiPnP (same setup, learned end-to-end).

### Cited Findings
- **MultiPnP:** anonymous, no arXiv. A reactive RL policy with refusal-based collision handling and naive baselines. It is a must-compare or must-discuss competitor on the identical 4-arm / 8-object MuJoCo setup — [MultiPnP](https://multipnp.github.io/).
- **ScheduleStream (ICRA 2026):** asynchronous durative actions and automatic allocation, but an offline schedule with swept-volume conflict conditions. No online replanning — [arXiv](https://arxiv.org/abs/2511.04758).
- **VAMP-MR (IROS 2026):** acceleration of the planning backbone plus TPG post-processing. No online replanning described — [arXiv](https://arxiv.org/abs/2607.13478).
- **xECBS (ICAPS 2024) and CBS-MP:** offline planning. No subset replanning during execution, and no released scenes found — [arXiv](https://arxiv.org/html/2404.00143v1).
- **Zhang & Pecora:** online manipulator coordination with replanning during motion (TASE), and online interference-aware task assignment (RA-L 2021). This is the strongest prior art against "conflict-aware online allocation" and "online coordination"; critical-section equivalence is UNVERIFIED — [TASE](https://api.openalex.org/works/W4366310718); [RA-L](https://api.openalex.org/works/W3186042173).
- **Qin et al. (RCIM 2024):** online greedy allocation under collision constraints, in simulation. Additional prior art for A — [x-mol](https://x-mol.com/paper/1812712343199084544).
- **Oberthür 2025 and El Ghazi 2024:** 0 and 1 citations (OpenAlex). No follow-up executes the schedules with motion planning — [OpenAlex](https://api.openalex.org/works/W4414568256).
- **Schnyder, Hartmann, Coros (Oct 2026):** an offline MDP policy over a composite roadmap. No online replanning, learned skills, physics simulation or hardware (all listed as future work) — [arXiv](https://arxiv.org/html/2610.07784).
- **cuArms (Oct 2026):** offline GPU CBS from skeletons. No learned skills, no runtime replanning, no physics engine named — [arXiv](https://arxiv.org/html/2610.36250).

### Inferences
- **A:** novel only as a planner-grounded or learned delay cost plus a regime map. It must cite and compare against Zhang & Pecora 2021, Qin et al. 2024 and Oberthür 2025.
- **B:** defensible against ScheduleStream, VAMP-MR/APEX-MR, xECBS, cuArms and Schnyder et al., which are all offline or policy lookup. It must be carefully differentiated from Zhang & Pecora TASE (online coordination with trajectory replanning).
- **C:** open. Schnyder et al. explicitly list learned skills and MuJoCo as future work, and MultiPnP uses learned coarse motion with classical IK, not a learned grasp confined to a reserved region.

### Gaps
- MultiPnP authors and venue, the Zhang & Pecora full texts, the APEX-MR mechanism, and Google Scholar cited-by lists remain unverified.
