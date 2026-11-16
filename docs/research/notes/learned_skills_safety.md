# Learned local manipulation skills inside planner-based (single/multi-arm) systems: handoff, safety, confinement

Scope note: research done 2026-11-16 with about 18 search/fetch calls. Items marked **[fetched]** were checked against the arXiv abstract page or full HTML. Items marked **[snippet]** come only from search-result summaries of the abstract and should be re-checked before citing in a paper. Nothing below comes from memory without a link.

---

## Q1. Annotated set of the most relevant papers (hybrid planner + learned local skill)

### Takeaway
"Motion planner brings the end-effector to a local region, then a learned closed-loop policy finishes the contact-rich part" is a well-established pattern (MoPA-RL 2020, OPTIMUS 2023, PSL 2024, SkillMimicGen 2024, ManipGen 2025). None of these papers has other robots in the workspace, and none gives a safety or confinement guarantee for the learned phase. Handoff is by pose proximity or fixed duration, and the return to the planner happens on a timer or a learned termination classifier, not a safety monitor.

### Cited Findings

**1. ManipGen: "Local Policies Enable Zero-shot Long-horizon Manipulation"** **[fetched]**
- Citation: M. Dalal, M. Liu, W. Talbott, C. Chen, D. Pathak, J. Zhang, R. Salakhutdinov. arXiv 2410.22332 (v2 Mar 2025); the arXiv comments say it was accepted at ICRA 2025. — [arXiv](https://arxiv.org/abs/2410.22332)
- Summary: Local policies are made invariant to absolute robot/object pose, skill order and global scene layout. They are combined with foundation models for vision, language and motion planning. The paper reports 97% zero-shot success on Robosuite tasks and real-world long-horizon tasks of up to 8 stages. — [arXiv](https://arxiv.org/abs/2410.22332)
- Handoff: Neural MP plans a collision-free path to a target pose, and a PID joint controller executes it. "We alternate between motion planning and deploying local policies until the task is complete." The initiation set is ‖X_ee − X_targ‖² < ε. A skill runs for a **fixed duration** (5 s Pick, 4 s Place, ≤8 s Open), then the gripper opens or closes. There is no success- or safety-based termination. — [arXiv HTML](https://arxiv.org/html/2410.22332v2)
- Observations: wrist RealSense D405, 84×84 depth clamped to 30 cm, first-frame depth plus a target segmentation mask, relative EE proprioception. — [arXiv HTML](https://arxiv.org/html/2410.22332v2)
- Data: state-based PPO experts with privileged inputs (16 FPS object keypoints), trained in Isaac Gym, then distilled into visuomotor policies with a **multitask DAgger variant** (replay buffer K=100, B=32). Augmentations are edge noise, holes and random crops. **DART is not used.** — [arXiv HTML](https://arxiv.org/html/2410.22332v2)
- Other robots: none. Safety: no dedicated safety stop. Reported failure modes include "imperfect motion planning, resulting in collisions" and local policies "failing to manipulate from sub-optimal initial poses." Robustness to pose-estimation error is claimed through "significant local randomization" but **not quantified**. — [arXiv HTML](https://arxiv.org/html/2410.22332v2)
- Eval: real world, 10 trials per task, 76% average (SayCan 40%, VoxPoser 16%, LLMTrajGen 14%, OpenVLA 0%). FurnitureBench 90% vs Transic 80%. Robosuite 97.33% vs PSL 96%. — [arXiv HTML](https://arxiv.org/html/2410.22332v2)

**2. OPTIMUS: "Imitating Task and Motion Planning with Visuomotor Transformers"** **[fetched, abstract only]**
- Citation: M. Dalal, A. Mandlekar, C. Garrett, A. Handa, R. Salakhutdinov, D. Fox. CoRL 2023. arXiv 2305.16309. — [arXiv](https://arxiv.org/abs/2305.16309)
- Summary: TAMP can "autonomously generate large-scale datasets of diverse demonstrations." OPTIMUS trains visuomotor Transformer policies by imitating a TAMP agent, using a data pipeline "specifically curated for imitation learning." It reports 70–80% success on tasks with more than 70 objects. — [arXiv](https://arxiv.org/abs/2305.16309)
- Data source: a planner (TAMP) expert, i.e. the same kind of scripted/planner expert as Angle C. Here the learned policy *replaces* the planner rather than sharing control with it. Filtering and handoff details were not on the abstract page (**gap**).
- Other robots: none reported. Safety mechanism: none reported in the abstract.

**3. Plan-Seq-Learn (PSL)** **[fetched, abstract only]**
- Citation: M. Dalal, T. Chiruvolu, D. Chaplot, R. Salakhutdinov. ICLR 2024. arXiv 2405.01534. — [arXiv](https://arxiv.org/abs/2405.01534)
- Summary: Motion planning bridges LLM high-level plans and learned low-level RL control, with no predefined skill library. It works from raw visual input and reports more than 85% success on over 25 tasks with up to 10 stages, across four benchmarks. — [arXiv](https://arxiv.org/abs/2405.01534)
- Handoff: planner to RL policy in a local region (abstract-level; termination details were not fetched, **gap**). Data: online RL. Other robots: none. Safety: none stated.

**4. MoPA-RL: "Motion Planner Augmented RL for Robot Manipulation in Obstructed Environments"** **[fetched, abstract only]**
- Citation: J. Yamada, Y. Lee, G. Salhotra, K. Pertsch, M. Pflueger, G. S. Sukhatme, J. J. Lim, P. Englert. CoRL 2020. arXiv 2010.11940. — [arXiv](https://arxiv.org/abs/2010.11940)
- Summary: The RL action space is augmented with a motion planner. Based on the magnitude of the action, the agent "smoothly transitions between directly executing the action and invoking a motion planner." It reports better learning efficiency, faster exploration and "safer policies that avoid collisions with the environment" in simulation. — [arXiv](https://arxiv.org/abs/2010.11940)
- Shared control is **action-level switching** chosen by the policy, not a spatial handoff. The safety claim is empirical and covers static obstacles only. The threshold and numbers were not fetched (**gap**).

**5. SkillMimicGen / SkillGen and Hybrid Skill Policies (HSP)** **[fetched, full HTML partially]**
- Citation: C. Garrett, A. Mandlekar, B. Wen, D. Fox. CoRL 2024, PMLR v270. arXiv 2410.18907. — [arXiv HTML](https://arxiv.org/html/2410.18907), [PMLR](https://proceedings.mlr.press/v270/garrett25a.html)
- Summary: Human demos are split into skill segments, which are adapted to new scenes and linked with motion-planned transit/transfer. Generated data trains Hybrid Skill Policies that learn skill initiation, a closed-loop skill policy and termination. — [arXiv HTML](https://arxiv.org/html/2410.18907)
- Handoff: this is the **clearest explicit planner↔policy handoff protocol**. The system predicts an initiation pose (HSP-Reg regresses it, HSP-Class classifies a source demo and adapts it, HSP-TAMP uses a TAMP planner). It plans there with TRAC-IK + RRT-Connect + OSC, runs the BC policy, and returns to the planner when a **learned termination classifier** fires. — [arXiv HTML](https://arxiv.org/html/2410.18907)
- Observations: front and **wrist RGB** plus proprioception. Data: MimicGen-style replay from about 10 source demos, keeping successes only, which is a scripted/replayed expert. — [arXiv HTML](https://arxiv.org/html/2410.18907)
- Safety: collisions are checked only for planned transit (ground-truth geometry in simulation, segmented point cloud on the real robot). **No collision handling during the learned skill** is described. Other robots: none. — [arXiv HTML](https://arxiv.org/html/2410.18907)
- Eval: 18 sim variants, HSP-TAMP 85.7% / HSP-Class 82.9% / HSP-Reg 72.6% vs 59.1% for MimicGen-data agents. Data-generation success is 75.4% vs 40.7%. On the real robot: Pick-Place-Milk 95%, Coffee 65% vs 14%. — [arXiv HTML](https://arxiv.org/html/2410.18907)

**6. Multi-Robot Multi-Goal Motion Planning with Stochastic Skills** **[fetched, full HTML]**. This is the **closest prior work to Angle C**.
- Citation: W. Schnyder, V. N. Hartmann, S. Coros (ETH Zürich). arXiv 2610.07784, 6 Oct 2026, with no venue listed. — [arXiv HTML](https://arxiv.org/html/2610.07784)
- Summary: Stochastic manipulation skills are integrated into sampling-based multi-robot planning. The problem is cast as an MDP over a multi-modal composite roadmap. Solving it gives a *reactive* policy that lets the controllable robots adapt to another robot's stochastic skill execution, avoiding the pessimism of conservative open-loop baselines. — [arXiv HTML](https://arxiv.org/html/2610.07784)
- Shared control: the skill robot acts autonomously as a closed-loop controller with a stochastic disturbance, and the planner steers the *other* robots. Skill outcomes are binned into spatio-temporal "nature states," and value iteration gives the policy. Handoff: when a skill ends, an exit value connects the realized final configuration to the next mode's roadmap. — [arXiv HTML](https://arxiv.org/html/2610.07784)
- Confinement: **no reserved region or safe set**. The conservative baseline checks each waypoint against all Monte Carlo skill realizations; the reactive planner uses a precomputed collision mask. The skill controller guarantees only self-collision and static-obstacle avoidance. **Abort and skill failure are not handled** and are listed as future work. — [arXiv HTML](https://arxiv.org/html/2610.07784)
- Skill source: not stated. "Learned skill policies" are listed as future work, and the stacking experiments use a *simulated* pick duration of 1 or 3 s. No physics simulator is used; MuJoCo and hardware are future work. — [arXiv HTML](https://arxiv.org/html/2610.07784)
- Eval: 2–4 six-DoF arms (12–24 DoF). Baselines are conservative open-loop, PP, PRM* and RRT*. Reactive vs conservative realized makespan: −13.3%/−8.1%/−43.4% in planar settings and −19.0 to −24.1% in stacking. But reactive *execution success* in dependency-graph stacking was 69/59/89% vs 80/100/100% for conservative, at 2/3/4 robots. dRRT* is not named (only discrete RRT is cited as an analogy). CBS is not mentioned. — [arXiv HTML](https://arxiv.org/html/2610.07784)

**7. DART: "Noise Injection for Robust Imitation Learning"** **[fetched, abstract only]**
- Citation: M. Laskey, J. Lee, R. Fox, A. Dragan, K. Goldberg. CoRL 2017. arXiv 1703.09327. — [arXiv](https://arxiv.org/abs/1703.09327)
- Summary: Noise is injected into supervisor actions during demonstration so the supervisor shows recoveries (off-policy). The noise level is tuned to approximate the learned policy's error. DAgger is described as tedious and costly, and it can visit dangerous states. — [arXiv](https://arxiv.org/abs/1703.09327)
- Grasping relevance: physical **grasping in clutter** on a Toyota HSR with human supervisors, where DART gave an average 62% performance increase over behavior cloning. On Humanoid, DART was up to 3× faster than DAgger in computation, and DAgger's executed policies had 80% less cumulative reward than the supervisor during training. — [arXiv](https://arxiv.org/abs/1703.09327)
- Safety angle: DART was explicitly motivated by DAgger visiting dangerous states during data collection. This is directly relevant to running DAgger rollouts inside a multi-arm cell. — [arXiv](https://arxiv.org/abs/1703.09327)

**8. GG-CNN: "Closing the Loop for Robotic Grasping"** **[fetched, full HTML]**. This is the key quantitative closed-loop vs open-loop reference; see Q4.
- Citation: D. Morrison, P. Corke, J. Leitner. RSS 2018. arXiv 1804.05172. — [arXiv](https://arxiv.org/abs/1804.05172)
- Summary: A generative grasping CNN predicts per-pixel grasp quality and pose from depth. It runs closed-loop at up to 50 Hz and reports 83% on adversarial unseen objects, 88% on household objects moved during the grasp, and 81% in dynamic clutter. — [arXiv](https://arxiv.org/abs/1804.05172)
- This is a learned grasp predictor inside a visual-servo loop, not a planner handoff. It is single-arm with no other robots.

**9. Provably Safe Deep RL for Robotic Manipulation in Human Environments** **[fetched, abstract only]**
- Citation: J. Thumm, M. Althoff. ICRA 2022. arXiv 2205.06311. — [arXiv](https://arxiv.org/abs/2205.06311)
- Summary: A shield based on fast reachability analysis of the human and the manipulator makes the robot come to a complete stop before a human can reach it. Safety is verified against ISO standards during both training and deployment. The shield also improved RL performance by preventing episode-ending collisions. Evaluated in simulation with human motion-capture data. — [arXiv](https://arxiv.org/abs/2205.06311)
- Per a search summary of the TUM PDF: speed-and-separation monitoring under ISO 10218, and a low-frequency RL agent combined with a high-frequency formal verifier usable with any online RL agent. **[snippet]** — [TUM PDF](https://mediatum.ub.tum.de/doc/1691362/1691362.pdf)
- Relevance: this is the shielding template (reachable sets + fail-safe stop) that Angle C's abort-and-handoff is a simplified, geometric version of. The "other agent" is a human, not another arm.

**10. X-Safe: Embodiment-Agnostic Safeguarding for Robot Arm Manipulation Policies via Action Masking** **[fetched, abstract only]**
- Citation: A. Beaudin, H. Krasowski, K. Nagpal, S. A. Seshia, M. Arcak, N. Mehr. arXiv 2606.22278 (v2 Oct 2026), with no venue listed. — [arXiv](https://arxiv.org/abs/2606.22278)
- Summary: A configuration-space safeguard reasons locally around the current configuration to define safe regions, then *masks* the policy's sampled actions to those regions. It gives formal probabilistic collision-avoidance guarantees without simplified robot models or learned safety functions. It is tested on 2 embodiments and 3 policies, in simulation and on hardware, and shows "less than half the performance degradation" of state-of-the-art safeguards. It criticises stop-based fallbacks as hurting performance. — [arXiv](https://arxiv.org/abs/2606.22278)
- Relevance: this is the strongest current competitor to a "confine by reserved region + abort" design. It is a safe-set filter on a learned manipulation policy. Multi-arm use was not stated in the abstract (**gap**).

**11. Learning a Decentralized Multi-arm Motion Planner** **[snippet]**
- Citation: H. Ha, J. Xu, S. Song. CoRL 2020 (PMLR v155). — [PMLR PDF](https://proceedings.mlr.press/v155/ha21a/ha21a.pdf)
- From the search summary: a shared multi-agent RL policy for decentralized multi-arm planning, using BiRRT expert demonstrations and trained on 1–4 arm tasks with static targets. — [PMLR PDF](https://proceedings.mlr.press/v155/ha21a/ha21a.pdf)
- Relevance: the learned component here is the *multi-arm motion* itself, not a local grasp skill. There is no safety guarantee in the summary.

**12. NeHMO: Neural Hamilton-Jacobi Reachability Learning for Decentralized Safe Multi-Arm Motion Planning** **[snippet]**
- arXiv 2507.13940. From the search summary: it learns neural HJ reachability value functions and embeds them in a trajectory optimizer for decentralized multi-arm pick-and-place in a shared workspace. — [arXiv HTML](https://arxiv.org/html/2507.13940)
- Relevance: a learned *safety* certificate for multi-arm workspaces, not a learned grasp skill. The search also returned a second ID (2607.00326) with the same title; the relationship between them is unclear (**uncertain**).

### Inferences
- The hybrid "planner → learned local skill → planner" loop is **already done** for single-arm systems (ManipGen, SkillGen/HSP, PSL). The ManipGen recipe of privileged expert, multitask DAgger, wrist depth and local randomization is published and reproducible. "ManipGen-style distillation" is therefore not itself a contribution.
- Termination and handoff in this line of work are weak: ManipGen uses a fixed timer and HSP uses a learned classifier. Neither has a safety-triggered abort. That leaves a clear slot for an *explicit, verifiable* abort condition.

### Gaps
- PSL and MoPA-RL termination/threshold details and OPTIMUS filtering details were not fetched beyond the abstracts.
- I did not find papers on *industrial bin-picking cells with learned grasping next to other robots* that publish a confinement mechanism. This was searched only indirectly, so absence is not proven.
- MimicGen (CoRL 2023) itself was not fetched. SkillGen is its successor with motion planning, so it covers the same ground.

---

## Q2. Safety and confinement for learned policies in shared workspaces (CBF, shields, safe sets)

### Takeaway
Safety filters for learned manipulation policies are an active 2025–2026 area: CBF-QP filters, latent CBFs, action masking, and reachability shields. However, they are almost all single-arm, with static obstacles or humans as the hazard. Multi-arm safety work (NeHMO, Ha et al.) learns the *motion*, not a local skill. A *spatial reservation* combined with an *abort-to-planner* design is simpler than CBFs and gives a different kind of guarantee: guaranteed by construction against other arms, with no guarantee about what happens inside the region.

### Cited Findings
- **Reachability shield with full stop**: the shield guarantees human safety during training and deployment and improves RL performance by preventing episode-ending collisions. — [Thumm & Althoff, ICRA 2022](https://arxiv.org/abs/2205.06311)
- **Action masking to local safe regions** in configuration space, with probabilistic guarantees and less than half the degradation of state-of-the-art safeguards. It argues stop-based fallbacks hurt performance. — [X-Safe, arXiv 2606.22278](https://arxiv.org/abs/2606.22278)
- **LatentCBF** filters a visuomotor imitation policy on hardware (a bag-of-candies lift without spilling). It removes unsafe modes but can push the policy out of distribution and lower task success. **[snippet]** — [arXiv 2511.18606](https://arxiv.org/pdf/2511.18606)
- **OSCBF**: an operational-space CBF min-norm safety filter for whole-body safety, used with policies trained on teleoperation data. **[snippet]** — [arXiv 2503.06736](https://arxiv.org/pdf/2503.06736)
- **Neural configuration-space barriers**: a distributionally robust CBF-QP on a 6-DoF xArm that accounts for learned-model and sensor errors. **[snippet]** — [arXiv 2503.04929](https://arxiv.org/html/2503.04929)
- **HCLBF safety filter** from STL specifications wraps an RL arm policy for tabletop obstacle avoidance without retraining. **[snippet]** — [arXiv 2511.14434](https://arxiv.org/pdf/2511.14434)
- **In-Distribution Barrier Functions**: self-supervised policy filters that avoid out-of-distribution states. This is relevant to "leaving the trained region ⇒ abort." **[snippet, title only]** — [arXiv 2301.12012](https://arxiv.org/pdf/2301.12012)
- **Shielded multi-agent RL** found only for a 2-DoF continuum surgical manipulator (one agent per DoF), not for multiple rigid arms. **[snippet]** — [Ji et al., arXiv 2106.07892](https://arxiv.org/abs/2106.07892v1)
- **Multi-arm skill-aware planning** handles the skill robot only through collision masks or Monte Carlo realizations, with no reserved region and no abort handling. — [Schnyder et al., arXiv 2610.07784](https://arxiv.org/html/2610.07784)
- In this line, DAgger's on-policy rollouts are noted to visit dangerous states, which motivates DART. — [DART, arXiv 1703.09327](https://arxiv.org/abs/1703.09327)

### Inferences
- A reserved region (the swept volume of the descent, which other arms' dRRT* plans avoid) plus a containment monitor (abort if the EE or arm leaves the region) is a **geometric shield**. Its guarantee is "no inter-arm collision as long as the monitor reacts before the arm leaves the region." Because the region is reserved for the duration, it does not need reachability of the other arms. That is cheaper than a CBF and composes cleanly with dRRT*'s discrete conflict checking.
- The weak point a reviewer will raise: a stop/abort fallback degrades performance (X-Safe's critique), and a margin is needed for braking distance. Measuring the abort rate and the extra makespan is necessary.
- A natural ablation is "reserved region + abort" vs "CBF/action-masking filter" vs "no confinement + conservative planner (Schnyder-style Monte Carlo sweep)."

### Gaps
- No paper found with a CBF or shield applied to a learned grasp policy *among other moving arms*.
- Quantitative intervention rates for these safety filters were not extracted (only abstracts were read).

---

## Q3. Is there work combining a multi-robot planner (dRRT*, CBS, prioritized) with learned local skills plus a reserved-region / safe-set guarantee?

### Takeaway
**No paper was found that does all of these together.** The nearest is Schnyder, Hartmann & Coros (Oct 2026). They integrate *stochastic* skills into a multi-robot composite-roadmap planner (with PP as a baseline). But the skills are not learned (learning is future work), there is no reserved region (they use collision masks / Monte Carlo realizations), abort and failure are not handled, and there is no physics simulator (MuJoCo is future work). Angle C fills exactly the gaps their future-work section lists. It is also a timing risk: that group may publish the learned-skill extension.

### Cited Findings
- Schnyder et al. list learned skill policies, skill failures such as dropped objects, MuJoCo validation and hardware as future work. Skill duration in their experiments is *simulated* (1 or 3 s). Reactive execution success can fall below the conservative baseline (69/59/89% vs 80/100/100%). — [arXiv 2610.07784](https://arxiv.org/html/2610.07784)
- CMU work by Veerapaneni (2026) on global multi-agent motion planning using local policies, learned or planned. It covers livelock/deadlock and "maintaining solution guarantees with partial planning, including using a learned ML policy." **[snippet; appears to be MAPF-style, not manipulators — uncertain]** — [CMU RI](https://publications.ri.cmu.edu/efficient-multi-agent-motion-planning-using-local-policies)
- CIPHER: hierarchical multi-robot planning with fallback to prioritized decoupled RRT, then composite RRT. It has fallbacks but no learned skills. **[snippet]** — [arXiv 2605.20395](https://arxiv.org/html/2605.20395)
- Ha et al. (learned decentralized multi-arm planner) and NeHMO (learned HJ reachability for multi-arm) learn the inter-arm motion or safety, not a local grasp skill. **[snippet]** — [PMLR v155](https://proceedings.mlr.press/v155/ha21a/ha21a.pdf), [arXiv 2507.13940](https://arxiv.org/html/2507.13940)
- Single-arm hybrid systems (ManipGen, SkillGen) have no other arms and no safety monitor on the learned phase. — [ManipGen](https://arxiv.org/html/2410.22332v2), [SkillGen](https://arxiv.org/html/2410.18907)

### Inferences
- Novelty grading for Angle C's components:
  - *ManipGen-style DAgger-distilled wrist-camera grasp policy*: **already done** (ManipGen; GG-CNN for closed-loop grasping).
  - *DART + DAgger from a scripted/planner expert*: **already done** separately (DART 2017; OPTIMUS/SkillGen for planner experts). Combining them is engineering, not novelty.
  - *Learned local skill inside a planner with handoff*: **already done** single-arm (ManipGen, HSP, PSL, MoPA-RL).
  - *Stochastic skill inside a multi-robot planner*: **partially done** (Schnyder et al. 2026, without learning, without abort, without physics).
  - *Learned skill in a multi-arm planner with reserved-region confinement, containment-violation abort and handoff back to dRRT*, in physics simulation*: **open**, as far as these searches show.
- The paper's contribution should therefore be framed as the *composition and its guarantee*: a reserved-region contract between dRRT* and an untrusted learned skill, with abort semantics and measured cost (makespan, abort rate). It should not be framed as the learned grasp itself.

### Gaps
- I did not search the CBS/MAPF-with-learned-policies literature in depth (e.g. the full Veerapaneni papers), nor work on "shared workspace exclusion zones" in industrial robot cells, which may exist in automation/IEEE CASE venues rather than ML venues.
- No dRRT*-specific follow-up with learned components was found, but this was not searched exhaustively.

---

## Q4. When is closed-loop learned grasping clearly better than open-loop scripted grasping? (quantitative)

### Takeaway
At small pose error, closed-loop and open-loop are statistically equal. GG-CNN (RSS 2018) measured 84% vs 81% and 92% vs 91% (open- vs closed-loop) with no injected error. Closed-loop clearly wins only once end-effector or pose error reaches roughly **8.5–25.5 mm (1σ)** or the object moves during the grasp. In that range open-loop fell to 38% (worst case) vs closed-loop 68–73%. The project's measured error (p99 1.25 mm / 3.4°) is far inside the regime where no benefit is expected. The project's own sweep (scripted success 60–95% at 10–15 mm or 5 mm + 20°) sits right in GG-CNN's benefit regime.

### Cited Findings
- **GG-CNN, error injection**: inaccurate kinematics were simulated by cross-correlating Cartesian velocities, with coefficients ~N(0, σ), σ ∈ {0, 0.05, 0.1, 0.15}, sampled once per grasp. Over the 170 mm open-loop descent from pre-grasp, this equals end-effector precision of **σ = 0, 8.5, 17.0, 25.5 mm**. There were 10 attempts per object per method. — [arXiv HTML](https://arxiv.org/html/1804.05172v2)
- **GG-CNN, no-error baseline**: open-loop 84% (67/80) adversarial and 92% (110/120) household; closed-loop 81% (65/80) and 91% (109/120). — [arXiv HTML](https://arxiv.org/html/1804.05172v2)
- **GG-CNN, with error**: open-loop reached "only 38% grasp success rate in the worst case." Closed-loop reached "68% and 73% … in the worst case" (adversarial and household). "The closed-loop method out-performs the open-loop method in all cases where kinematic errors are present." Per-σ numbers are only in Fig. 8 (**not extracted**). — [arXiv HTML](https://arxiv.org/html/1804.05172v2)
- **GG-CNN, limit**: "unable to update the grasp pose when the gripper is within 70mm of the object." The RealSense cannot give accurate depth closer than about 150 mm, so the last stretch is effectively open-loop. — [arXiv HTML](https://arxiv.org/html/1804.05172v2)
- **GG-CNN, dynamic objects**: 88% success on household objects moved during the grasp attempt. — [arXiv](https://arxiv.org/abs/1804.05172)
- **Grasp-MPC**: open-loop OSC "significantly drops with perturbed grasp poses," while Grasp-MPC reached 60.3% (a 14% drop) under perturbation. **[snippet; perturbation magnitude not extracted]** — [arXiv 2509.06201](https://arxiv.org/pdf/2509.06201)
- **VFAS-Grasp**: the final approach was kept *open-loop* because the wrist camera cannot give a reliable point cloud once the object geometry passes the fingertips. **[snippet]** — [arXiv 2310.18459](https://arxiv.org/pdf/2310.18459)
- **ManipGen**: robustness to pose-estimation error is claimed through local randomization but not quantified. Policies fail "from sub-optimal initial poses." — [arXiv HTML](https://arxiv.org/html/2410.22332v2)

### Inferences
- With the project's measured noise (p99 1.25 mm / 3.4°) and a 100% scripted baseline, a learned policy **cannot win on success rate**. At best it ties, as in GG-CNN's no-error result. Evaluating "under measured perception noise" alone gives a null result. The evaluation must sweep injected error into the 10–25 mm / ≥20° range, where the project's own sweep shows scripted success at 60–95%. Adding object motion or disturbance during descent (GG-CNN's dynamic case) also helps.
- The geometry matters: 70 mm opening on a 50 mm cube leaves ±10 mm lateral slack. GG-CNN's 8.5–25.5 mm σ range straddles that slack, which is consistent with the project sweep's breakdown at 10–15 mm.
- The wrist-camera near-field blind spot (GG-CNN at 70 mm; VFAS) is a real design risk. MuJoCo rendered depth has no minimum range, so simulation will overstate closed-loop benefit unless a minimum-depth clamp or near-range dropout is simulated. ManipGen clamps the *maximum* depth at 30 cm, but no minimum-depth clamp is reported.
- Honest framing: the learned skill's value in Angle C is *robustness at degraded perception*, such as a cheaper camera, occlusion by other arms, calibration drift or a bumped object. That robustness costs a confinement mechanism, which is what the paper is about.

### Gaps
- No paper found that reports a full success-vs-pose-error curve for closed-loop learned vs scripted open-loop parallel-jaw grasping of a known-geometry object in simulation. GG-CNN's per-σ values are only in a figure.
- No quantitative orientation-error (degrees) comparison was found.

---

## Q5. Synthesis: novelty and feasibility of Angle C

### Takeaway
**Novelty: partially done / open.** Every component exists. The combination (learned local grasp skill + multi-arm dRRT* + reserved-region confinement + containment-triggered abort and handoff, evaluated in physics) was not found. The closest work (Schnyder et al., Oct 2026) names the missing pieces as future work. **Feasibility: high on engineering, medium on result strength.** The main risk is that at the measured noise level the learned policy brings no success gain, while the reservation and abort add makespan.

### Cited Findings
- The hybrid planner + learned local policy pattern with wrist depth and DAgger distillation is established. — [ManipGen](https://arxiv.org/html/2410.22332v2), [SkillGen](https://arxiv.org/html/2410.18907), [PSL](https://arxiv.org/abs/2405.01534), [MoPA-RL](https://arxiv.org/abs/2010.11940)
- Planner-expert imitation is established. — [OPTIMUS](https://arxiv.org/abs/2305.16309)
- DART noise for robust imitation is established, including grasping in clutter (+62% vs BC). — [DART](https://arxiv.org/abs/1703.09327)
- Multi-robot planning with stochastic skills exists, but without learned skills, reserved regions, abort handling or physics. — [Schnyder et al.](https://arxiv.org/html/2610.07784)
- Safety filters for learned manipulation policies exist only in single-arm, human or static-obstacle settings. — [Thumm & Althoff](https://arxiv.org/abs/2205.06311), [X-Safe](https://arxiv.org/abs/2606.22278)
- Closed-loop beats open-loop only under significant pose error (≥8.5 mm σ) or moving objects. — [GG-CNN](https://arxiv.org/html/1804.05172v2)

### Inferences
- **What is new (claimable):** (1) a reserved-region contract between a multi-robot sampling planner and an untrusted learned skill, with a provable "no inter-arm collision" property when the containment monitor and braking margin hold; (2) abort-and-handoff semantics, where the planner re-plans from the abort configuration with event-driven replanning; (3) empirical cost of confinement (makespan, abort rate, region size vs throughput) in a 4-arm physics cell; (4) a closed-loop vs scripted comparison across a *calibrated* perception-error sweep, anchored to measured overhead-camera error.
- **What is not new (do not claim):** DAgger/DART distillation, wrist-camera grasp policies, planner-to-pre-grasp handoff.
- **Feasibility:** The ingredients are in place: MuJoCo 4×UR10e, dRRT* with event-driven replanning, a scripted expert at 100%, and a measured noise model. The ManipGen recipe is simpler here, with one object class (a cube) and one skill, so a scripted expert can replace PPO teachers. Estimated risks:
  1. *Null result at measured noise.* Mitigation: make the error sweep (0 to 25 mm, 0 to 20°) and disturbance/occlusion conditions the main experiment.
  2. *Reserved region too conservative.* The swept descent volume blocks neighbours. Mitigation: report the region size vs makespan trade-off, and compare to a Schnyder-style Monte Carlo conservative baseline.
  3. *Abort rate.* If the policy corrects laterally by 10–15 mm, the region must include that correction envelope or the policy will trigger aborts exactly when it is useful. The region should be "descent sweep ⊕ correction margin," and the margin is a key parameter.
  4. *DAgger rollouts in a shared cell.* Collect them in single-arm simulation (safe), which DART's safety motivation supports.
  5. *Wrist-camera near-field blind spot.* Simulate a minimum depth to avoid overstating the benefit.
  6. *Timing.* Schnyder et al. explicitly list learned skills and MuJoCo as next steps, so the novelty window may be short.
- **Strongest reviewer objection to pre-empt:** "Why not a CBF / action-masking filter (X-Safe) instead of reservation + abort?" Answer with an ablation, or argue composability with the discrete conflict model of dRRT* plus zero runtime cost for the other arms.

### Gaps
- No direct evidence on how large the correction envelope of a DAgger-distilled grasp policy typically is, which determines the region size. This must be measured in the project.
- Venue status of 2026 preprints (Schnyder et al., X-Safe) is unknown.
