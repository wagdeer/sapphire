# B2 loop eligibility: bounded literature, source and counterexample investigation

**Research proposal for review — 2026-09-28. No eligibility policy is finalized. No production code or repository tests were changed. B2/B3 remain unimplemented.** This supplements [B2_DESIGN](B2_DESIGN.md); its earlier global-gap and visual-prefix assumptions are explicitly provisional.

The best minimal candidate is **C: recent-predecessor exclusion only inside the same type-0 odometry chain, followed by existing spatial/visual retrieval and authoritative BBS/GICP**. A second experiment is C plus optional topology-based verification ordering. The investigation does **not** justify replacing the global gap with a universal hard shortest-path threshold. C is an invariant-based adaptation to Sapphire, not an algorithm proven superior by the cited papers or by real-data evaluation.

Evidence levels used below: **source** = inspected implementation; **publication** = primary paper; **synthetic** = this investigation's executable counterexample; **inference** = proposed Sapphire consequence. No downloaded SLAM system was built or benchmarked. Research samples current default-branch source at pinned commits, not necessarily the authors' paper-release implementation.

## 1. Literature matrix and chronology

Graph types: **A** pose/constraint graph; **B** semantic scene graph; **C** per-place descriptor graph; **D** navigation/traversability graph. **A-v** below is a pose-node *covisibility* graph: edges mean shared observations, not odometry constraints. It is neither a semantic scene graph nor a navigation graph. “Multi-session unestablished” means the inspected evidence does not establish that capability or policy, not that the method could never support it.

| Work | Publication year | Graph type | Loop candidate rule | Metric proximity? | Graph/topological distance? | Semantics? | Multi-session? | Open source? | Directly reusable idea |
|---|---:|---|---|---|---|---|---|---|---|
| [S-Graphs 2.0](https://arxiv.org/abs/2502.18044) | 2025 RA-L | A+B | Same floor, adequate accumulated travel, XY proximity; registration | Yes | Floor relation and path travel; no candidate pose-graph shortest path in inspected selector | Structural floor semantics | Unestablished for this selector | [Repository](https://github.com/snt-arg/lidar_situational_graphs) | Distinguish reliable domain exclusions from spatial tests; hierarchy only if supported by observations |
| [SG-SLAM](https://arxiv.org/abs/2503.11145) | 2025 IROS | B+C, A backend | Old descriptor prefix, descriptor KNN, semantic/geometric verification, first success | Descriptor geometry; not a global pose-radius gate in SearchLoop | Per-scan instance relationships, **not** historical pose-hop eligibility | Yes | Relocalization; independent-session policy unestablished | [Repository](https://github.com/nubot-nudt/SG-SLAM) | Retrieval/verification separation; no solution to cross-chain recent-ID exclusion |
| [SGLC](https://arxiv.org/abs/2407.08106) | 2024 RA-L | B+C | Scan descriptor/graph correspondences, geometric checks, coarse/fine/refine registration | Within-place geometry and estimated relative translation | Descriptor topology, not pose-graph geodesic gating | Yes | Pair matching does not establish session management | [Repository](https://github.com/nubot-nudt/SGLC) | Better representation/registration is a different problem |
| [Semantic graph GAT](https://arxiv.org/abs/2501.19382) | 2025 JIRS | B+C, A backend | Learned graph comparison; supplied ROS wrapper prefilters by travel/proximity and caps candidates | Yes in wrapper | Graph used by descriptor; wrapper uses accumulated trajectory travel | Yes | Unestablished | [Repository](https://github.com/crepuscularlight/SemanticLoopClosure) | Separate learned similarity from historical eligibility and budget |
| [MOLA original](https://ingmec.ual.es/~jlblanco/papers/blanco2019mola_rss2019.pdf) | 2019 RSS | A | Spatially near and sufficiently distant through WorldModel Dijkstra | Yes | Yes; historical edge weights not established here | No | Generic graph idea; session-specific exclusion unestablished | [Repository](https://github.com/MOLAorg/mola) | Metric plausibility plus constraint novelty |
| MOLA current Simplemap / frame loop closure | Current source, not a new 2024/25 paper | A (submaps/frames) | Submap: ≥2 unit hops plus uncertainty-aware overlap; frame: frame separation plus metric interval and budget ranking | Yes | Submap: actual graph hops; frame: index separation called topological gap | No | Preassembled graph/map; no Sapphire-like chain rule established | [Repository](https://github.com/MOLAorg/mola_sm_loop_closure) | Separate hard adjacency, overlap scoring and verification scheduling |
| [ORB-SLAM](https://doi.org/10.1109/TRO.2015.2463671), [ORB-SLAM2](https://arxiv.org/abs/1610.06475), [ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644) | 2015 / 2017 / 2021 | A-v plus essential/pose graph | Exclude connected KFs; covisibility score/group support; ORB3 additionally rejects overlapping candidate neighborhoods | No global metric-radius prerequisite for BoW proposal | Neighborhood membership; **no upstream universal shortest-path threshold found** | No | ORB3 Atlas/merge yes | [ORB2](https://github.com/raulmur/ORB_SLAM2), [ORB3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | Distinguish exclusion, support, neighborhood expansion and geometric confirmation |
| stella_vslam / OpenVSLAM lineage | Current implementation, not a 2025 paper | A-v + spanning tree/loop graph | Default connected-KF exclusion; optional depth traversal threshold 50 | No global radius for BoW search | Optional parent/children/loop-edge traversal | No | Map reuse; independent-session policy not established here | [Repository](https://github.com/stella-cv/stella_vslam) | Concrete warning about configurable hard graph neighborhoods |
| [RTAB-Map multi-session](https://arxiv.org/abs/2407.15305), [extended system](https://arxiv.org/abs/2403.06341) | **2014 / 2019**, not 2024 | A; navigation is a separate D use | Appearance Bayesian hypotheses outside short-term memory; geometry; retrieve neighboring history after a match | Proximity branch yes; global appearance not restricted to pose radius | Neighborhood prior/retrieval, not just hard distance | No | Yes | [Repository](https://github.com/introlab/rtabmap) | Keep inter-session proposals possible; expand around a successful historical match |
| [Loop Closure Prioritization / LAMP](https://www-robotics.jpl.nasa.gov/media/documents/Loop_Closure_Prioritization_for_Efficient_and_Scalable_Multi-Robot_SLAM.pdf) | 2022 RA-L | A, with learned graph utility | Rank batches using graph uncertainty, point-cloud observability and beacon evidence | Yes in generation/other modules | Graph-conditioned utility, not a universal hop gate | No semantic labels; learned graph model | Multi-robot yes | [Repository](https://github.com/NeBula-Autonomy/LAMP) | Budget prioritization is distinct from eligibility |

Verified publication distinctions:

- S-Graphs 2.0: first arXiv 2025-02-25, revised 2025-10-03; RA-L 2025, DOI [10.1109/LRA.2025.3619744](https://orbilu.uni.lu/bitstream/10993/66413/1/S-Graphs_2.0__A_Hierarchical-Semantic_Optimization_and_Loop_Closure_for_SLAM.pdf). SG-SLAM: first arXiv 2025-03-14; authors' repository records IROS 2025 acceptance.
- SGLC: first arXiv 2024-07-11; RA-L 9(12), 11545–11552, 2024, DOI [10.1109/LRA.2024.3495455](https://doi.org/10.1109/LRA.2024.3495455).
- GAT: [published online 2025-01-11](https://www.research-collection.ethz.ch/server/api/core/bitstreams/f8cee95d-c648-4adf-8bcb-3c360296026a/content), arXiv 2025-01-31, JIRS 111:13. The supplied repository's HEAD dates to **2023**; do not label that code a verified 2025 publication release.
- RTAB-Map's multi-session paper is IROS **2014** (arXiv 2024-07-22); its extended system paper is JFR **2019** (arXiv 2024-03-10). These are particularly relevant examples of misleading recent upload dates.
- ORB2: 2016 arXiv / 2017 journal; ORB3: 2020 arXiv / 2021 journal. MOLA's original mechanism is RSS 2019; current MOLA code and its 2025 odometry work must not be presented as the same implementation.

### Current source sampling / maintenance evidence

The following are **author dates of the sampled HEAD commits**, checked against each patch's `From` SHA, observed 2026-09-28. They indicate source revision age, not a guarantee of active support, latest issue activity, or latest release. Full SHAs and source hashes are in the audit directory.

| Repository | Sampled HEAD | HEAD author date |
|---|---|---|
| [MOLAorg/mola](https://github.com/MOLAorg/mola/commit/f71bb7c8b0ba60170b546631634bf62351c09846) | `f71bb7c8b0ba` | Sun, 27 Sep 2026 00:47:35 +0200 |
| [MOLAorg/mola_sm_loop_closure](https://github.com/MOLAorg/mola_sm_loop_closure/commit/1cd5fb89471a175f77c71744001bffee26a25844) | `1cd5fb89471a` | Fri, 25 Sep 2026 09:52:20 +0200 |
| [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3/commit/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4) | `4452a3c4ab75` | Thu, 10 Feb 2022 09:26:50 +0100 |
| [crepuscularlight/SemanticLoopClosure](https://github.com/crepuscularlight/SemanticLoopClosure/commit/cff7c147606a38daf459a16a7f3b93070bd6ad02) | `cff7c147606a` | Sat, 28 Oct 2023 16:17:06 +0200 |
| [introlab/rtabmap](https://github.com/introlab/rtabmap/commit/66c72be7db6863b65fafbc7fa134148503e0b815) | `66c72be7db68` | Sat, 26 Sep 2026 21:07:59 -0700 |
| [nubot-nudt/SG-SLAM](https://github.com/nubot-nudt/SG-SLAM/commit/633607e66a9e4eb853209b6c45e41bbe3a05f1b9) | `633607e66a9e` | Fri, 24 Oct 2025 08:48:41 +0800 |
| [nubot-nudt/SGLC](https://github.com/nubot-nudt/SGLC/commit/20f1e5d615fd4269520576b6f7f75aab8a7f5506) | `20f1e5d615fd` | Mon, 3 Mar 2025 08:44:59 +0800 |
| [raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2/commit/f2e6f51cdc8d067655d90a78c06261378e07e8f3) | `f2e6f51cdc8d` | Tue, 10 Oct 2017 21:43:35 -0700 |
| [snt-arg/lidar_situational_graphs](https://github.com/snt-arg/lidar_situational_graphs/commit/35dd3561730af299a0cc2730e3e3ce44a0699c06) | `35dd3561730a` | Mon, 6 Jul 2026 14:29:37 +0200 |
| [stella-cv/stella_vslam](https://github.com/stella-cv/stella_vslam/commit/e445b5452535e781fdf6777ec33a06f5f8f5e416) | `e445b5452535` | Wed, 26 Aug 2026 20:41:04 +0900 |

MRPT's sampled HEAD is `12c0de08e0f763fef66c3f7b2122dae0a15b5ee2`; its merge patch describes another commit, so no HEAD date is inferred from that patch. LAMP was inspected through the official web repository/paper; its HEAD date was not established in this bounded pass.

## 2. What the seed implementations actually do

### S-Graphs 2.0: structural semantics, not chain distance

`LoopDetector::find_candidates` first applies a distance-since-last-loop throttle, then minimum accumulated trajectory travel, equal `floor_level`, and maximum distance in estimated **XY**. Matching follows. Floor labels therefore act as **hard exclusions**. The paper's keyframe/wall/room/floor hierarchy also controls optimization scope; that is separate from candidate selection. [Selector source](https://github.com/snt-arg/lidar_situational_graphs/blob/35dd3561730af299a0cc2730e3e3ce44a0699c06/lidar_situational_graphs/include/s_graphs/frontend/loop_detector.hpp#L230), [floor analyzer](https://github.com/snt-arg/lidar_situational_graphs/blob/35dd3561730af299a0cc2730e3e3ce44a0699c06/lidar_situational_graphs/src/s_graphs/frontend/floor_analyzer.cpp).

The reusable principle is to reject candidates only using a well-founded relationship. Correct floor identity can disambiguate similar floors. Sapphire currently has no equivalent floor-observation contract. Z clustering alone is unreliable on stairs, ramps and drifted multi-session maps. Importing semantic scene ownership or inferring hard floor regions merely to fix SQL temporal indexing is unjustified. Floor detection can exploit geometry; it does not necessarily mean adding a neural semantic segmentation network, but it still adds a new inferred environmental label.

### SG-SLAM: semantic topology does not replace temporal indexing

`SemGraphMapping::mainProcess` computes `num_exclude_curr = 150 / keyframe_interval`; `SearchLoop` builds a descriptor-search vector ending before that recent window. It runs descriptor KNN, thresholds **descriptor distance**, calls `GeometryVeriPoseEstimation`, and stops at the first success. This is not a shortest path through historical pose constraints. [Mapping source](https://github.com/nubot-nudt/SG-SLAM/blob/633607e66a9e4eb853209b6c45e41bbe3a05f1b9/cpp/semgraph_slam/pipeline/SemGraphMapping.cpp#L92).

Instance centers/labels and geometric edges support matching and relocalization. Accepted loops become GTSAM constraints in the ROS backend; optimized poses subsequently update semantic map instances. The map's semantic connectivity and the backend constraint graph have different meanings. [ROS backend](https://github.com/nubot-nudt/SG-SLAM/blob/633607e66a9e4eb853209b6c45e41bbe3a05f1b9/ros/ros2/semSLAM.cpp), [graph frontend](https://github.com/nubot-nudt/SG-SLAM/blob/633607e66a9e4eb853209b6c45e41bbe3a05f1b9/cpp/semgraph_slam/frontend/SemGraph.cpp).

The retrieval/verification split is directly reusable in principle, but Sapphire already has it. Removing labels would change SG-SLAM's representation and verification, not expose a ready-made multi-chain temporal filter. Its hard recent-window implementation is evidence **against assuming newer graph-based systems have solved this specific problem**.

### SGLC: descriptor topology and registration

`SemanticGraph::loop_pairs_similarity` builds instance graphs, descriptors and correspondences; it prunes outliers and checks graph/background compatibility. `loop_poses_estimation` performs alignment/refinement. These operate on a pair of scans, not on paths through the historical constraint graph. [Pair matching and estimation](https://github.com/nubot-nudt/SGLC/blob/20f1e5d615fd4269520576b6f7f75aab8a7f5506/semgraph/SemanticGraph.cpp#L239).

The inspected `eval_lcd_seq.cpp` reads supplied scan pairs from a file and evaluates each pair, so that executable does not establish an online candidate-eligibility policy. [Evaluation driver](https://github.com/nubot-nudt/SGLC/blob/20f1e5d615fd4269520576b6f7f75aab8a7f5506/src/eval_lcd_seq.cpp). The paper's graph-guided coarse/fine/refine mechanism is useful registration research, but replacing Sapphire's accepted BBS/GICP path is outside this task. A graph descriptor is not evidence for a hard pose-hop rule.

### Semantic GAT: learned place representation plus a separate heuristic wrapper

The paper uses attention to encode semantic/spatial/geometric relationships within a scan and compare graph vectors. That graph is type C/B, not a historical type-A distance. [Paper](https://arxiv.org/abs/2501.19382).

The supplied `SGPR.loop_detection` wrapper accumulates odometry translation, requires travel difference >20 and current positional separation <0.02 times that travel difference, then randomly subsamples to 64 candidates if needed. Those are **hard heuristic spatial/travel gates plus a budget**, outside the neural graph comparator. They are neither a calibrated Bayesian prior nor a multi-chain metric. [Wrapper](https://github.com/crepuscularlight/SemanticLoopClosure/blob/cff7c147606a38daf459a16a7f3b93070bd6ad02/src/SGPR.py#L68), [model/configuration layout](https://github.com/crepuscularlight/SemanticLoopClosure/blob/cff7c147606a38daf459a16a7f3b93070bd6ad02/README.md). No justification was found for porting its neural/semantic pipeline to fix B2 eligibility.

### MOLA: three mechanisms that must stay separate

**2019 publication:** metric proximity plus WorldModel Dijkstra distance gates candidate registration; expensive checks are queued. The paper establishes the pattern but does not specify enough in the inspected passage to settle edge weights, loop-edge costs or a session-root policy. Current root repository no longer exposes the historical WorldModel paths tried in this investigation; no claim of exact 2019 weights is made. [Original paper](https://ingmec.ual.es/~jlblanco/papers/blanco2019mola_rss2019.pdf).

**Current `SimplemapLoopClosure::find_next_loop_closure`:** run Dijkstra from each submap root on `state_.submapsGraph` without a weight callback; use the resulting shortest-path tree depth. MRPT explicitly defaults every edge to cost **1**. Non-GNSS pairs at depth ≤1 are rejected; the exception involves the georeference submap. Covariance is propagated along the tree and sampled to estimate AABB overlap. Overlap scores rank remaining pairs. Already-checked pairs are skipped. [Selector](https://github.com/MOLAorg/mola_sm_loop_closure/blob/1cd5fb89471a175f77c71744001bffee26a25844/module/src/SimplemapLoopClosure.cpp#L1100), [MRPT default edge cost](https://github.com/MRPT/mrpt/blob/12c0de08e0f763fef66c3f7b2122dae0a15b5ee2/modules/mrpt_graphs/include/mrpt/graphs/dijkstra.h#L165).

Odometry and accepted ICP submap links both enter this graph (`insertEdge`, including `lambdaAddIcpEdge`). There is no zero-cost or low-cost loop-edge rule here. Accepted links can shorten future paths. A newly linked pair becomes adjacent, but **a two-hop corroborating pair remains eligible** under the actual ≥2 gate. Do not attribute suppression by a hypothetical >3 or >50 threshold to this implementation. Its all-roots submap search, uncertainty sampling and offline workflow are not a constant-cost online Sapphire candidate check. [Edge insertion](https://github.com/MOLAorg/mola_sm_loop_closure/blob/1cd5fb89471a175f77c71744001bffee26a25844/module/src/SimplemapLoopClosure.cpp#L1450).

**Current `FrameToFrameLoopClosure::find_loop_candidates`:** minimum frame-index separation and a metric-distance interval generate pairs. Block-pair deduplication, proximity/stratified/multi-objective strategies choose a bounded set. A comment calling index separation “topological gap” does **not** make it shortest-path distance. The C++ parameter default is distance-stratified; the shipped pipeline selects multi-objective. [Frame selector](https://github.com/MOLAorg/mola_sm_loop_closure/blob/1cd5fb89471a175f77c71744001bffee26a25844/module/src/FrameToFrameLoopClosure.cpp#L1075), [parameters](https://github.com/MOLAorg/mola_sm_loop_closure/blob/1cd5fb89471a175f77c71744001bffee26a25844/module/include/mola_sm_loop_closure/FrameToFrameLoopClosure.h), [pipeline](https://github.com/MOLAorg/mola_sm_loop_closure/blob/1cd5fb89471a175f77c71744001bffee26a25844/pipelines/loop-closure-f2f-lidar3d-gicp.yaml).

A useful practical negative result appears in the [2026 changelog](https://github.com/MOLAorg/mola_sm_loop_closure/blob/1cd5fb89471a175f77c71744001bffee26a25844/CHANGELOG.rst): an index-midpoint coverage score clustered candidates spatially on non-linear trajectories; it was replaced by spatial-midpoint coverage, and candidate deduplication/greedy rescoring were corrected. That supports measuring spatial coverage and actual verification utilization. It is **other-system evidence**, not ORB evidence and not proof that all graph-distance gates fail.

## 3. Lessons from ORB-SLAM-style topological filtering

### Exact mechanisms, defaults and provenance

| Implementation | Graph / operation | Hard gate, support or rank? | Default / active path |
|---|---|---|---|
| ORB2 `KeyFrameDatabase::DetectLoopCandidates` | Query's connected-KF set; shared landmark/covisibility relationships | Excludes directly connected KFs, then BoW and covisibility score aggregation | Active in standard loop detection; no configurable minimum shortest-path distance here |
| ORB2 `LoopClosing::DetectLoop` | Candidate plus connected KFs intersect prior candidate groups | Temporal consistency filter, not graph distance | `mnCovisibilityConsistencyTh=3`; counter begins at 0, so threshold 3 means repeated increments, not “3 edges.” Also suppresses processing within 10 IDs of last loop |
| ORB3 `Run → NewDetectCommonRegions → DetectNBestCandidates` | Connected KFs excluded in BoW DB; top covisible scores accumulate | Hard local exclusion + ranking | Up to 3 loop/merge BoW candidates requested; normal loop processing enabled by default |
| ORB3 `DetectCommonRegionsFromBoW` | Candidate plus best 10 covisible neighbors intersects query connected set | Hard rejection of the candidate neighborhood (`bAbortByNearKF`) | Active on fresh BoW-candidate path; no standalone distance-threshold switch |
| ORB3 geometric region continuation | Previous common-region hypothesis projected/verified; support from nearby KFs | Geometric confirmation and expansion | Coincidence count ≥3 in continuation; spatial covisible support also used. Constructor's inherited `mnCovisibilityConsistencyTh=3` is not the old ORB2 group filter running unchanged |
| stella_vslam `detect_loop_candidates_impl` | Default covisible neighbors; optional spanning parent + children + stored loop edges | Hard exclusion before BoW, followed separately by continuity filter | `reject_by_graph_distance=false`, `min_distance_on_graph=50`, `min_continuity=3`; AIST equirectangular example explicitly enables graph-distance rejection |

Source anchors: [ORB2 DB](https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/KeyFrameDatabase.cc#L76), [ORB2 loop](https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/LoopClosing.cc#L103), [ORB3 DB](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/KeyFrameDatabase.cc#L604), [ORB3 fresh-candidate gate](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LoopClosing.cc#L578), [ORB3 system toggle](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L101), [stella selector](https://github.com/stella-cv/stella_vslam/blob/e445b5452535e781fdf6777ec33a06f5f8f5e416/src/stella_vslam/module/loop_detector.cc#L72), [stella standard AIST configuration](https://github.com/stella-cv/stella_vslam/blob/e445b5452535e781fdf6777ec33a06f5f8f5e416/example/aist/equirectangular.yaml#L40).

ORB3 `KeyFrame::GetConnectedKeyFrames` returns keys of `mConnectedKeyFrameWeights`. `UpdateConnections` counts shared map-point observations. Its threshold 15 shapes ordered/reciprocal covisibility links, while the full counter map is assigned to `mConnectedKeyFrameWeights`; treating the returned set as uniformly “≥15 shared features” is inaccurate. It is not an odometry-neighbor set. Loop correction includes map-point fusion and connection updates, so covisibility can change following a closure. The separate stored loop edge is not directly traversed by upstream `bAbortByNearKF`. [KeyFrame implementation](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/KeyFrame.cc#L379), [loop correction](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LoopClosing.cc#L973).

ORB3 defaults `loopClosing` to true; disabling it disables place recognition/merging as well. Small-map and inertial-initialization checks additionally delay detection. Thus “normally active” means the local neighborhood gate runs when the ordinary fresh-BoW path is reached, not on every frame or every continuation hypothesis. No claim is made that a setting with graph threshold 50 exists in upstream ORB3. [System](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L101), [entry path](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LoopClosing.cc#L324).

stella's documentation calls its optional graph an “essential graph,” but the inspected traversal visits **parent, loop edges, children**; it does not iterate all covisibility/strong essential-graph edges. It uses a **LIFO vector**, visited-on-discovery, and depth limit, not BFS/Dijkstra. With loops, the discovered depth need not be minimum distance. The audit reproduces a node at shortest distance 3 that escapes a threshold-4 rejection because a longer route discovers an intermediate node first. This is an algorithm-level counterexample, not a binary execution or a claim of observed user failures. [Traversal source](https://github.com/stella-cv/stella_vslam/blob/e445b5452535e781fdf6777ec33a06f5f8f5e416/src/stella_vslam/module/loop_detector.cc#L72).

stella is an OpenVSLAM-derived implementation in the ORB-style lineage, not upstream ORB3. This pass establishes presence in current stella and absence from the inspected upstream ORB2/3 selectors; it does not establish the first historical commit that introduced the switch.

### Practical reports: what is and is not supported

- [ORB3 issue #109](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/109) asks to disable loop correction for a **pure-VIO comparison**. It is not evidence that graph-neighborhood filtering performed badly.
- [ORB3 PR #745](https://github.com/UZ-SLAMLab/ORB_SLAM3/pull/745) and its community-fork discussion address candidate scoring/iterator/Sim3 bugs. They demonstrate real maintenance concerns, not a recommendation to remove a graph-distance gate.
- stella's [AIST configuration](https://github.com/stella-cv/stella_vslam/blob/e445b5452535e781fdf6777ec33a06f5f8f5e416/example/aist/equirectangular.yaml#L40) and [discussion #504](https://github.com/stella-cv/stella_vslam/discussions/504) / [#611](https://github.com/stella-cv/stella_vslam/discussions/611) show the optional graph filter **enabled** at 50 in actual reported configurations. These discussions concern crashes/instructions, not evidence of loop-recall damage caused by the gate.
- Source default off and a prominent example on are mixed operational evidence. The bounded search found **no credible basis for “practitioners often disable it because topology filtering is poor,” much less “everyone disables it.”** No field report isolated keyframe-density sensitivity or corroboration suppression as the reason for changing this option.

Search scope covered official ORB3 issues/PRs, linked community fixes, stella code/configuration/discussions, and targeted queries for `bAbortByNearKF`, `reject_by_graph_distance`, `min_distance_on_graph`, loop-disable, covisibility and threshold failures. Search results are not a representative deployment survey. The negative result is retained rather than converted into a popularity claim.

### Failure principles and “do not repeat” rules

| Rule for Sapphire | Evidence classification | Reason and transfer limit |
|---|---|---|
| Keep graph meaning explicit; do not transplant ORB's neighborhood size or stella's 50 | **Direct ORB-style source evidence**, Sapphire inference | Covisibility measures shared observations; Sapphire type-0 means odometry succession and type-1 means estimated geometric constraint |
| Do not interpret connectivity as temporal adjacency | **Sapphire invariant + synthetic** | A B1 attachment joins chains geometrically without creating a cross-session odometry predecessor |
| Do not use raw edge count as a universal physical distance | **Inference**, with source-defined edge semantics | Variable keyframe density, branching visibility and sessions make hops incomparable in metres; this is not a demonstrated ORB field failure |
| Do not let one inserted loop automatically forbid all nearby corroboration | **stella/MOLA source mechanism + synthetic Sapphire inference** | Traversal over stored loop edges changes distances; upstream ORB3 changes covisibility through fusion instead. Actual MOLA ≥2 still allows two-hop corroboration |
| Do not treat historical type-1 edges as unquestionable eligibility truth | **Inference + wrong-edge synthetic case** | The graph-only gate produces identical decisions for true and false edges having the same endpoints; it cannot diagnose geometric correctness |
| Do not hard-reject good retrieved evidence solely because an uncertain graph says “near” without an explicit invariant | **Inference**, informed by ORB hard gates and RTAB/LAMP alternatives | Recent same-chain redundancy is an explicit baseline policy; arbitrary type-1-induced neighborhood size lacks such a guarantee |
| Do not call a bounded DFS “shortest-path distance” | **Direct stella source + executable counterexample** | First-discovery depths can depend on traversal order when there are cycles |
| Do not equate soft ranking with guaranteed verification | **Synthetic + LAMP scheduling principle** | A finite budget or first-success stop can still starve down-ranked corroboration |
| Keep same-chain exclusion separate from graph novelty | **Sapphire inference** | C fixes inter-chain semantics independently of optional novelty ranking; C's K still needs density/overlap evaluation |

Physical closeness and graph distance answer different questions. A slow, feature-rich pass may insert many KFs over a short path; a sparse pass may span that same path in two edges. A disconnected session has no finite graph path before alignment. A wrong loop can create a shortcut; a correct loop can also make useful repeated geometric observations appear redundant. The source establishes mechanisms capable of these effects; the synthetic tests establish counterexamples, **not their prevalence in ORB deployments**.

The evidence argues against adopting **naïve hard T as a Sapphire default**, not against all topology use. Reordering eligible proposals preserves the candidate set, but only fair scheduling or exhaustive verification preserves access to every candidate. With B2's first-accepted-loop behavior, even an unlimited attempt budget does not verify candidates after the first success. No claim that soft ranking alone eliminates estimation-error feedback is warranted.

## 4. Policy comparison and exact distance alternatives

Let `R(q)` be the union of existing spatial and visual proposals for transient query q. Targets must be committed and older than q. K=3 below is the existing baseline, held constant to isolate chain semantics; this investigation does not tune or finalize K.

| Policy | Hard eligibility | Benefits | Failure / cost / ORB lesson |
|---|---|---|---|
| **G** | `t<q && q-t>K` | O(1); simple prefix | Suppresses other-chain 99/98 for q101. SQL adjacency is neither odometry nor covisibility |
| **C** | `t<q && (chain(t)!=chain(q) || d0(q,t)>K)` | Fixes exact multi-chain counterexample; type-1 errors cannot change temporal membership; O(1) active-tail predicate | More redundant inter-chain proposals; recent same-chain true loops still excluded; K depends on sampling |
| **T** | Candidate and full-graph distance >threshold | Can avoid rechecking existing graph neighborhoods | Loop-induced shortcuts, wrong-edge feedback, density/threshold sensitivity; same-chain temporal exclusion not equivalent; BFS O(V+E) per query |
| **M-hard** | C plus spatial proximity plus hard T | Lower verification load | Inherits hard-T false exclusions; mandatory spatial test can also lose visual recovery when poses drift |
| **M-soft** | C on spatial **or** visual proposals; topology only orders attempts | Preserves inter-chain eligibility and existing owners; candidate-budget diversity possible | Ordering still changes first-success result and may starve candidates under budget; optional experiment, not guaranteed accuracy gain |
| **H** | Same region/component/community or region ranking | Could save work on large maps | No reliable room/floor/region identity exists; full graph connected after B1; communities change with loop edges; a new graph adds unjustified state |

No arbitrary numeric quality scores are assigned. G/C have constant predicate cost; T/M/H require extra graph or region work. C needs only existing root facts; full T needs adjacency and query scratch. H would require evidence/maintenance beyond this task. All policy families need deterministic tie breaking and separate reporting of retrieved, eligible, attempted and accepted counts. More eligible targets may increase BBS/GICP time, even when predicate time is negligible.

### Metrics considered

| Metric | Precise definition | Assessment |
|---|---|---|
| **d0, type-0 chain distance** | Minimum number of undirected type-0 edges; infinity across components | Recommended *temporal* measure for review. Under current linear contiguous chains, same-chain value is `q-t`. Type-1 excluded completely |
| Full-graph hops d01 | Minimum number of type-0/type-1 edges, each unit cost; add transient `(q-1,q,0)` in query scratch | Literature-supported by current MOLA submap selector. Unit cost counts relationships, not confidence/metres. Suitable diagnostic/optional soft signal; no hard threshold recommended |
| Odometry travel length | Sum of norms of translation of original type-0 relative transforms along one chain | More metric than KF count, but measures path travel, not elapsed time or independent information. Historical chain components disconnected; loops cannot supply missing odometry travel |
| Travel length with zero/low-cost type-1 | Dijkstra on travel edges plus artificial loop costs | Collapses attached regions; a false edge may erase a long path. No cost invariant or inspected source justifies these weights for Sapphire |
| High-cost type-1 | Assign a large finite penalty to cross-loop paths | Requires arbitrary units/calibration; can approximate chain separation badly. Use explicit type-0 membership instead |
| Confidence/uncertainty-weighted paths | Weight edge by covariance/information | Neither sum of covariance scalars nor inverse information is generally a posterior pose uncertainty; orientation, Jacobians and correlations matter. Do not invent weights |
| Same-chain distance / previous-chain unrestricted | d0 finite and >K, or d0 infinite | Exactly C; no second topology representation |

A full-graph distance must be evaluated on a defined **committed revision**, plus the transient odometry edge for q. Mutating the committed graph just to query it is unnecessary and unsafe. In a valid B1-attached Sapphire history the full graph is connected; a disconnected full graph indicates invalid history, not permission to silently continue. Infinity in d0 across valid chains is expected.

## 5. Sapphire ownership, storage and visual archive implications

### Deriving chain membership without a session ID

The existing `validateTopology` already derives roots: IDs are contiguous from 1; a valid type-0 predecessor carries forward the previous root; missing predecessor requires a type-1 root attachment and starts a new root. Type-1 links do **not** merge odometry domains. Link directions and original-anchor consistency remain separately validated. [Current validator](../sapphire/src/mapping/storage/map_database.cpp#L36).

Reconstruction is O(V+E), piggybacking on existing validation. A sorted root list costs O(S) for S chains and arbitrary historical membership takes O(log S); the already-derived per-node root vector permits O(1) lookups for O(V) memory. The active B2 tail needs only its accepted root r and q: `t<r` means historical other chain; `r<=t<q` means same chain. This O(1) simplification relies on **non-interleaved, contiguous, append-only type-0 chains**. If interleaving is ever introduced, the invariant and prefix formula must be redesigned, not patched by pretending IDs still encode membership.

For q101 and r100, targets99/98 are different-chain and eligible under C; target100 is same-chain and excluded. For roots rA<rB<rC, all targets before rC are historical domains regardless of whether their type-1 attachments connect A/B/C. Historical arbitrary queries use their own root interval and reject t≥q, not whichever chain happens to be active now.

### Retrieval placement

For spatial retrieval, reuse transient-query AABB and committed poses, then apply C before target payload preparation and BBS/GICP. For visual retrieval, excluded descriptors must be removed from the searchable population **before NNDR / nearest-neighbor competition**, not only filtered after top results. After retrieval, deduplicate and defensively apply the same target predicate, retain baseline ranking for C, optionally reorder for M-soft, then use unchanged BBS/GICP acceptance. Visual evidence must remain able to propose candidates outside current spatial proximity; a hard intersection of the two sources would remove its recovery role.

Topology-based region restriction *before both retrieval owners* is unnecessary for C and risks removing candidates before their appearance evidence is considered. Neighbor expansion following a successful loop can be investigated later using historical type-0 neighbors, with a bounded proposal budget and the same eligibility/geometric checks; it cannot imply loop acceptance.

### The archive can remain one monotone prefix under C

Use **zero-based backend keys** q,t and active root key r. The eligible prefix is

```
b(q) = min(q, max(r, q-K))
eligible iff 0 <= t < b(q)
```

For active continuation q≥r and K≥0 the min is redundant but makes the no-self boundary explicit. SQL q101/r100 correspond to keys100/99: `b=99`, therefore SQL1..99 are eligible and SQL100 is excluded. For Q2, the same old-chain prefix remains; when the active chain ages past K, same-chain scenes join one at a time. Prefix advancement may be **zero**, rather than always one node.

B1 currently materializes all committed scenes including Q0. A one-time handoff must canonicalize to the selected eligible prefix; for C at Q1 it excludes Q0, whereas the old global-gap baseline excludes additional old-chain targets. Keep existing ascending insertion and settled training/fingerprint rules. Simply erasing descriptors from a differently trained index does not establish equal-prefix behavior. No second Faiss archive, persistent session field, or per-query graph-dependent erase/reinsert is needed for C. General hard T produces non-prefix, revision-dependent membership, complicating NNDR and archive organization considerably. M-soft changes only post-retrieval ordering, not archive membership.

A2's **unchanged** `prepareHistory` is still global-gap based. Its default result cannot serve as an identical candidate/NNDR oracle for a proposed C prefix. Any later acceptance must explicitly align writer, reconstructed visual membership and historical query semantics to the reviewed policy. Old G fixtures remain useful regression cases, but expected cross-root results must be intentionally reviewed.

### Persistence: no new field does not mean no compatibility impact

The current `commitFinalizedSubmap` requires ordinary `id-loop.to_id>3`, and `validateTopology` imposes the same rule on ordinary type-1 edges when reopening. A C-selected `(101,99,1)` would be rejected. [Writer gate](../sapphire/src/mapping/storage/map_database.cpp#L794), [reader gate](../sapphire/src/mapping/storage/map_database.cpp#L65).

If C is approved later, both gates must use chain-aware semantics while preserving root attachment, forward type-0 direction, backward type-1 direction, at-most-one ordinary loop per query, original anchors, transaction/revision rules and failure behavior. This is an **accepted W/A2 semantic-contract amendment**, even though Node/Link schema need not change. Old readers may reject newly valid cross-chain nearby-ID edges. A format/version compatibility decision must be made before producing such maps; “no persistent session ID needed” is not permission to silently promise old-reader compatibility. No such changes were made here.

The original B2 throughput issue remains: W preflight/topology validation and full serialized-pose reconciliation can scale with map size. Constant-time C cannot establish end-to-end constant-time continuation or resolve those independent costs.

## 6. Bayesian, information and scheduling interpretation

RTAB-Map is a real example of recursive Bayesian loop hypotheses: its current `BayesFilter::computePosterior` propagates the previous posterior through a prediction operator, multiplies by likelihood and normalizes. [Implementation](https://github.com/introlab/rtabmap/blob/66c72be7db6863b65fafbc7fa134148503e0b815/corelib/src/BayesFilter.cpp#L150). The multi-session paper describes geometry after hypothesis selection and retrieving neighboring historical nodes after a loop, providing evidence for **expansion rather than permanent suppression**. [2014 multi-session paper](https://arxiv.org/html/2407.15305v1).

A generic expression is `p_t(i) ∝ likelihood(z_t|i) Σ_j P(i|j,graph) p_(t-1)(j)`, with a new-place hypothesis and nonzero recovery support. That is a formal filtering structure, but transition/observation models still require assumptions. Sapphire currently has no calibrated descriptor likelihood, transition matrix, posterior state or reset/relocalization contract. Merely multiplying a score by a hop penalty is **not** a Bayesian method.

LAMP's 2022 prioritization seeks batches that reduce pose covariance, using learned utility and observability. This supports spending verification budget where it is useful; its GNN/annealing/multi-robot machinery is not a minimal B2 dependency. [Paper](https://www-robotics.jpl.nasa.gov/media/documents/Loop_Closure_Prioritization_for_Efficient_and_Scalable_Multi-Robot_SLAM.pdf), [released stack](https://github.com/NeBula-Autonomy/LAMP).

MOLA covariance propagation is probabilistic pose modeling, while its hop cutoff is a heuristic. ORB covisibility scores/consistency checks likewise are not a calibrated topology prior. A factor graph representing conditional dependencies for optimization does not automatically make candidate gating Bayesian.

For B2, keep C as a clear structural heuristic/invariant proposal. If soft novelty is later tested, specify the rank, deterministic tie breaks, maximum graph visits, baseline candidates protected from demotion, and stop behavior. One reasonable experiment reserves some attempt budget for original retrieval order and some for topology diversity; with a budget of one that separation is impossible. Do not add a persistent pending-candidate scheduler or an unbounded queue merely to claim fairness.

### Robust accepted constraints are a separate question

This bounded inspection concerns what the methods teach about imperfect graph evidence; none is proposed for B2 adoption or as a replacement for BBS/GICP.

| Approach / original year | What it evaluates | Inspected source / primary evidence | Lesson and limit |
|---|---|---|---|
| PCM, ICRA **2018** | Pairwise compatibility of proposed inter-map relative measurements using odometry cycles; chooses a large mutually consistent subset | [Original paper](https://robots.et.byu.edu/jmangelson/pubs/2018/mangelson18icra.pdf); [Kimera-RPGO `Pcm.h`](https://github.com/MIT-SPARK/Kimera-RPGO/blob/master/include/KimeraRPGO/outlier/Pcm.h), `parseAndIncrementAdjMatrix`, `findInliers` / maximum-clique heuristic | Evaluate consistency of measured transforms, not just whether endpoints are close in a pose graph. Pairwise consistency does not prove global truth, especially for mutually consistent outliers |
| GNC / TLS, RA-L **2020**, arXiv **2019** | Robust residual loss and iteratively changing weights in optimization | [Yang et al.](https://arxiv.org/abs/1909.08605); [Kimera solver parameters](https://github.com/MIT-SPARK/Kimera-RPGO/blob/master/include/KimeraRPGO/SolverParams.h); current MOLA `lc_common::run_gnc` | An accepted constraint need not be treated as permanently equally reliable. GNC is an optimization strategy, TLS a loss; global correctness is not guaranteed by naming them |
| Switchable constraints, IROS **2012** | Jointly optimizes poses and latent switches for potential outlier factors | [Paper](https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf); [author's Vertigo implementation page](https://nikosuenderhauf.github.io/projects/switchableConstraints/) (C++ extensions to g2o/GTSAM) | Effective constraint influence may be inferred instead of treated as unquestionable connectivity; additional state/prior assumptions would exceed B2 |
| Dynamic Covariance Scaling, ICRA **2013** | Reduces factor influence as residual error grows without adding the switch variables of the original switchable formulation | [Original paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/agarwal13icra.pdf); [g2o `RobustKernelDCS::robustify`](https://github.com/RainerKuemmerle/g2o/blob/master/g2o/core/robust_kernel_impl.cpp) | Residual/information treatment is different from a pre-verification hop cutoff; tuning and initialization still matter |

PCM's “consistency graph” has **measurements as vertices** and pairwise-compatibility edges. It is neither the robot pose graph A nor a scene/descriptor/navigation graph B/C/D; explicitly adding this label avoids misclassifying it as another pose-hop policy. Kimera's inspected implementation creates adjacency from candidate factors and calls a clique heuristic; an exact maximum is not automatically guaranteed by that implementation.

Current MOLA's `run_gnc` explicitly selects **Geman–McClure (GM)** and passes known-inlier indices. Do not describe it as current MOLA GNC-TLS merely because TLS appears in the GNC literature. [Pinned implementation](https://github.com/MOLAorg/mola_sm_loop_closure/blob/1cd5fb89471a175f77c71744001bffee26a25844/module/src/common/gnc_optimizer.cpp#L22).

The common lesson is limited: estimated loop constraints can be imperfect, and geometric/consistency/residual evidence is richer than connectivity. These methods do **not** prove that preserving every candidate is optimal, that more corroborating edges always help, or that a robust solver can rescue evidence never proposed. An outlier can survive geometric verification; conversely, a correct but redundant edge may add little information. Eligibility, attempt scheduling, measurement verification and robust factor treatment must remain separate. This task changes none of Sapphire's accepted geometry, factor noise or optimizer behavior.

## 7. Executable synthetic comparison

[Probe](../../audit/2026-09-28-loop-eligibility/policy_probe.py), [complete JSON](../../audit/2026-09-28-loop-eligibility/policy_results.json), [generated table](../../audit/2026-09-28-loop-eligibility/policy_results.md).

Run `python3 /home/user/code/sapphire_git/audit/2026-09-28-loop-eligibility/policy_probe.py`. No external packages or Sapphire imports. It uses a supplied retrieved-candidate list, so it tests **policy logic**, not point-cloud overlap, descriptors, registration, SLAM accuracy or real-time latency.

Definitions: G/C use K=3. **T1** isolates current MOLA's non-GNSS unit-hop `d01>1` gate on a synthetic graph; it is not a reproduction of the entire MOLA algorithm. **T3** is a hypothetical stronger `d01>3` stress test, not a MOLA/ORB default. **M2** retains C candidates and stably moves ≤2-hop candidates after >2-hop candidates. Radius2 is an explicit experimental stress choice, not a recommended production weight/threshold or a claimed published algorithm. All candidate lists below are also the attempt order.

| Case | G | C | T1_MOLA_hop_gate | T3_stress_only | M2_soft_experimental |
|---|---|---|---|---|---|
| fresh_Q1 | [97, 96] | [99, 98, 97, 96] | [99, 98, 97, 96] | [97, 96] | [98, 97, 96, 99] |
| fresh_Q2 | [98, 97] | [99, 98, 97] | [100, 99, 98, 97] | [98, 97] | [99, 98, 97] |
| before_shortcut | [99, 98, 97, 102] | [99, 98, 97, 102] | [99, 98, 97, 102, 103, 104] | [99, 98, 97, 102] | [99, 98, 97, 102] |
| after_verified_shortcut | [99, 98, 97, 102] | [99, 98, 97, 102] | [99, 98, 97, 102, 103, 104] | [97, 102] | [98, 97, 102, 99] |
| after_wrong_shortcut_same_connectivity | [99, 98, 97, 102] | [99, 98, 97, 102] | [99, 98, 97, 102, 103, 104] | [97, 102] | [98, 97, 102, 99] |
| three_chains_Q1 | [102, 99, 98] | [104, 103, 102, 99, 98] | [104, 103, 102, 99, 98] | [102, 99, 98] | [103, 102, 99, 98, 104] |
| same_place_sparse_sampling | [] | [] | [1] | [] | [] |
| same_place_dense_sampling | [1] | [1] | [1] | [1] | [1] |
| disconnected_history_diagnostic | [] | [99, 98] | [99, 98] | [99, 98] | [99, 98] |
| A_F_continuous_query101 | [97, 96] | [97, 96] | [99, 98, 97, 96] | [97, 96] | [97, 96] |
| F_restart_query101 | [97, 96] | [99, 98, 97, 96] | [99, 98, 97, 96] | [97, 96] | [98, 97, 96, 99] |
| F_continuous_query103 | [99, 98, 97] | [99, 98, 97] | [101, 100, 99, 98, 97] | [99, 98, 97] | [99, 98, 97] |
| F_restart_query103 | [99, 98, 97] | [99, 98, 97] | [101, 100, 99, 98, 97] | [99, 98, 97] | [99, 98, 97] |
| E_query101_after_root100_H40 | [41, 40, 42, 97] | [41, 40, 42, 97] | [41, 40, 42, 97] | [42, 97] | [41, 42, 97, 40] |
| E_query102_after_corrob101_H41 | [42, 41, 43, 97] | [42, 41, 43, 97] | [42, 41, 43, 97, 100] | [43, 97] | [42, 43, 97, 41] |

Interpretation:

- Fresh Q1 graph: old1..99, attachment100→99, transient100→101. Retrieved `[100,99,98,97,96]`. G loses99/98; C restores them. **Actual MOLA-style T1 also retains them**. T3 loses them through the attachment shortcut. This prevents overclaiming against the literature-supported small adjacency gate.
- At Q2, T1 alone admits same-chain root100 at distance2. It therefore cannot substitute for the existing K=3 temporal intent. C excludes100/101 but retains99.
- Shortcut case: chain100..106 attached to97; insert type-1 `(105,99)`. The path q106→105→99 has two hops. C remains unchanged; T3 drops99/98. Labeling the old edge wrong produces the same candidate decisions: topology alone cannot recover its correctness.
- Three-chain case includes A1..99, B100..104, C105..106; C correctly permits B104/103 immediately after attachment, while G/T3 suppress them.
- Sparse/dense sampling represent the same conceptual return with different inserted-node counts. C also changes same-chain behavior with K; chain awareness does not cure density sensitivity. This is a reason to evaluate K, not proof that travel length is automatically better.
- The disconnected full-graph row is a **diagnostic only**, outside valid post-B1 Sapphire history. It illustrates infinite full-graph distance, not a proposed recovery mode.
- M2 never removes a C candidate, but after the shortcut it demotes strongest retrieved target99 behind98. A one-attempt budget would miss99. Soft ordering avoids hard-set loss, not budget loss.
- **14,553** root/query/K combinations verify the proposed contiguous-prefix identity; fifteen scenario assertions and the LIFO-vs-BFS counterexample pass. These tests provide no statistical recall or speed claim.

## 7a. Falsification, endogenous topology and restart counterfactuals

All directions, including C and soft ranking, are hypotheses. The source survey and fixtures were used to seek counterexamples, not merely demonstrations. Distinguish a paper's algorithmic claim, a source path, its configuration default, user reports and this report's inference; agreement between two of these is not evidence for the others.

| Hypothesis tested | Strongest contrary evidence in this investigation | Consequence |
|---|---|---|
| Upstream ORB3 rejects candidates below a universal graph-distance threshold | Source uses connected/covisible neighborhood intersection; the explicit 50-depth switch belongs to stella, not upstream ORB3 | Correct the premise; retain only the transferable neighborhood/feedback concern |
| Practitioners commonly disable this because it performs poorly | No supporting prevalence evidence; stella AIST example and several user configurations enable it | No population-level claim; default-off does not prove the asserted reason |
| Full graph distance must suppress the first useful post-B1 loops | Actual MOLA unit-hop >1 admits old99/98 at q101; it also admits same-chain root100 at q102 | Do not condemn all topology gates; this gate alone solves a different adjacency problem |
| Adding the actual MOLA gate to C improves B2 | A fresh ordinary q has just one incident edge, its planned type-0 predecessor; C already excludes that predecessor | Every remaining C target has d01≥2. C+T1 is redundant under this invariant; no new mechanism is warranted |
| C is restart invariant | Replacing 99→100 odometry with 100→99 attachment admits99/98 at q101, with identical physical trajectory and undirected connectivity | C deliberately distinguishes trusted temporal domains, not physical motion. The difference must be bounded and measured |
| C solves all temporal-neighborhood problems | Sparse/dense fixtures change same-chain eligibility with K; physical overlap and correlated evidence survive a restart | K remains a heuristic; C fixes the cross-chain identity error only |
| Soft topology ordering is harmless because membership stays constant | M2 demotes target99; an attempt budget of one does not examine it | Soft ranking needs budget/first-success evaluation; default recommendation keeps existing ordering |
| Modern semantic/topological papers validate the replacement | SG-SLAM retains a recent sequence window; SGLC/GAT primarily change representation/registration | Classify them as irrelevant to the specific eligibility defect, while retaining limited conceptual observations |

### Stable structure versus estimated connectivity

For fixed q, targets, type-0 edges and original anchors, inserting any type-1 edge leaves **d0 and C hard eligibility invariant**. It may change d01, optimized poses, spatial retrieval, and rankings. Thus C does **not** guarantee that a bad loop cannot change the overall proposal/acceptance pipeline; it guarantees only that a type-1 shortcut cannot change its temporal predicate. Type-0 lineage is a structurally validated fact; type-0 transforms can still contain odometry error. Small d0 represents sequence adjacency, not a proof of geometric redundancy.

For the common query106, before/after type-1 `(105,99)`, the recorded policy changes are:

| Policy | Before | After | Classification |
|---|---|---|---|
| G | `[99,98,97,102]` | Same | Invariant, but does not fix the independent chain-boundary failure |
| C | `[99,98,97,102]` | Same | Desired temporal-predicate invariance |
| T1 actual MOLA hop component | `[99,98,97,102,103,104]` | Same | No corroboration suppression here; weaker than K=3 temporal exclusion |
| T3 hypothetical stronger gate | `[99,98,97,102]` | `[97,102]` | Potential redundancy suppression, but **dangerous** when99/98 are stipulated valid corroboration; indistinguishable if the shortcut is wrong |
| M2 experimental soft order | `[99,98,97,102]` | `[98,97,102,99]` | Set-preserving; harmless only if changed attempt/stop behavior loses no useful evidence |

These classify effects, not measured accuracy. The JSON records eligibility additions/removals and order changes for each tested policy. H was rejected as a concrete proposal because no reliable region assignment was established; there is no invented H rule to test. M-hard corresponds to imposing an additional T gate and inherits its removals.

There is a more severe **persistence counterexample** to hard full-graph distance: a proposed `(106,99)` has d01=9 in the pre-insertion fixture and d01=1 after its own edge is stored. Reapplying a hard d01>3 rule to the final reconstructed graph rejects that very accepted edge. Temporarily removing each tested edge does not generally solve this: later loops may shorten alternate paths and retroactively invalidate older edges. Reconstructing the complete chronological prefix for each decision adds work and historical assumptions. C avoids this because adding type-1 edges cannot change the classification it validates. A transient ranking decision should not become a map validity invariant.

### Case F: representational session split

The continuous case has type-0 chain1..101. The split case has chain1..99, root100 attached to99, and type-0 100→101. Nodes, candidate order, assumed physical poses and undirected edge endpoints are identical; only one edge's type/direction changes.

At q101 with `[100,99,98,97,96]`, G returns `[97,96]` in both cases, T1 `[99,98,97,96]` in both, and T3 `[97,96]` in both. C changes from `[97,96]` to `[99,98,97,96]`; M2 additionally reorders the split list. By q103 C's sets coincide (`[99,98,97]` for the supplied candidates). Therefore this fixture favors G/T on strict label invariance, a genuine counterargument to C, while G still fails when the previous session is temporally unrelated.

For a split at r with unchanged IDs and otherwise equivalent inputs, C can add only old-chain targets in `[q-K,r-1]`; there are at most K such targets, and the difference disappears when q≥r+K. Unrelated older nodes outside that narrow boundary window do not disappear or appear. The fixtures explicitly check that behavior. This bound relies on contiguous IDs and a *pure* split, not a restart after an arbitrary different trajectory.

Exact physical-temporal invariance cannot be guaranteed from the permitted facts: a geometrically correct attachment does not reveal whether the gap was a millisecond restart, days of inactivity or relocation to a different path. Reimposing cross-chain temporal exclusion based only on the attachment would invent that missing information. If real data show the bounded split effect is materially harmful, the recommendation must be reconsidered; it cannot be defended by definition.

### Cross-chain locality and the attachment shortcut

For Qj after rootQ0 attached to H, the full graph has a path to H of length j+1; a historical neighbor k odometry edges from H has a path no longer than j+1+k. A hard full-graph threshold can suppress precisely the old neighborhood being revisited. That is an intentional geometric connection becoming an artificial *temporal* shortcut if used for recency. It resembles stella's loop-edge traversal feedback, but does not establish the same observed failure in upstream ORB3's covisibility graph.

| Cross-chain choice | Evaluation |
|---|---|
| No temporal exclusion merely for crossing chains | Minimal C proposal; still requires a retrieved proposal and geometric verification, not “try every historical node” |
| Spatial-only hard eligibility | Cheap but excludes appearance-based recovery when committed poses drift; conflicts with preserving both existing sources |
| Appearance-only hard eligibility | Loses geometry-only candidates and cases with absent descriptors; also unsupported |
| Spatial **or** visual proposal, optionally topology-ranked | Matches current ownership and preserves complementary evidence; ranking effects remain experimental |
| Temporary suppression around attachment H | May reduce redundant work, but no established invariant says these targets cannot corroborate/correct attachment; not recommended as a hard rule |

No special persistent attachment Link type is needed even for a future ranking experiment: an outgoing type-1 edge from a derived noninitial root with no incoming type-0 predecessor identifies the attachment role. The baseline uses no full-graph ranking; in the diagnostic d01 both attachment and ordinary type-1 edges cost1. If a future M variant down-ranks attachment neighborhoods, evaluate it separately against protecting those proposals during early continuation. No literature here justifies choosing different numeric attachment/loop weights.

### Case E: consecutive corroborating queries

With root100→H40 already accepted, q101→H41 has path length3 (`101–100–40–41`). It passes C and T1; T3 rejects it. With q101→H41 added to the common history, q102→H42 again has length3 and the same result. These common-history rows intentionally hold the graph constant across policies.

A second simulation lets each policy change its *own* subsequent graph, using one stipulated geometrically valid proposed loop per query and no multi-loop-per-query feature:

| Policy | q101→41 | q102→42 | q103→43 | q104→44 |
|---|---|---|---|---|
| G, C, T1, M2 | Allow | Allow | Allow | Allow |
| T3 | Reject | Allow | Reject | Allow |

The alternation is a counterexample to automatic corroboration, not proof that every retained constraint is useful. Nearby scans can share scene points, biases and odometry error; extra edges are not automatically statistically independent information. ORB's candidate-group/spatial confirmation uses nearby support positively; RTAB retrieves neighboring history; MOLA retains two-hop pairs. These support keeping the question open rather than assuming proximity means uselessness. LAMP's utility/budget work conversely supports avoiding unlimited redundant effort.

For the minimal proposal, **allow consecutive-query corroboration through existing retrieval and BBS/GICP**, keeping one loop per query. Do not add a special corroboration window, unconditional acceptance, or hard post-loop cooldown. Soft deprioritization or a limited protected window are real-data alternatives if repeated registration dominates the budget; they are not settled requirements.

### Strongest counterargument and falsification experiment

**Strongest counterargument to C:** it discards a useful redundancy cue at a restart. A physically continuous restart can immediately re-admit overlapping old scans, increase BBS/GICP work, and let an ambiguous near-attachment match win the first-success race. Even with unchanged geometric verification, that can reduce graph quality or crowd out a more informative proposal. Fewer hard exclusions can worsen practical performance under a fixed budget; d0 invariance alone does not establish usefulness.

Falsify C with paired replay of the **same sensor trajectory** continuously and with controlled frontend splits, plus genuine revisits after independent sessions. At identical hardware, geometry thresholds, retrieval settings and attempt/time budgets, compare G, C, C+T1 (expected logically identical), C with the M-soft experiment, and a clearly labeled stronger T diagnostic. Include weak/aliased attachment regions, changed keyframe/submap density and separately injected historical false loops. Use ground-truth overlap/transforms where available and independently inspect all newly admitted near-boundary acceptances. Count useful-loop recall, false accepted loops, attempted/rejected registrations, first-success target, constraint residual consistency, trajectory error and latency/backpressure tails.

**C is falsified as the preferred default** if its restored cross-chain proposals yield no useful-loop benefit on genuine independent sessions while reproducibly increasing false accepted loops, degrading trajectory consistency, or preventing the required continuation rate under the same budgets; also if a simpler admissible alternative preserves that recall with better accuracy/cost. Pre-register application acceptance tolerances rather than selecting a convenient threshold after seeing results. If results only falsify M-soft, retain C with baseline ranking. If they falsify C, revise the policy explicitly; do not quietly impose a new universal graph threshold. Synthetic logic cannot settle this empirical tradeoff.

## 7b. One semantic contract across retrieval, W and A2 — proposed, not finalized

The following is implementation-independent **review pseudocode** for the leading hypothesis. It is not an adopted B2 rule. Valid historical structure, payload integrity, geometric acceptance and temporal eligibility remain distinct contracts.

```
classify(query, target, type0_topology, K):
    require validated contiguous chain structure
    if target is not committed or target.id >= query.id:
        return not_historical
    qroot = root_of(query)  # planned type-0 predecessor supplies root for transient q
    troot = root_of(target)
    if qroot != troot:
        return other_chain
    if type0_edge_count(query, target) <= K:
        return same_chain_recent
    return same_chain_nonrecent

eligible(query, target, topology, K):
    return classify(...) in {other_chain, same_chain_nonrecent}

propose_and_verify(query, committed_revision):
    visual_population = committed targets satisfying eligible(...)
    # In current append-only continuation this population is one prefix.
    visual = retrieve_visual_from_eligible_population(query, visual_population)
    spatial = spatial_retrieval(query, committed_poses)
    candidates = deduplicate(visual union spatial)
    candidates = filter(eligible, candidates)
    ordered = existing_deterministic_ranking(candidates)
    # Optional M experiment only: order using revision-local d01/metric/appearance.
    # Never remove eligibility because of a rank, visits cap or failed graph search.
    for target in ordered within explicit verification budget:
        if BBS_then_GICP_accepts(query, target):
            return one_ordinary_loop(query, target)
    return no_ordinary_loop

W_optional_loop_preflight(query, target, planned_type0_base):
    require eligible(query, target, topology_with_planned_type0_base, K)
    require existing directions, uniqueness, anchors and finalized-write invariants
    # No rank/score threshold: writer validates admissibility, not winner optimality.

A2_or_W_validate_recorded_graph(graph):
    roots = derive_and_validate_type0_chains(graph)
    for each type1 edge (query, target):
        if query is a noninitial chain root:
            validate_existing_B1_root_attachment_contract(query, target)
        else:
            require eligible(query, target, roots, K)
        require existing edge direction/uniqueness/payload contracts
```

The visual retrieval function above excludes ineligible targets from the population **before** computing nearest neighbors/NNDR. It does not authorize post-NNDR filtering as a substitute. Spatial checks may apply C after the index query because discarded targets then do not compete in a descriptor ratio. Candidate budget is a scheduling outcome, not a new persistent validity rule.

| Consumer | Shared fact it must use | What it must not reinterpret |
|---|---|---|
| Spatial eligibility | `eligible(q,t)` from validated roots | No separate global-ID gate across roots |
| Visual population | Exactly the same predicate before NNDR | No old `q-3` population if selected rule is C |
| Candidate ranking | Only eligible proposals, deterministic on same inputs/revision | “Low priority” cannot mean structurally invalid |
| W ordinary-loop preflight | Same predicate with planned q predecessor/root | No unchanged `id-target>3` rejection of cross-chain pairs |
| A2/W graph validation | Same stable type-0 relation, root attachment exception | No full-graph hop test that rejects an edge because it exists |
| Independent reopen tests | Same IDs/roots/K/population and revision-aligned ranking inputs | Do not compare different Faiss populations or reconstruct historical scores from a later optimized graph |

A2 validates facts derivable from persisted history; it cannot re-run BBS/GICP or certify historical ranking from Node/Link records alone. The shared contract defines *where* ranking belongs, not a requirement for storage to rank. For deterministic live-vs-reopen ranking comparisons, use the same committed graph revision, same transient query, canonical visual membership/training mode and deterministic ties. Ranking may legitimately change after a new revision; historical edge validity must not.

K and the selected admissibility semantics must be identical for writer and reader; they cannot become independently tunable runtime thresholds that retroactively invalidate existing edges. Any later K change requires an explicit compatibility decision.

The standalone probe round-trips its graph through JSON and recomputes identical lists. This is only a semantic reconstruction check. Required later production acceptance remains a real cross-process W write → destroy → A2 reopen → continuation test, including adjacent cross-chain pairs, multiple roots, own-edge/full-graph self-invalidation, visual NNDR counterexamples and policy/version compatibility. A1/A2/W/B1 are not amended by this document.

This decomposition needs **zero new persistent fields, zero new Link types and zero session table**. It needs reviewed changes to existing predicate call sites and validation semantics if accepted. It offers no permission to create a new policy manager, graph database, robust backend or session architecture.

## 8. Code portability / applicability shortlist

| Project / component | License as inspected | Language / ROS | GTSAM / Ceres / other dependencies | Isolation / usefulness |
|---|---|---|---|---|
| S-Graphs selector/floor graph | GPLv3 root license | C++; current ROS2/rclcpp | g2o, PCL, Eigen; no GTSAM/Ceres requirement found in inspected main CMake | Selector small but floor identity depends on hierarchy; concept only |
| SG-SLAM `SemGraphMapping::SearchLoop` | MIT | C++; ROS1 Noetic / ROS2 Foxy instructions | GTSAM backend, Ceres, Sophus, Eigen, TBB, nanoflann, labeled clouds | Recent-window descriptor selector separable, but no chain-aware mechanism to import |
| SGLC pair matcher | MIT | C++17 standalone evaluators; no ROS needed there | Ceres, PCL, Eigen, Sophus, OpenCV, TBB; no GTSAM in inspected evaluator CMake | Useful future registration research, not B2 temporal eligibility |
| SemanticLoopClosure / GAT | `package.xml` says BSD; no root LICENSE found | Python/PyTorch + C++ ROS1 Noetic | GTSAM, Ceres, PCL, CUDA/model/data preprocessing | Repository-wide license terms not fully established; no code import proposed; too much machinery |
| MOLA Simplemap selector | GPLv3 | C++; standalone and ROS2 packaging | GTSAM, MRPT, mp2p_icp, MOLA factors/georeferencing | Reimplement small unit-hop diagnostic from existing adjacency; do not import the full graph owner |
| MRPT Dijkstra | BSD-style MRPT foundational library; per-file header | C++ | MRPT graph/container utilities | Unit BFS is enough for equal costs; no extra dependency needed |
| ORB2 / ORB3 | GPLv3 | C++; optional ROS1 examples | g2o, DBoW2, OpenCV, Eigen; ORB3 Sophus | Covisibility data absent in Sapphire; mechanism lessons only |
| stella_vslam | Declared BSD-2 in original/fork files | C++; standalone, separate ROS wrappers | g2o or GTSAM backend, OpenCV, BoW, Eigen | Optional graph selector easy to understand; do not copy its LIFO traversal as shortest path |
| RTAB-Map filter/neighborhoods | BSD-3-style project/source license | C++ core; ROS1/ROS2 integrations | OpenCV/PCL; optimizer backends configurable, not intrinsic to predicate | Formal posterior needs more state/calibration; neighbor expansion concept small |
| LAMP prioritization | MIT declared in repository | C++ + Python; ROS1 catkin | GTSAM, Torch/geometric packages, multi-robot stack | Budget/observability principle useful; learned prioritizer excessive for B2 |

License entries describe source declarations, not a legal compatibility opinion. Exact license/build manifests are retained in the audit samples. No external code was incorporated into Sapphire. stella's licensing has also been publicly disputed in [issue #249](https://github.com/stella-cv/stella_vslam/issues/249); this research depends on algorithm inspection, not an assumption that copying is cleared.

**Directly applicable:** derive C from existing type-0 roots; retain metric/appearance proposal plus independent geometric verification; optionally measure MOLA-style unit-hop novelty without making it eligibility; prototype bounded historical-neighbor proposal expansion following RTAB's principle. These are small algorithmic adaptations, not imports of external owners.

**Conceptually useful:** S-Graphs' reliable region distinctions, ORB's local support versus global proposal separation, RTAB's posterior and memory model, MOLA's uncertainty/coverage ranking, LAMP's information-aware verification budget. Each requires independent evidence before widening B2.

**Not useful for this specific defect:** SGLC/GAT semantic descriptors and semantic registration, SG-SLAM's semantic graph ownership, a new navigation/room/community graph, or neural dense mapping. The broadened search also found [Neural Graph Map](https://arxiv.org/abs/2405.03633); its dense-map/loop integration objective is separate, so it was not promoted into a candidate-policy implementation audit. No new representation is justified solely by the word “graph.”

## 9. Fourteen-point recommendation for review

1. **Best minimal candidate:** C on the existing spatial/visual proposal union, keeping current ranking and BBS/GICP. This fixes the demonstrated chain-domain error with minimal state. It remains a proposal.
2. **Second-best experiment:** M-soft = C plus bounded full-graph novelty/diversity ordering. Run it against unchanged retrieval order; keep it disabled in any proposed baseline until real-data benefit is demonstrated.
3. **Global gap:** retain `t<q` as historical identity order; restrict gap K to the same odometry chain. K=3 is an initial controlled baseline, not a newly justified physical constant.
4. **Hard vs soft topology:** type-0 temporal exclusion can be hard if its existing purpose is retained. General type-0/type-1 proximity should be diagnostic or soft ordering; no universal hard d01 cutoff selected.
5. **Exact distance:** hard temporal distance is d0, unit type-0 path length with infinity across chains. Optional novelty distance is unweighted undirected d01. A bounded BFS may return exact distance through radius R and “>R,” not a fabricated exact value beyond the search bound. R remains experimental.
6. **Edge participation:** d0 excludes all type-1 edges. d01 assigns both types cost1 because it counts constraints, not because their information is equal. No low/zero/high type-1 penalty chosen.
7. **Roots:** use existing derived root SQL IDs; B1 root begins a new domain even though its attachment connects the global graph. Active tail predicate uses its root; arbitrary historical queries use their own root.
8. **Placement:** spatial query → C → payload preparation; visual eligible population → NNDR/retrieval → defensive C; merge/deduplicate → optional ranking → BBS/GICP. Never let topology manufacture an accepted loop.
9. **Visual archive:** one prefix remains sufficient under current append invariants, with `b(q)=max(r,q-K)` for continuation. Update one-time B1 handoff, monotone advancement and oracle expectations. Full hard T would need more complicated membership handling.
10. **Complexity:** C O(1) per active candidate, O(V+E) root reconstruction already aligned with validation. Optional BFS O(V+E) time/O(V) scratch worst case; bounded-hop balls can still be large at high degree, so a visits cap must have an explicit neutral-ranking fallback. Candidate sort O(C log C), or O(C) stable bucket partition. Verification/retrieval and existing W costs still dominate plausible latency.
11. **Persistence:** no new schema/session IDs needed to derive C, but writer and reopen validation semantics must be amended together; decide old-reader/version compatibility explicitly. No change authorized or implemented by this report.
12. **B2 design:** leave eligibility unresolved; mark old global-gap rows provisional; add shared root predicate, transient-query prefix contract, cross-root storage-validation impact, oracle changes and compatibility decision. Preserve accepted failure/transaction/correction ownership. These caveats are now linked in B2_DESIGN; the replacement policy is not adopted.
13. **Synthetic follow-up:** retain current fifteen cases; add exhaustive legal root layouts/invalid links, two-process writer/reopen acceptance after policy approval, visual NNDR adversarial near descriptors, Flat/IVF transition under identical prefixes, and capped high-degree BFS with no eligibility loss. Test first-success and budget starvation explicitly.
14. **Real-data questions:** How often are restored99/98 candidates valid? How many repeat constraints are redundant versus corroborative? Does K=3 correspond to enough view separation across speeds/submap density? Does C increase BBS/GICP rejection cost or false positives? Does M-soft improve accepted geometry/trajectory at equal attempt/time budgets without starving attached regions? How sensitive are results to an incorrect historical type-1 edge, drifted metric poses, sensor reset, multiple roots and weak/empty appearance? Measure proposal recall, attempts, acceptance, graph consistency, drift and latency tails on identical inputs/hardware; no current result answers these empirically.

## 10. Audit and bounded-work conclusion

The source/code survey is complete for the requested families, with explicitly limited historical/field evidence. The strongest finding is **separation of odometry-domain adjacency from existing geometric connectivity**. The ORB case provides exact mechanisms and reproducible hazards, but no support for a claim of widespread disabling.

Only this research document and provisional annotations in B2_DESIGN were changed in the repository. Synthetic code/results, source samples, fetch metadata and preservation records are outside production under [the audit directory](../../audit/2026-09-28-loop-eligibility/README.md). Existing source/test hashes are verified separately. The B2 temporal rule is awaiting review; B2 implementation has not started.
