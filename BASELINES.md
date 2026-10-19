# Baselines

The nine recovery methods that MIRe is compared with ([README](README.md#2-method) describes MIRe). Run any of them with `--recovery <name>` ([README](README.md#4-evaluation)).

| Method | Source | What it does (all but `core_detect` use the oracle trigger: end of the perturbation + 8 steps) |
|---|---|---|
| `vanilla` | – | No recovery (only the trigger step is recorded) |
| `replan` | – | Discards the remaining action chunk and queries again with the same instruction |
| `rehome` | – | Lifts the arm, interpolates the joints back to the initial pose, and queries again |
| `retreat` | [Simple Retreat](https://arxiv.org/abs/2610.06921) (arXiv, uploaded 2026-10-02) | Retreats to the joint pose of 120 steps earlier and hands back after 40 steps |
| `safeloop` | [SafeLoop](https://arxiv.org/abs/2609.26313) (IROS 2026) | Rolls the joints back to a safe waypoint saved every 20 steps |
| `rewind_il` | [Rewind-IL](https://arxiv.org/abs/2604.16683) (RA-L 2026, presented at ICRA 2027) | Returns to the joint pose of the latest subgoal checkpoint |
| `volo` | [VoLo](https://arxiv.org/abs/2606.07723) (CoRL 2026) | Re-grasps the perturbed target with a GraspGen grasp and hands back right away |
| `core` | [CoRe](https://arxiv.org/abs/2608.14822) (arXiv, uploaded 2026-08-14) | Picks a rejoin point with an imagined rollout, physically puts the target back, and hands back. Detection is replaced by an oracle (100% detection) |
| `core_detect` | [CoRe](https://arxiv.org/abs/2608.14822) (arXiv, uploaded 2026-08-14) | CoRe closer to the paper, triggered by its latent Mahalanobis detector. Recovery after the trigger is the same as `core` |
| `mire` | ours | Does not rewind the world; re-grasps with the original grasp, carries with a DMP, and hands back early once the shadow VLA agrees |

Since this repository compares recovery, all baselines use the same oracle trigger instead of their own detectors or VLMs (only `core_detect` uses the original detector). Differences from the original papers are listed per method below.

### rehome
When a failure occurs, the gripper returns to its initial position, the action chunk is cleared, and the policy starts again.
- The arm is first lifted and then interpolated to the initial joint pose, so that it does not hit objects on the way back.
- A held object stays in the gripper on the way back.

### retreat
**Source**: Sheng et al., *Does a Learned Corrector Beat a Simple Retreat? Evidence from a Frozen VLA*, arXiv, uploaded 2026-10-02 · [arXiv:2610.06921](https://arxiv.org/abs/2610.06921) (code to be released)

When a perturbation occurs, the arm retreats to the joint pose of 120 steps earlier and hands back to the VLA after 40 steps.
- A simple recovery that only retreats, with no planner or learning. The paper finds no difference between a learned corrector and this retreat.
- Joint interpolation only, without path planning or lifting.

### safeloop
**Source**: Lou et al., *SafeLoop: Risk-Aware Rollback for Vision-Language-Action Manipulation*, IROS 2026 · [arXiv:2609.26313](https://arxiv.org/abs/2609.26313) · [GitHub](https://github.com/Loule0-0/SafeLoop/tree/release/safeloop)

Rewinds to a safe joint pose saved every 20 steps before the perturbation, then hands back to the VLA.
- The learned risk predictor and decision module of the paper are replaced by an oracle.
  - The state before an injected perturbation is always safe, so the joint pose is saved every 20 steps.
  - The perturbation time is known, so detection is set to 8 steps after the perturbation ends.
- As in the paper, no rollback is done if the arm touches something on the rewind path.

### rewind_il
**Source**: Zheng et al., *Rewind-IL: Online Failure Detection and State Respawning for Imitation Learning*, IEEE RA-L 2026 (presented at ICRA 2027) · [arXiv:2604.16683](https://arxiv.org/abs/2604.16683) · [Project](https://sjay05.github.io/rewind-il)

Returns to the joint pose of the latest subgoal checkpoint (when the target was grasped or a goal condition was met), then hands back to the VLA.
- The VLM-based checkpoint selection and the policy-feature-based recognition of the paper are replaced by BDDL milestones tracked in the simulator.
- If there is no checkpoint to return to, it does nothing.

### volo
**Source**: Chen et al., *VoLo: A Physical Orchestrator for Open-Vocabulary Long-Horizon Manipulation*, CoRL 2026 · [arXiv:2606.07723](https://arxiv.org/abs/2606.07723) · [Project](https://chicychen.github.io/VoLo/) · [GitHub](https://github.com/NVlabs/VoLoAgent)

Re-grasps the perturbed target with a GraspGen grasp and hands back to the VLA right after lifting it.
- Of the paper's tools, only the grasp tool is reproduced. VLM monitoring and tool selection are replaced by the oracle trigger and state rules, and object perception by simulator poses.
- The grasp is the first candidate, in score order, that passes IK and collision checks. How the VLA originally grasped the object is not considered.

### core
**Source**: Zhang et al., *Imagining Recovery: Inference-Time Counterfactual Realignment for Vision-Language-Action Models*, arXiv, uploaded 2026-08-14 · [arXiv:2608.14822](https://arxiv.org/abs/2608.14822) (code not released)

Rolls out the policy in imagination from a reference point before the perturbation to pick a rejoin point, puts the moved target back in place, moves the arm to the rejoin point, and hands back to the policy.
- Detection is the same oracle as for the other baselines (100% detection). The version with the original detector is `core_detect`.
- In LIBERO, the paper restores the world by writing the simulator state directly; we restore it by physical pick-and-place. Restoration can therefore fail. In that case, instead of the paper's request for human help, only the arm is moved to the rejoin point before handing back.
- The imagined rollout uses observations rendered by moving the arm in a separate copy of the simulator, instead of the paper's RGB-D voxel synthesis.

### core_detect
**Source**: the same paper as `core` (CoRe)

CoRe closer to the paper, triggered by its online OOD detector (Mahalanobis distance of the action-head latents). Recovery after the trigger is the same as `core`.
- The detector follows the paper, but uses one query instead of eight because the OFT action head is deterministic.
- There are no false triggers before the perturbation (0/1200), but triggering is late: the median delay from the perturbation to the first trigger is 60–170 steps (9–18 for the oracle). The quarter of episodes with the latest triggers has an 8% success rate, far below `core` on the same episodes (55%).
