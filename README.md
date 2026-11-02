# wrpuppets-fbt-ik

Full-body IK for VR on any humanoid skeleton: a headset, two controllers (or finger-tracking gloves) and three
trackers (waist, feet) drive the whole body -- hips, legs, spine and head, arms and hands. Plain C++17 for any
engine (not an Unreal plugin, though it grew out of WrPuppets), no dependencies, a static library: four sources, five
headers; namespace `wrpuppets::fbt`.

The solvers never look at a bone's name: a skeleton is a **rig file** with **roles** (below), so any game's or
engine's humanoid works the same way. A game uses it through a small adapter of its own: read the devices, run the
calibration and the solver, write the solved rotations into its skeleton.

- **The IK stack** (`Fbt/FbtIk.hpp`), solvers in order, each on the pose the previous one produced: the hips on the
  waist tracker; the hips give way when the feet or the head are out of reach (a one-sided least squares); the legs'
  bending plane from the feet; the legs, two bones with a soft reach; the spine to the head (it curls forward when
  compressed, then distributed passes, then FABRIK lands the head exactly); the shoulders protract towards a hand
  beyond reach; the arms, two bones with a soft reach, the elbow from the hand's position; the hand's rotation onto
  the wrist; a wrist limit.
- **The calibration** (`Fbt/FbtCalibration.hpp`): one capture standing straight maps the devices onto the skeleton
  and scales it to the player's eye height; the arms are measured to the player's (the skeleton's arms are scaled to
  them); the controllers take a fixed mount (OpenXR's grip pose), the hand-tracking wrist joint is used as it comes.
- **The filter** (`Fbt/FbtFilter.hpp`): a 1-euro filter on position and rotation per device; an impossible one-frame
  step is held for a few frames; a lost device holds its last pose.

## Build

```
add_subdirectory(wrpuppets-fbt-ik)
target_link_libraries(your_target PRIVATE wrpuppets_fbt_ik)
```

or on its own: `cmake -S . -B build && cmake --build build --config Release`.

## Use

```cpp
#include "Fbt/FbtSkeleton.hpp"
#include "Fbt/FbtCalibration.hpp"
#include "Fbt/FbtFilter.hpp"
#include "Fbt/FbtIk.hpp"
using namespace wrpuppets::fbt;

Skeleton skeleton;
skeleton.Load("humanoid_rig.txt");          // your skeleton's rig file (below)
Calibrator calibrator;
calibrator.Init(skeleton);
IkStack ik;
ik.Init(skeleton);

// Once per session, the player standing straight, arms down: the headset and the devices in room space
// (RoomPose: x right, y forward, z up, metres; devices in CalDevice order).
Calibration calibration;
calibrator.Capture(head, devices, &calibration);

// Every frame (filter the devices first with a PoseFilter each):
Goal goals[kGoalCount];
calibrator.Goals(calibration, head, devices, goals);
StackOptions options;
options.armScale[0] = calibrator.ArmScale(calibration, 0);
options.armScale[1] = calibrator.ArmScale(calibration, 1);
Pose pose = skeleton.ReferencePose();
ik.Solve(pose, goals, options);
// pose.localRot / localPos (and modelRot / modelPos) -> your skeleton
```

`StackOptions` switches each solver on and off and holds its tuning; `StackDebug` reports what each one decided.

## The rig file

Plain text, `#` starts a comment:

```
role <role> <bone index>
bone <index> <parent index, -1 for the root> <name>  <local pos xyz>  <local rot xyzw>  <model pos xyz>  <model rot xyzw>
```

- Bones in parent order (a parent before its children). Local = relative to the parent; model = the whole skeleton's
  space. Both are the **reference pose**.
- **Model space:** x right, y forward, z up, metres. Convert from your engine's axes when you write the file.
- **Reference pose:** a T-pose -- standing straight, arms straight out to the sides, palms down, thumbs forward. The
  elbows a hair back and the knees a little forward of straight (the way they bend), the spine with its natural
  S-curve: a perfectly straight leg or spine leaves the direction it bends in undefined. Bone frames can be anything:
  every axis the solvers need comes from this pose.
- **Roles:**

| Roles | |
|---|---|
| `hips`, `head` | required |
| `spine0` .. `spine7`, `neck`, `neck1` | the chain from the hips to the head, as many as the skeleton has (each the parent of the next) |
| `left_upleg`, `left_leg`, `left_foot` (and `right_`) | required: hip, knee, ankle |
| `left_upperarm`, `left_forearm`, `left_hand` (and `right_`) | required: shoulder, elbow, wrist; the upper arm's parent is taken as the clavicle |
| `left_thumb1`, `left_hand_end` (and `right_`) | optional: the thumb's first joint and the hand's end, for the hand's axes (else straight out along x, thumb forward) |
| `left_eye`, `right_eye` | optional: else the eyes are taken 8.5 cm forward and 5.5 cm up of the head joint |

## The lab

The FBT IK lab runs this library outside any game: a desktop and VR tool with a mirror, a flight recorder, a replay
that classes every bad frame (a device jumped, a limb at the end of its reach, or the solver), a stress test, and a
generic humanoid rig to start from.

## References

The published algorithms it uses, and where the approach comes from. Everything here is implemented from scratch.

- **FABRIK** -- A. Aristidou and J. Lasenby, "FABRIK: A fast, iterative solver for the Inverse Kinematics problem",
  *Graphical Models* 73(5), 2011. The spine to the head.
- **The elbow from the hand's position** -- D. Parger, J. H. Mueller, M. Steinberger and D. Schmalstieg, "Human
  Upper-Body Inverse Kinematics for Increased Embodiment in Consumer-Grade Virtual Reality", VRST 2018 (VRArmIK).
- **The 1-euro filter** -- G. Casiez, N. Roussel and D. Vogel, "1€ Filter: A Simple Speed-based Low-pass Filter for
  Noisy Input in Interactive Systems", CHI 2012. The devices.
- **WrPuppets** -- the author's earlier full-body tracking project, where this approach was first developed: how the
  solvers are stacked and in which order, the calibration, the hips giving way to feet and head out of reach, the
  legs' bending plane from the feet, the twist solver. WrPuppets ran on Unreal Engine, which provided the framework
  and some of the IK nodes it used; this library implements the whole approach from scratch, with no engine.

## Licence

MIT (see LICENSE).
