#pragma once

// THE FULL-BODY IK STACK, engine-independent: a game's adapter and offline tools run the same code. Solvers run in
// WrPuppets' order, each on the pose the previous one produced:
//   1   The hips on the waist goal (position + rotation)
//   4f  The hips give way: feet and head goals out of reach pull the hips off the tracker (one-sided least squares)
//   2-3 the legs' bending plane from the feet (WrPuppets' knee-to-foot trick, as the foot's lateral axis)
//   4   The legs: two bones from the hips, soft reach
//   5   The spine to the head: compressed, the spine first curls forward evenly to the head goal's distance (a rounding
//       back, turned back onto its own line); then a few distributed passes (each joint, root first, takes its share of the swing that brings
//       the head to its goal, so the bend spreads along the spine), then FABRIK (Aristidou & Lasenby) lands the head exactly; the
//       head on the goal's rotation
//   5b  The shoulders protract: a clavicle swings its shoulder (at most 20 deg) towards a hand beyond the arm's reach
//       (the arms are scaled to the player's beforehand, see StackOptions::armScale)
//   6-7 The arms: two bones, soft reach; the elbow points down (and a little back), or -- with elbowFromThumb --
//       away from the hand's thumb with a downward bias (a second hand-driven elbow on top of the twist solver's roll)
//   8-9 Twist (WrPuppets' twist solver): the wrist's twist relative to the forearm, half of it rolled into the
//       forearm, the elbow rolled about the shoulder -> wrist line by -twist/2 (at most 45 deg); the hand keeps its pose
//   10  Wrist limit: the hand's bend against the forearm (the swing part, beyond the avatar's reference) capped
// Everything is in the avatar's model space (x right, y forward, z up, metres).
#include "Fbt/FbtSkeleton.hpp"

namespace wrpuppets::fbt {

enum GoalId { kGoalHead, kGoalLeftHand, kGoalRightHand, kGoalHips, kGoalLeftFoot, kGoalRightFoot, kGoalCount };

struct Goal {
    bool active = false;
    Vec3 pos;
    Quat rot;
};

struct StackOptions {
    bool setHips = true;
    bool hipsGiveWay = true;        // 4f: feet and head out of reach pull the hips off the waist tracker (least squares)
    float hipsStiffness = 0.05f;    // the tracker's hold on the hips, against a pull weight of 1
    float footPull = 1.0f, headPull = 1.0f;
    float legReachShare = 0.98f;    // a leg pulls past this share of its length (the soft reach bends away from 0.97)
    float spineReachShare = 0.98f;  // the spine pulls past this share of its length
    bool legs = true;
    float legSoftness = 0.03f;   // soft reach: from (1 - s) of the leg's length it eases to full extension, reached at the length
    bool spine = true;
    bool spineDistributed = true;
    bool spineCurl = true;          // compressed (the head goal nearer than the spine reaches): the spine first curls
                                    // forward, evenly, to the goal's distance (a rounding back) instead of FABRIK's fold   // false: FABRIK only, which puts the motion in the short neck bones
    bool arms = true;
    float armSoftness = 0.05f;   // soft reach for the arms (wider than the legs: the hands pin at full reach while the spine leans)
    bool elbowFromThumb = true;  // false: WrPuppets' arrangement (the elbow follows the hand only through the twist solver),
                                 // which flips when the wrist's twist relative to the forearm passes 180 deg (stress test)
    bool elbowFromRadial = true;    // with elbowFromThumb: away from the hand's radial axis (the thumb without its ~34 deg
                                    // tilt towards the palm, which lifts the elbow when the palms face down)
    float elbowDownPalmUp = 1.0f;   // with elbowFromThumb: more downward pull as the palm turns up (x the palm normal's
                                    // upward part): palms up, the elbows tuck down to the ribs instead of pointing inwards
    float elbowFadeStart = 0.35f;   // "away from the thumb" at full weight from this length across the line (sin of
    float elbowFadeEnd = 0.10f;     // the thumb's angle to it: 20 deg), none below this (6 deg); between, the rest direction.
                                    // Wider bands and a rest direction with a backward or outward part were tried on 13
                                    // lab recordings: more flips every time (the rule's far side opposes the rest)
    float elbowRestBack = 0.0f, elbowRestOut = 0.0f;   // the rest direction: down, this much back, this much outward
    // ARMS v2 (WrPuppets' arrangement, 2026-11-01): the elbow from the hand's POSITION only -- the avatar's rest bend swung onto
    // the arm, plus position zones (VRArmIK's) -- the hand's rotation straight onto the wrist bone, no twist solver, no
    // wrist limit. 0 flips on 13 lab recordings. v1 (the thumb rule + the twist solver) flipped
    // with the hand folded back on the chest; its parts stay as options.
    bool elbowFromPosition = true;  // the elbow a function of the hand's position only (see FbtIk.cpp); false: the thumb rule
    bool elbowZones = true;         // with elbowFromPosition: the zone offsets (hands high, low, across); false: the swung rest only
    bool twistRollPeriodic = false; // the twist solver's elbow roll -45 deg sin(twist) instead of -twist/2 clamped
    float elbowDown = 0.0f;      // with elbowFromThumb: a downward bias on "away from the thumb". 0 is best on both VR
                                 // recordings: any bias pulls the elbow off the hand, the wrist twist grows and wraps (flips)
    // The arms scaled to the player's (upper arm and forearm alike, the avatar's proportion kept):
    // the arm length measured with the arms out to the sides, over the avatar's. Sideways the shoulder cannot give way (the
    // clavicle swings, it does not shorten), so that direction is made exact by the length; forward and up the clavicle
    // covers what is left (5b). armLengthTune: a hand setting on top (VRChat's arm length).
    float armScale[2] = {1.0f, 1.0f};
    float armLengthTune = 1.0f;
    // 5b: the shoulder protracts: the clavicle swings the shoulder towards a hand beyond
    // the arm's reach, by the excess, at most clavicleLimitDeg.
    bool clavicle = true;
    float clavicleLimitDeg = 20.0f;   // a real forward reach protracts the shoulder 5-6 cm (lab wrist joints: 4-7 cm beyond the sideways arm)
    bool twist = false;             // WrPuppets' twist solver (8-9): v1; off in v2 (the hand bone takes the goal rotation whole)
    float wristLimitDeg = 0.0f;    // the hand's bend against the forearm, at most; 0 = no limit (v2: none, as WrPuppets)
    int limbIterations = 40;     // FABRIK passes at most (it converges slowly near full stretch; 7 joints are cheap)
    float limbPrecision = 1e-4f; // FABRIK stops when the end is this close to the goal, metres
};

// What the solvers decided, for drawing and checking (the lab).
struct StackDebug {
    Vec3 hipsShift;            // 4f: how far the hips gave way from the waist tracker
    bool legPlaneValid[2]{};
    Vec3 legPlaneOrigin[2];    // mid hip -> ankle goal
    Vec3 legKneeDir[2];        // unit: where the knee will point, in the bending plane, perpendicular to the leg line
    float legReach[2]{};       // hip -> ankle goal distance over the leg's length (> 1: out of reach)
    int spineIterations = 0;   // FABRIK passes used
    float spineError = 0;      // the head's distance from its goal after the solve, metres
    float spineReach = 0;      // spine root -> head goal over the chain's length (> 1: out of reach)
    float armExtension[2]{};   // 5b: the hand's distance from the shoulder over the (scaled) arm's length, before the clavicle
    float clavicleDeg[2]{};    // 5b: the clavicle's swing
    bool elbowValid[2]{};
    Vec3 elbowOrigin[2], elbowDir[2];   // mid shoulder -> wrist goal; unit, where the elbow points
    float armReach[2]{};
    float twistDeg[2]{}, elbowRollDeg[2]{};   // the wrist's measured twist, the elbow roll applied
    float wristBendDeg[2]{};                  // the hand's bend against the forearm before the limit
    bool wristClamped[2]{};
};

// Two bones (upper, lower, end) solved analytically: the end on a target, the middle joint bending towards a given
// direction, each bone aimed with its hinge axis on the bending plane's normal, soft reach near full extension.
struct TwoBone {
    int upper = -1, lower = -1, end = -1;
    Vec3 upperAlong, upperHinge, lowerAlong, lowerHinge;   // in the bones' own frames
    float a = 0, b = 0;                                    // upper and lower lengths
};
// refBend: the direction the middle joint bends to in the reference pose (legs: forward; arms in the T-pose: back).
bool MakeTwoBone(const Skeleton& skeleton, int upper, int lower, int end, Vec3 refBend, TwoBone* out);
// Returns the root -> target distance over the chain's length (> 1: out of reach).
float SolveTwoBone(const Skeleton& skeleton, Pose& pose, const TwoBone& chain, Vec3 target, Vec3 bendDir, Quat endRot,
                   float softness);

// A chain of joints from a root bone down to an end bone (each the parent of the next).
struct Chain {
    int bones[16]{};
    int count = 0;
    float length[16]{};   // bone i -> bone i+1, from the reference pose
    float total = 0;
};

class IkStack {
public:
    // Resolves the bones the stack drives; false when the skeleton lacks one of them.
    bool Init(const Skeleton& skeleton);
    // pose: the input pose (local transforms); solved in place, model transforms refreshed.
    void Solve(Pose& pose, const Goal goals[kGoalCount], const StackOptions& options, StackDebug* debug = nullptr) const;

private:
    const Skeleton* m_skeleton = nullptr;
    int m_hips = -1, m_upLeg[2]{-1, -1}, m_leg[2]{-1, -1}, m_foot[2]{-1, -1};
    Vec3 m_footSideLocal[2];   // the foot's lateral axis (model x in the reference pose) in the foot bone's frame
    Vec3 m_footForwardLocal[2];   // the foot's forward (model y in the reference pose) in its frame
    Vec3 m_hipsForwardLocal;      // the hips' forward (model y in the reference pose) in their frame
    Vec3 m_hipsSideLocal;         // the hips' lateral axis (model x in the reference pose) in their frame
    // Thigh and shin in their own frames: the long axis (towards the child joint) and the lateral axis (model x in the
    // reference pose), which the solve sends to the leg's line and its knee hinge.
    TwoBone m_legChain[2];
    TwoBone m_armChain[2];
    int m_clavicle[2]{-1, -1};   // the upper arms' parents (LeftShoulder, RightShoulder)
    Vec3 m_handThumbLocal[2];   // from the wrist towards the thumb, across the hand, in the hand's frame
    Vec3 m_handRadialLocal[2];  // the same without the thumb's tilt towards the palm
    Vec3 m_handPalmLocal[2];    // out of the palm (the avatar's T-pose palm faces down), in the hand's frame
    Vec3 m_handAlongLocal[2];   // the hand's long axis (wrist -> hand end) in its frame
    Quat m_handRefLocal[2];     // the hand's reference rotation relative to the forearm
    float m_handRefBend[2]{};   // the angle between the hand's long axis and the forearm's in the reference pose
    Chain m_spine;   // Spine .. Head
    float m_spineWeight[16]{};
    int m_head = -1;
};

// A chain (limb) solved with FABRIK (Aristidou & Lasenby, 2011): the chain's joints reach `target` (positions, lengths kept), then each bone, from the root,
// swings by the shortest arc onto its new direction (its twist kept); the end bone takes `endRot`. The root joint
// stays. initial: the joints to start the passes from (the root excepted), else the pose's. Returns the passes used;
// *error = the end's distance from the target.
int SolveLimbFabrik(const Skeleton& skeleton, Pose& pose, const Chain& chain, Vec3 target, Quat endRot, int iterations,
                    float precision, float* error, const Vec3* initial = nullptr);

// The distributed spine: passes from the root; at each joint (but the end) the swing that brings the end towards the
// target as seen from that joint, of which the joint applies weight_i / (sum of the weights from it to the last), at
// most 20 deg a pass, none when the two directions are nearly opposite (a flip), all scaled by strength (0..1); the
// end bone takes endRot. Swings cannot shorten a straight chain along its own line: the stack follows with FABRIK.
// Returns the passes used; *error = the end's distance from the target.
int SolveLimbDistributed(const Skeleton& skeleton, Pose& pose, const Chain& chain, const float* weights, Vec3 target, Quat endRot,
                         float strength, int iterations, float precision, float* error);

// Writes a model-space transform into a bone as its local transform (its parent's model transform is current).
void SetModel(const Skeleton& skeleton, Pose& pose, int bone, Vec3 modelPos, Quat modelRot);

}  // namespace wrpuppets::fbt
