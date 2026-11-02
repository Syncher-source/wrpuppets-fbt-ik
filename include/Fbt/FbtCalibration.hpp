#pragma once

// FULL-BODY TRACKING CALIBRATION, engine-independent: a game's adapter and offline tools run the same code. Inputs are
// the devices in "room" space: OpenXR's STAGE space mapped to the avatar's model axes (x right, y forward, z up,
// metres; STAGE (x, y, z) -> (x, -z, y)).
//
// Capture, with the player standing straight (feet as the avatar's, arms hanging relaxed along the body -- only the hips, feet
// and head use the avatar's reference pose; the hands need no capture, the arms give their length):
//   - scale = the avatar's eye height / the player's (the headset's height above the floor);
//   - body frame: heading from the headset; origin on the floor where the headset maps exactly onto the avatar's eyes (the head
//     goal has no offset, so any other origin would leave the avatar's head off its reference at capture), or -- headAnchored =
//     false -- midway between the foot trackers (more repeatable offsets, but the trackers' mounts shift it and the head
//     goal with it);
//   - per device (waist, feet, hands): its pose in the body frame, scaled; the rigid offset to its bone in the avatar's reference
//     pose (model space) and the inverse of its rotation.
// Then, every frame: delta = deviceRot * inverse(capturedRot); boneRot = delta * boneRefRot;
// bonePos = scaledDevicePos + delta * offset. The head: its rotation is the headset's delta on the avatar's reference head, its
// position puts the avatar's eyes (the eye-joint midpoint) exactly on the headset.
#include "Fbt/FbtIk.hpp"

namespace wrpuppets::fbt {

enum CalDevice { kCalWaist, kCalLeftFoot, kCalRightFoot, kCalLeftHand, kCalRightHand, kCalCount };

struct RoomPose {
    bool valid = false;
    Vec3 pos;
    Quat rot;
    bool wrist = false;   // a hand: the hand-tracking skeleton's wrist joint (XR_EXT_hand_tracking), not a grip pose
};

struct Calibration {
    bool done = false;
    float scale = 1.0f, playerEyeHeight = 0.0f, avatarEyeHeight = 0.0f;
    Vec3 origin;          // the body frame's origin in room space (on the floor)
    Quat yawInverse;      // room -> body frame rotation (yaw only)
    Quat headRotInverse;  // inverse of the headset's body-frame rotation at capture
    bool valid[kCalCount]{};
    Vec3 offset[kCalCount];       // device -> bone, model space
    Quat rotInverse[kCalCount];   // inverse of the device's body-frame rotation at capture
    float armLength[2]{};         // the player's arms (left, right), shoulder -> wrist in the avatar's scale; 0 = not measured
};

class Calibrator {
public:
    bool headAnchored = true;
    // The controllers sit in the hand in a known way (OpenXR's grip pose: origin in the fist, +X across the palm, -Z
    // through the fist towards thumb and index, +Y back towards the wrist), so -- as in WrPuppets -- the hands take a
    // fixed mount instead of what the T-pose captured (which bakes the player's T-pose slop into every hand pose: up to
    // 11 cm and 20 deg on the lab recordings). false: the captured rigid offsets, as for the trackers.
    // A hand given as the hand-tracking wrist joint (RoomPose::wrist) needs no mount at all: the joint is anatomical and
    // sits on the wrist -- what VRChat uses, so controllers and gloves place the hand the same way. Its axes are
    // SteamVR's skeleton convention (see IdealWrist), not the OpenXR spec's. The grip mount below is the fallback.
    bool handsFixedMount = true;
    float handWristBack = 0.08f;   // the wrist behind the grip origin along +Y, metres (the player's size; scaled)
    // Reads the avatar's reference pose from the skeleton (eyes, head, hips, feet, hands); false when a bone is missing.
    bool Init(const Skeleton& skeleton);
    // head: the headset; devices in CalDevice order. Needs the headset and both feet; *why when it fails.
    bool Capture(const RoomPose& head, const RoomPose devices[kCalCount], Calibration* out, const char** why = nullptr) const;
    // The player's arm lengths: with the arms hanging relaxed along the body (the calibration pose; a T-pose works too),
    // the avatar's shoulder joint (reference pose) -> the hand goal, over relaxedReach (a relaxed arm is not quite straight).
    void MeasureArms(Calibration& c, const RoomPose& head, const RoomPose devices[kCalCount]) const;
    float relaxedReach = 0.98f;
    // The second instant: the arms straight out to the sides. The length then holds the shoulder-width difference
    // too, which no clavicle swing can absorb: the avatar's arms scaled to it are straight exactly there (ArmScale), the
    // clavicle covers forward and up. Measured from the span between the hands (minus the avatar's shoulder width, halved:
    // both arms alike), so it does not depend on where the body stands; the hands must be side by side near
    // shoulder height; false (and nothing changed) otherwise.
    bool MeasureArmsOut(Calibration& c, const RoomPose& head, const RoomPose devices[kCalCount]) const;
    // One arm (0 left, 1 right) straight out to its side: its length from the avatar's shoulder as the NECK places it. `head` is
    // the headset at the capture (not an earlier one): its goal is the avatar's head joint, which stays over the shoulders when the head
    // turns to look at the arm (no mirror in VR). The shoulder is the avatar's reference offset from the head joint -- the drop
    // and the half shoulder width -- laid towards the hand (horizontally), so neither the body's turn nor the waist
    // tracker enters the length. (From the waist tracker it was off by 25 cm sideways in use: the belt tilts with the
    // pelvis, and 0.5 m below the shoulders 30 deg of tilt is that much; left 1.40 x the avatar's arm, right 0.47 x.)
    // The hand must be out to the side: within 32 deg of the hips' side axis (the waist tracker's yaw, which a tilt does
    // not change; the step 2 frame without it); false (nothing changed) otherwise.
    // *why (optional) says what refused it: "lost" (head or hand), "dir" (*cosOut = cosine to straight out, needs
    // 0.85 = 32 deg), "length" (*ratioOut = shoulder-to-hand over the avatar's arm, needs 0.6..1.5).
    bool MeasureArmOut(Calibration& c, const RoomPose& head, const RoomPose devices[kCalCount], int side,
                       const char** why = nullptr, float* cosOut = nullptr, float* ratioOut = nullptr) const;
    float AvatarArmLength(int side) const { return m_armRef[side]; }
    // The scale for the avatar's arm bones from the measured length (1 where unmeasured).
    float ArmScale(const Calibration& c, int side) const { return c.armLength[side] > 0 && m_armRef[side] > 0 ? c.armLength[side] / m_armRef[side] : 1.0f; }
    // The IK goals for this frame from the devices (the hips, feet, hands and head; inactive where a device is lost).
    void Goals(const Calibration& c, const RoomPose& head, const RoomPose devices[kCalCount], Goal goals[kGoalCount]) const;
    // A room-space point in the avatar's model space through the calibration (for drawing devices next to the avatar).
    Vec3 ToModel(const Calibration& c, Vec3 roomPos) const;

private:
    int m_bone[kCalCount]{-1, -1, -1, -1, -1};
    int m_head = -1;
    Vec3 m_eye;          // the avatar's eye-joint midpoint, model space (reference pose)
    Vec3 m_eyeInHead;    // the same in the head bone's frame
    Vec3 m_headRefPos;   // the avatar's head joint, reference pose
    Quat m_headRef;
    Vec3 m_shoulderRef[2];   // the avatar's shoulder joints (upper arms), reference pose
    float m_armRef[2]{};     // the avatar's arm lengths, shoulder -> wrist
    Vec3 m_boneRefPos[kCalCount];
    Quat m_boneRefRot[kCalCount];
};

}  // namespace wrpuppets::fbt
