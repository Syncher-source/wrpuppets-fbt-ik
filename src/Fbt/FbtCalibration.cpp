#include "Fbt/FbtCalibration.hpp"
#include <cmath>

namespace wrpuppets::fbt {
namespace {
// The yaw (about z) of a rotation's forward (+y), as a rotation.
Quat YawOf(Quat q) {
    const Vec3 f = Rotate(q, {0, 1, 0});
    const float yaw = std::atan2(-f.x, f.y);
    return AxisAngle({0, 0, 1}, yaw);
}
const GoalId kGoalOf[kCalCount] = {kGoalHips, kGoalLeftFoot, kGoalRightFoot, kGoalLeftHand, kGoalRightHand};
// The grip pose in the body frame of an ideal T-pose (palm down, fist axis forward), in model axes: the grip's +X, its
// -Z, its +Y. Matches the three lab calibrations within 10-20 deg.
// The hand-tracking wrist joint in the same T-pose. Not the OpenXR spec's frame: SteamVR passes its own skeleton's
// through (lab recording, Index controllers, arms hanging): the right hand's +X runs down the hand towards the
// fingers, the left's up the forearm (the left is the mirror), +Y towards the thumb's side, +Z out of the back of the
// hand -- which in the T-pose (palm down, fingers out along the arm, thumb forward) is the identity for both hands.
Quat IdealWrist(bool) { return Quat{}; }
Quat IdealGrip(bool left) {
    return left ? FromBasis({0, 0, -1}, {0, 1, 0}, {1, 0, 0}) : FromBasis({0, 0, 1}, {0, 1, 0}, {-1, 0, 0});
}
}  // namespace

bool Calibrator::Init(const Skeleton& sk) {
    const char* roles[kCalCount] = {"hips", "left_foot", "right_foot", "left_hand", "right_hand"};
    for (int i = 0; i < kCalCount; ++i) {
        m_bone[i] = sk.Role(roles[i]);
        if (m_bone[i] < 0) return false;
        m_boneRefPos[i] = sk.BoneAt(m_bone[i]).refModelPos;
        m_boneRefRot[i] = sk.BoneAt(m_bone[i]).refModelRot;
    }
    for (int side = 0; side < 2; ++side) {
        const int upper = sk.Role(side == 0 ? "left_upperarm" : "right_upperarm"), hand = sk.Role(side == 0 ? "left_hand" : "right_hand");
        const int fore = sk.Role(side == 0 ? "left_forearm" : "right_forearm");
        if (upper < 0 || hand < 0 || fore < 0) return false;
        m_shoulderRef[side] = sk.BoneAt(upper).refModelPos;
        m_armRef[side] = Length(sk.BoneAt(fore).refModelPos - sk.BoneAt(upper).refModelPos) + Length(sk.BoneAt(hand).refModelPos - sk.BoneAt(fore).refModelPos);
    }
    m_head = sk.Role("head");
    if (m_head < 0) return false;
    m_eye = sk.EyePoint();   // the eye bones' midpoint, or a point off the head joint for a rig without eyes
    m_headRef = sk.BoneAt(m_head).refModelRot;
    m_headRefPos = sk.BoneAt(m_head).refModelPos;
    m_eyeInHead = Rotate(Conjugate(m_headRef), m_eye - sk.BoneAt(m_head).refModelPos);
    return true;
}

bool Calibrator::Capture(const RoomPose& head, const RoomPose devices[kCalCount], Calibration* out, const char** why) const {
    auto fail = [&](const char* w) { if (why) *why = w; return false; };
    if (!head.valid) return fail("headset not tracked");
    if (!devices[kCalLeftFoot].valid || !devices[kCalRightFoot].valid) return fail("both foot trackers are needed");
    if (head.pos.z < 0.5f) return fail("headset below 0.5 m: stand up");
    Calibration c;
    c.playerEyeHeight = head.pos.z;
    c.avatarEyeHeight = m_eye.z;
    c.scale = c.avatarEyeHeight / c.playerEyeHeight;
    const Quat yaw = YawOf(head.rot);
    c.yawInverse = Conjugate(yaw);
    if (headAnchored) {
        // (head - origin) turned and scaled must be the avatar's eye point: origin = head - yaw * eye / scale (on the floor).
        const Vec3 eyeRoom = Rotate(yaw, Vec3{m_eye.x, m_eye.y, 0} * (1.0f / c.scale));
        c.origin = {head.pos.x - eyeRoom.x, head.pos.y - eyeRoom.y, 0};
    } else {
        const Vec3 l = devices[kCalLeftFoot].pos, r = devices[kCalRightFoot].pos;
        c.origin = {(l.x + r.x) * 0.5f, (l.y + r.y) * 0.5f, 0};
    }
    c.headRotInverse = Conjugate(Normalize(c.yawInverse * head.rot));
    for (int i = 0; i < kCalCount; ++i) {
        if (!devices[i].valid) continue;
        const Vec3 p = Rotate(c.yawInverse, devices[i].pos - c.origin) * c.scale;
        const Quat q = Normalize(c.yawInverse * devices[i].rot);
        c.offset[i] = m_boneRefPos[i] - p;
        c.rotInverse[i] = Conjugate(q);
        c.valid[i] = true;
    }
    c.done = true;
    MeasureArms(c, head, devices);
    *out = c;
    return true;
}

Vec3 Calibrator::ToModel(const Calibration& c, Vec3 roomPos) const {
    return Rotate(c.yawInverse, roomPos - c.origin) * c.scale;
}

void Calibrator::Goals(const Calibration& c, const RoomPose& head, const RoomPose devices[kCalCount], Goal goals[kGoalCount]) const {
    for (int g = 0; g < kGoalCount; ++g) goals[g].active = false;
    if (!c.done) return;
    for (int i = 0; i < kCalCount; ++i) {
        if (!c.valid[i] || !devices[i].valid) continue;
        const Vec3 p = ToModel(c, devices[i].pos);
        const Quat delta = Normalize(Normalize(c.yawInverse * devices[i].rot) * c.rotInverse[i]);
        Goal& g = goals[kGoalOf[i]];
        g.active = true;
        if (devices[i].wrist && (i == kCalLeftHand || i == kCalRightHand)) {
            const Quat q = Normalize(c.yawInverse * devices[i].rot);
            g.rot = Normalize(q * Conjugate(IdealWrist(i == kCalLeftHand)) * m_boneRefRot[i]);
            g.pos = p;
            continue;
        }
        if (handsFixedMount && (i == kCalLeftHand || i == kCalRightHand)) {
            const Quat q = Normalize(c.yawInverse * devices[i].rot);
            g.rot = Normalize(q * Conjugate(IdealGrip(i == kCalLeftHand)) * m_boneRefRot[i]);
            g.pos = p + Rotate(q, Vec3{0, 0, handWristBack * c.scale});
            continue;
        }
        g.rot = Normalize(delta * m_boneRefRot[i]);
        g.pos = p + Rotate(delta, c.offset[i]);
    }
    if (head.valid) {
        const Quat delta = Normalize(Normalize(c.yawInverse * head.rot) * c.headRotInverse);
        Goal& g = goals[kGoalHead];
        g.active = true;
        g.rot = Normalize(delta * m_headRef);
        g.pos = ToModel(c, head.pos) - Rotate(g.rot, m_eyeInHead);
    }
}

void Calibrator::MeasureArms(Calibration& c, const RoomPose& head, const RoomPose devices[kCalCount]) const {
    Goal goals[kGoalCount];
    Goals(c, head, devices, goals);
    for (int side = 0; side < 2; ++side) {
        const Goal& hand = goals[side == 0 ? kGoalLeftHand : kGoalRightHand];
        c.armLength[side] = hand.active ? Length(hand.pos - m_shoulderRef[side]) / relaxedReach : 0.0f;
        // Not an arm: a hand far off (held up, out of place) -- left unmeasured rather than wrong.
        if (c.armLength[side] < 0.6f * m_armRef[side] || c.armLength[side] > 1.5f * m_armRef[side]) c.armLength[side] = 0;
    }
}

bool Calibrator::MeasureArmsOut(Calibration& c, const RoomPose& head, const RoomPose devices[kCalCount]) const {
    Goal goals[kGoalCount];
    Goals(c, head, devices, goals);
    const Goal &l = goals[kGoalLeftHand], &r = goals[kGoalRightHand];
    if (!l.active || !r.active) return false;
    // From the span between the hands, not from the avatar's shoulders: the body may have moved since the capture set the
    // origin (a step sideways lengthens one arm and shortens the other: a live capture read 0.65 m against 0.52-0.55 in
    // the lab). Each arm = (span - the avatar's shoulder width) / 2: the arms are taken as equal.
    const Vec3 across = r.pos - l.pos;
    const float span = Length(across), shoulders = Length(m_shoulderRef[1] - m_shoulderRef[0]);
    const float arm = (span - shoulders) * 0.5f, vArm = (m_armRef[0] + m_armRef[1]) * 0.5f;
    // Out to the sides: the hands mostly side by side (the span along the body's right axis), near shoulder height.
    const float shoulderZ = (m_shoulderRef[0].z + m_shoulderRef[1].z) * 0.5f;
    if (span < 1e-3f || across.x / span < 0.9f) return false;
    if (std::fabs(l.pos.z - shoulderZ) > 0.35f * vArm || std::fabs(r.pos.z - shoulderZ) > 0.35f * vArm) return false;
    if (arm < 0.6f * vArm || arm > 1.5f * vArm) return false;
    c.armLength[0] = c.armLength[1] = arm;
    return true;
}

bool Calibrator::MeasureArmOut(Calibration& c, const RoomPose& head, const RoomPose devices[kCalCount], int side,
                               const char** why, float* cosOut, float* ratioOut) const {
    if (side < 0 || side > 1) return false;
    Goal goals[kGoalCount];
    Goals(c, head, devices, goals);
    const Goal& hips = goals[kGoalHips];
    const Goal& neck = goals[kGoalHead];
    const Goal& hand = goals[side == 0 ? kGoalLeftHand : kGoalRightHand];
    if (!neck.active || !hand.active) { if (why) *why = "lost"; return false; }
    // Straight out = the hips' side axis, yaw only; the calibration's own frame without the waist tracker.
    Vec3 out{side == 0 ? -1.0f : 1.0f, 0, 0};
    if (hips.active) {
        const Vec3 o = Rotate(Normalize(hips.rot * Conjugate(m_boneRefRot[kCalWaist])), out);
        const float n = std::sqrt(o.x * o.x + o.y * o.y);
        if (n > 0.3f) out = {o.x / n, o.y / n, 0};
    }
    // The shoulder from the head joint: the avatar's drop below it, half the avatar's shoulder width towards the hand, the avatar's forward offset.
    const Vec3 ref = m_shoulderRef[side] - m_headRefPos;
    Vec3 u{hand.pos.x - neck.pos.x, hand.pos.y - neck.pos.y, 0};
    const float un = std::sqrt(u.x * u.x + u.y * u.y);
    u = un > 1e-3f ? Vec3{u.x / un, u.y / un, 0} : out;
    const float s = side == 0 ? -1.0f : 1.0f;            // u is +x of the reference for the right arm, -x for the left
    const Vec3 fwd{-s * u.y, s * u.x, 0};                // the reference's +y with u as its side axis
    const Vec3 shoulder = neck.pos + u * (s * ref.x) + fwd * ref.y + Vec3{0, 0, ref.z};
    const Vec3 d = hand.pos - shoulder;
    const float length = Length(d);
    const float cosine = length < 1e-3f ? 0.0f : Dot(d, out) / length;
    if (cosOut) *cosOut = cosine;
    if (ratioOut) *ratioOut = m_armRef[side] > 0 ? length / m_armRef[side] : 0.0f;
    if (cosine < 0.85f) { if (why) *why = "dir"; return false; }
    if (length < 0.6f * m_armRef[side] || length > 1.5f * m_armRef[side]) { if (why) *why = "length"; return false; }
    c.armLength[side] = length;
    return true;
}

}  // namespace wrpuppets::fbt
