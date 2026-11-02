#include "Fbt/FbtFilter.hpp"
#include <algorithm>
#include <cmath>

namespace wrpuppets::fbt {
namespace {
constexpr float kPi = 3.14159265f;
// The 1-euro smoothing factor for a cut-off frequency and a time step.
float Alpha(float cutoff, float dt) {
    const float tau = 1.0f / (2.0f * kPi * cutoff);
    return 1.0f / (1.0f + tau / dt);
}
float AngleDeg(Quat a, Quat b) {
    const float d = std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * std::acos(std::min(1.0f, d)) * 57.29578f;
}
Quat Slerp(Quat a, Quat b, float t) {
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0) { b = {-b.x, -b.y, -b.z, -b.w}; d = -d; }
    if (d > 0.9995f) return Normalize(Quat{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t});
    const float th = std::acos(d), s = std::sin(th);
    const float wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
    return Normalize(Quat{a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb});
}
}  // namespace

RoomPose PoseFilter::Filter(const RoomPose& in, float dt, const FilterOptions& o) {
    if (!in.valid) {
        // Lost (occluded: a crouch hides the waist and feet): the last pose holds until the device is back -- dropping
        // the goal would snap its bone to the avatar's reference pose (lab recording: 135 deg in one frame).
        if (!o.holdLost || !m_have) { const int r = m_rejected, l = m_lostFrames; *this = {}; m_rejected = r; m_lostFrames = l; return in; }
        m_lost = true;
        ++m_lostFrames;
        return m_out;
    }
    if (m_have && in.wrist != m_out.wrist) {   // the hand's source changed (wrist joint <-> grip): a different point
        const int r = m_rejected, l = m_lostFrames;
        *this = {};
        m_rejected = r;
        m_lostFrames = l;
    }
    if (m_lost && o.enabled && dt > 0) {
        // Found again: ease in from the held pose at the filter's resting cut-off; the step back is no motion, so it
        // must neither count as speed (which would open the cut-off and snap) nor be rejected as a jump.
        m_lost = false;
        m_hold = 0;
        m_posSpeed = m_rotSpeed = 0;
        m_lastRaw = in;
    }
    m_lost = false;
    if (!o.enabled || !m_have || dt <= 0) {
        m_have = true;
        m_out = m_lastRaw = in;
        m_posSpeed = m_rotSpeed = 0;
        return in;
    }
    // Reject an impossible one-frame step (scaled to 90 Hz), for at most maxHoldFrames in a row.
    const float frames = std::max(1.0f, dt * 90.0f);
    const bool jump = AngleDeg(in.rot, m_lastRaw.rot) > o.rejectStepDeg * frames || Length(in.pos - m_lastRaw.pos) > o.rejectStepM * frames;
    const bool frozen = o.holdFrozen && m_hold > 0 && Length(in.pos - m_lastRejected.pos) < 1e-6f && AngleDeg(in.rot, m_lastRejected.rot) < 1e-4f;
    if (jump && (m_hold < o.maxHoldFrames || frozen)) {
        ++m_hold;
        ++m_rejected;
        m_lastRejected = in;
        return m_out;
    }
    m_hold = 0;
    // Speeds from the raw steps, smoothed (the 1-euro derivative filter), then the adaptive cut-offs.
    const float rawPosSpeed = Length(in.pos - m_lastRaw.pos) / dt, rawRotSpeed = AngleDeg(in.rot, m_lastRaw.rot) / dt;
    const float ad = Alpha(o.derivCutoff, dt);
    m_posSpeed += (rawPosSpeed - m_posSpeed) * ad;
    m_rotSpeed += (rawRotSpeed - m_rotSpeed) * ad;
    const float ap = Alpha(o.posMinCutoff + o.posBeta * m_posSpeed, dt), ar = Alpha(o.rotMinCutoff + o.rotBeta * m_rotSpeed, dt);
    m_out.valid = true;
    m_out.pos = m_out.pos + (in.pos - m_out.pos) * ap;
    m_out.rot = Slerp(m_out.rot, in.rot, ar);
    m_lastRaw = in;
    return m_out;
}

}  // namespace wrpuppets::fbt
