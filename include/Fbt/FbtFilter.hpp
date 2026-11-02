#pragma once

// DEVICE FILTER for full-body tracking (waist, feet, hands; never the headset, which is the view): the 1-euro filter
// (Casiez, Roussel, Vogel, "1 Euro Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive
// Systems", CHI 2012) on position and rotation -- a low cut-off when the device is slow (jitter smoothed), rising with
// its speed (little lag when it moves fast) -- after a rejection of physically impossible one-frame steps (a puck's
// rotation jumping 40-60 deg in 11 ms while it moves 1-3 cm, a controller jumping 16 cm: lab recording). A rejected
// step holds the previous pose for at most a few frames; if the device really is there, it is then accepted and eased
// in by the filter.
#include "Fbt/FbtCalibration.hpp"

namespace wrpuppets::fbt {

struct FilterOptions {
    bool enabled = true;
    float posMinCutoff = 2.0f;    // Hz, at rest
    float posBeta = 4.0f;         // cut-off rise per m/s
    float rotMinCutoff = 2.0f;    // Hz, at rest
    float rotBeta = 0.05f;        // cut-off rise per deg/s
    float derivCutoff = 1.0f;     // Hz, for the speed estimate
    float rejectStepDeg = 25.0f;  // a larger rotation step in one frame (at 90 Hz) is held
    float rejectStepM = 0.08f;    // a larger position step in one frame is held
    int maxHoldFrames = 6;        // 66 ms: the crouch noise comes in bursts; longer holds make the catch-up step worse
    bool holdLost = true;         // a lost device keeps its last pose until found (then eases in); false: it drops out
    bool holdFrozen = false;      // a rejected step whose pose then stays bit-for-bit still is held past maxHoldFrames: a
                                  // runtime's placeholder pose, not tracking (the headset while the dashboard is up)
};
// The headset as the IK's head goal (the view itself is never filtered): no smoothing, only the impossible steps held
// -- 2.4 m and 100 deg in a frame, frozen for two seconds (lab recording, the SteamVR dashboard) -- for as long
// as the pose stays frozen.
inline FilterOptions HeadGoalFilter() {
    FilterOptions o;
    o.posMinCutoff = o.rotMinCutoff = 1000.0f;   // pass-through
    o.rejectStepM = 0.5f;
    o.rejectStepDeg = 90.0f;
    o.maxHoldFrames = 180;
    o.holdFrozen = true;
    return o;
}

class PoseFilter {
public:
    void Reset() { *this = {}; }
    // dt: seconds since the previous sample. Returns the filtered pose (invalid passes through and resets).
    RoomPose Filter(const RoomPose& in, float dt, const FilterOptions& o);
    int Rejected() const { return m_rejected; }   // samples held so far
    int LostFrames() const { return m_lostFrames; }   // frames a lost device was held
    bool Lost() const { return m_lost; }              // holding right now

private:
    bool m_have = false;
    RoomPose m_out, m_lastRaw;
    float m_posSpeed = 0, m_rotSpeed = 0;   // filtered speeds, m/s and deg/s
    int m_hold = 0, m_rejected = 0, m_lostFrames = 0;
    RoomPose m_lastRejected;
    bool m_lost = false;
};

}  // namespace wrpuppets::fbt
