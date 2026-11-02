#include "Fbt/FbtIk.hpp"
#include <algorithm>
#include <cmath>

namespace wrpuppets::fbt {

void SetModel(const Skeleton& skeleton, Pose& pose, int bone, Vec3 modelPos, Quat modelRot) {
    const int parent = skeleton.BoneAt(bone).parent;
    const Quat parentRot = parent >= 0 ? pose.modelRot[parent] : Quat{};
    const Vec3 parentPos = parent >= 0 ? pose.modelPos[parent] : Vec3{};
    const Quat inverse = Conjugate(parentRot);
    pose.localPos[bone] = Rotate(inverse, modelPos - parentPos);
    pose.localRot[bone] = Normalize(inverse * modelRot);
    pose.modelPos[bone] = modelPos;
    pose.modelRot[bone] = Normalize(modelRot);
}

namespace {
// The foot-driven knee plane fades out between these lengths of the foot's lateral axis across the leg line (1 = the
// axis perpendicular to the leg, 0 = along it), handing over to the hips' forward.
constexpr float kPlaneFadeStart = 0.35f, kPlaneFadeEnd = 0.10f;
}

int SolveLimbFabrik(const Skeleton& sk, Pose& pose, const Chain& chain, Vec3 target, Quat endRot, int iterations,
                    float precision, float* error, const Vec3* initial) {
    const int n = chain.count;
    Vec3 p[16];
    for (int i = 0; i < n; ++i) p[i] = initial && i > 0 ? initial[i] : pose.modelPos[chain.bones[i]];
    const Vec3 root = p[0];
    // Out of reach needs no special case: the passes straighten the chain towards the target by themselves, and a
    // separate straight-line answer would not meet theirs at the border (the shape would snap).
    int used = 0;
    for (; used < iterations && Length(p[n - 1] - target) > precision; ++used) {
        p[n - 1] = target;                                               // backward: from the end to the root
        for (int i = n - 2; i >= 0; --i) p[i] = p[i + 1] + Normalize(p[i] - p[i + 1]) * chain.length[i];
        p[0] = root;                                                     // forward: from the root to the end
        for (int i = 1; i < n; ++i) p[i] = p[i - 1] + Normalize(p[i] - p[i - 1]) * chain.length[i - 1];
    }
    // Positions to rotations, root first: each bone's current direction to its child swings onto the solved one.
    Quat boneRot = pose.modelRot[chain.bones[0]];   // the bone's current model rotation
    Vec3 at = root;
    for (int i = 0; i + 1 < n; ++i) {
        const int bone = chain.bones[i], child = chain.bones[i + 1];
        if (i > 0) {
            const int parent = sk.BoneAt(bone).parent;
            at = pose.modelPos[parent] + Rotate(pose.modelRot[parent], pose.localPos[bone]);
            boneRot = Normalize(pose.modelRot[parent] * pose.localRot[bone]);
        }
        const Vec3 current = Rotate(boneRot, pose.localPos[child]);
        const Quat rot = Normalize(FromTo(current, p[i + 1] - at) * boneRot);
        SetModel(sk, pose, bone, at, rot);
    }
    const int end = chain.bones[n - 1], endParent = sk.BoneAt(end).parent;
    const Vec3 endPos = pose.modelPos[endParent] + Rotate(pose.modelRot[endParent], pose.localPos[end]);
    SetModel(sk, pose, end, endPos, endRot);
    if (error) *error = Length(endPos - target);
    return used;
}

bool MakeTwoBone(const Skeleton& sk, int upper, int lower, int end, Vec3 refBend, TwoBone* out) {
    if (upper < 0 || lower < 0 || end < 0 || sk.BoneAt(lower).parent != upper || sk.BoneAt(end).parent != lower) return false;
    TwoBone t;
    t.upper = upper;
    t.lower = lower;
    t.end = end;
    const Bone &u = sk.BoneAt(upper), &l = sk.BoneAt(lower), &e = sk.BoneAt(end);
    t.a = Length(l.refLocalPos);
    t.b = Length(e.refLocalPos);
    const Vec3 line = Normalize(e.refModelPos - u.refModelPos);
    const Vec3 hinge = Normalize(Cross(line, refBend - line * Dot(refBend, line)));
    t.upperAlong = Normalize(l.refLocalPos);
    t.lowerAlong = Normalize(e.refLocalPos);
    t.upperHinge = Rotate(Conjugate(u.refModelRot), hinge);
    t.lowerHinge = Rotate(Conjugate(l.refModelRot), hinge);
    *out = t;
    return t.a > 0 && t.b > 0;
}

float SolveTwoBone(const Skeleton& sk, Pose& pose, const TwoBone& c, Vec3 target, Vec3 bendDir, Quat endRot, float softness) {
    const Vec3 root = pose.modelPos[c.upper];
    const Vec3 line = Normalize(target - root);
    const float full = c.a + c.b;
    float d = Length(target - root);
    const float reach = full > 0 ? d / full : 0;
    const float soft = std::max(0.0f, softness) * full, start = full - soft;
    // Soft reach: from (1 - s) of the length the distance eases along a cubic (slope 1 there, 0 at the full length) and
    // arrives at full extension exactly at the length -- the limb straightens when the player's does (an earlier curve
    // straightened only at 1.03: "the avatar's arms never straight"), and near straight, where 1 mm moves the joint by degrees,
    // the slope is near zero so tracking jitter is damped.
    if (soft > 1e-4f && d > start) {
        const float t = std::min((d - start) / soft, 1.0f);
        d = start + soft * (t + t * t - t * t * t);
    }
    d = std::clamp(d, std::fabs(c.a - c.b) + 1e-3f, full - 1e-4f);
    const Vec3 bend = Normalize(bendDir - line * Dot(bendDir, line));
    const float cosRoot = std::clamp((c.a * c.a + d * d - c.b * c.b) / (2 * c.a * d), -1.0f, 1.0f);
    const Vec3 mid = root + (line * cosRoot + bend * std::sqrt(1.0f - cosRoot * cosRoot)) * c.a;
    const Vec3 endPos = root + line * d;
    const Vec3 hinge = Normalize(Cross(line, bend));
    SetModel(sk, pose, c.upper, root, AimRoll(c.upperAlong, c.upperHinge, mid - root, hinge));
    SetModel(sk, pose, c.lower, mid, AimRoll(c.lowerAlong, c.lowerHinge, endPos - mid, hinge));
    SetModel(sk, pose, c.end, endPos, endRot);
    return reach;
}

int SolveLimbDistributed(const Skeleton& sk, Pose& pose, const Chain& chain, const float* weights, Vec3 target, Quat endRot,
                         float strength, int iterations, float precision, float* error) {
    const int n = chain.count, end = chain.bones[n - 1];
    // Each bone's model transform from its parent's (only the chain changes; its parents' are current).
    auto refresh = [&](int from) {
        for (int i = from; i < n; ++i) {
            const int b = chain.bones[i], parent = sk.BoneAt(b).parent;
            pose.modelPos[b] = pose.modelPos[parent] + Rotate(pose.modelRot[parent], pose.localPos[b]);
            pose.modelRot[b] = Normalize(pose.modelRot[parent] * pose.localRot[b]);
        }
    };
    refresh(1);
    int used = 0;
    for (; used < iterations && Length(pose.modelPos[end] - target) > precision; ++used) {
        float remaining = 0;
        for (int i = 0; i + 1 < n; ++i) remaining += weights[i];
        for (int i = 0; i + 1 < n; ++i) {
            const int b = chain.bones[i];
            const Vec3 joint = pose.modelPos[b];
            const Vec3 toEnd = pose.modelPos[end] - joint, toTarget = target - joint;
            if (Length(toEnd) < 1e-4f || Length(toTarget) < 1e-4f) { remaining -= weights[i]; continue; }
            const float share = remaining > 1e-6f ? weights[i] / remaining : 1.0f;
            remaining -= weights[i];
            const Quat full = FromTo(toEnd, toTarget);
            const float angle = 2.0f * std::acos(std::clamp(full.w, -1.0f, 1.0f));
            // Nearly opposite: the swing's axis is arbitrary (a flip); leave it to the other joints and later passes.
            if (angle < 1e-6f || angle > 2.618f) continue;
            const Quat part = AxisAngle({full.x, full.y, full.z}, std::min(angle * share, 0.349f) * strength);   // at most 20 deg per pass
            SetModel(sk, pose, b, joint, Normalize(part * pose.modelRot[b]));
            refresh(i + 1);
        }
    }
    const int endParent = sk.BoneAt(end).parent;
    SetModel(sk, pose, end, pose.modelPos[endParent] + Rotate(pose.modelRot[endParent], pose.localPos[end]), endRot);
    if (error) *error = Length(pose.modelPos[end] - target);
    return used;
}

bool IkStack::Init(const Skeleton& skeleton) {
    m_skeleton = &skeleton;
    m_hips = skeleton.Role("hips");
    m_upLeg[0] = skeleton.Role("left_upleg");
    m_upLeg[1] = skeleton.Role("right_upleg");
    m_leg[0] = skeleton.Role("left_leg");
    m_leg[1] = skeleton.Role("right_leg");
    m_foot[0] = skeleton.Role("left_foot");
    m_foot[1] = skeleton.Role("right_foot");
    for (int side = 0; side < 2; ++side) {
        if (m_upLeg[side] < 0 || m_leg[side] < 0 || m_foot[side] < 0) return false;
        const Bone& foot = skeleton.BoneAt(m_foot[side]);
        m_footSideLocal[side] = Rotate(Conjugate(foot.refModelRot), {1, 0, 0});
        m_footForwardLocal[side] = Rotate(Conjugate(foot.refModelRot), {0, 1, 0});
        if (!MakeTwoBone(skeleton, m_upLeg[side], m_leg[side], m_foot[side], {0, 1, 0}, &m_legChain[side])) return false;   // knees bend forward
        // Arms: upper arm -> forearm -> hand; in the T-pose the elbow points back. The thumb side of the hand from the
        // first thumb joint, across the hand's own axis (wrist -> hand end).
        const bool left = side == 0;
        const int upper = skeleton.Role(left ? "left_upperarm" : "right_upperarm"), fore = skeleton.Role(left ? "left_forearm" : "right_forearm"),
                  hand = skeleton.Role(left ? "left_hand" : "right_hand");
        if (!MakeTwoBone(skeleton, upper, fore, hand, {0, -1, 0}, &m_armChain[side])) return false;
        m_clavicle[side] = skeleton.BoneAt(upper).parent;
        const Bone& h = skeleton.BoneAt(hand);
        // Optional roles: the thumb's first joint and the hand's end (fingertips side); without them the hand's long axis
        // is taken as straight out along x and the thumb side as forward, which a T-pose with the palms down gives.
        const int thumb = skeleton.Role(left ? "left_thumb1" : "right_thumb1"), handEnd = skeleton.Role(left ? "left_hand_end" : "right_hand_end");
        const Vec3 along = handEnd >= 0 ? Normalize(skeleton.BoneAt(handEnd).refModelPos - h.refModelPos) : Vec3{left ? -1.0f : 1.0f, 0, 0};
        Vec3 toThumb = thumb >= 0 ? skeleton.BoneAt(thumb).refModelPos - h.refModelPos : Vec3{0, 1, 0};
        toThumb = Normalize(toThumb - along * Dot(toThumb, along));
        m_handThumbLocal[side] = Rotate(Conjugate(h.refModelRot), toThumb);
        // The radial axis: the thumb direction without its tilt towards the palm (the avatar's T-pose palm faces down: model z
        // dropped), across the hand's long axis -- where the fist axis (the grip's -Z) points.
        Vec3 radial = toThumb - Vec3{0, 0, 1} * toThumb.z;
        radial = Normalize(radial - along * Dot(radial, along));
        m_handRadialLocal[side] = Rotate(Conjugate(h.refModelRot), radial);
        m_handPalmLocal[side] = Rotate(Conjugate(h.refModelRot), {0, 0, -1});
        m_handAlongLocal[side] = Rotate(Conjugate(h.refModelRot), along);
        m_handRefLocal[side] = h.refLocalRot;
        {   // the hand's long axis against the forearm's in the reference pose (the avatar's rest hand is not aligned with it)
            const Bone& f = skeleton.BoneAt(fore);
            m_handRefBend[side] = std::acos(std::clamp(Dot(along, Normalize(h.refModelPos - f.refModelPos)), -1.0f, 1.0f));
        }
    }
    if (m_hips < 0) return false;
    m_hipsForwardLocal = Rotate(Conjugate(skeleton.BoneAt(m_hips).refModelRot), {0, 1, 0});
    m_hipsSideLocal = Rotate(Conjugate(skeleton.BoneAt(m_hips).refModelRot), {1, 0, 0});
    // The spine chain: its root (as in WrPuppets) .. the last spine bone, the neck bones, the head.
    m_head = skeleton.Role("head");
    m_spine = {};
    const char* spineRoles[] = {"spine0", "spine1", "spine2", "spine3", "spine4", "spine5", "spine6", "spine7", "neck", "neck1", "head"};
    for (const char* role : spineRoles)
        if (const int b = skeleton.Role(role); b >= 0 && m_spine.count < 16) m_spine.bones[m_spine.count++] = b;
    for (int i = 0; i + 1 < m_spine.count; ++i) {
        if (skeleton.BoneAt(m_spine.bones[i + 1]).parent != m_spine.bones[i]) return false;   // not a parent chain
        m_spine.length[i] = Length(skeleton.BoneAt(m_spine.bones[i + 1]).refLocalPos);
        m_spine.total += m_spine.length[i];
        // Distributed spine: the spine bones share the bend evenly, the neck a little more, its top a little less.
        const int b = m_spine.bones[i];
        m_spineWeight[i] = b == skeleton.Role("neck") ? 1.2f : b == skeleton.Role("neck1") ? 0.8f : 1.0f;
    }
    return m_head >= 0 && m_spine.count >= 3;
}

void IkStack::Solve(Pose& pose, const Goal goals[kGoalCount], const StackOptions& options, StackDebug* debug) const {
    if (!m_skeleton) return;
    const Skeleton& sk = *m_skeleton;
    sk.Fk(pose);

    // 1. The hips on the waist goal: its position and rotation; everything below follows.
    if (options.setHips && goals[kGoalHips].active && m_hips >= 0) {
        SetModel(sk, pose, m_hips, goals[kGoalHips].pos, goals[kGoalHips].rot);
        sk.Fk(pose);
    }

    // 4f. The hips give way (the full-body arbitration of WrPuppets): the waist tracker places the hips, but a leg too
    // short for its ankle goal, or a spine too short for the head goal, pulls them. Least squares on the hips' shift
    // d (their rotation stays the tracker's; the legs' and the spine's roots move with them):
    //   E(d) = w_hips |d|^2 + sum over the feet and the head of w * max(0, |root + d - goal| - reach)^2
    // one-sided (a goal within reach does not pull), reach = a share of the chain's length (just below where the soft
    // reach starts bending away from the goal). Gauss-Newton on the active terms, a few steps (3 unknowns).
    if (options.hipsGiveWay && options.setHips && goals[kGoalHips].active && m_hips >= 0) {
        struct Term { Vec3 root, goal; float reach, weight; } terms[3];
        int count = 0;
        for (int side = 0; side < 2 && options.legs; ++side) {
            const Goal& foot = goals[side == 0 ? kGoalLeftFoot : kGoalRightFoot];
            const TwoBone& c = m_legChain[side];
            if (foot.active) terms[count++] = {pose.modelPos[c.upper], foot.pos, (c.a + c.b) * options.legReachShare, options.footPull};
        }
        if (options.spine && goals[kGoalHead].active && m_spine.count >= 3)
            terms[count++] = {pose.modelPos[m_spine.bones[0]], goals[kGoalHead].pos, m_spine.total * options.spineReachShare, options.headPull};
        // The cost itself, for the line search: convex (a stiffness plus squared overshoots of convex distances), so it
        // has one minimum, which moves smoothly with the goals.
        auto cost = [&](Vec3 at) {
            float e = options.hipsStiffness * Dot(at, at);
            for (int i = 0; i < count; ++i) {
                const float over = Length(terms[i].root + at - terms[i].goal) - terms[i].reach;
                if (over > 0) e += terms[i].weight * over * over;
            }
            return e;
        };
        Vec3 d{};
        float current = cost(d);
        for (int it = 0; it < 16 && count > 0; ++it) {
            // Normal equations of the linearised problem: (w_hips I + sum w u u^T) step = -(w_hips d + sum w e u).
            float A[3][3] = {{options.hipsStiffness, 0, 0}, {0, options.hipsStiffness, 0}, {0, 0, options.hipsStiffness}};
            Vec3 g = d * options.hipsStiffness;
            bool any = false;
            for (int i = 0; i < count; ++i) {
                const Vec3 v = terms[i].root + d - terms[i].goal;
                const float len = Length(v), e = len - terms[i].reach;
                if (e <= 0 || len < 1e-5f) continue;
                any = true;
                const Vec3 u = v * (1.0f / len);
                const float uu[3] = {u.x, u.y, u.z};
                for (int r = 0; r < 3; ++r)
                    for (int k = 0; k < 3; ++k) A[r][k] += terms[i].weight * uu[r] * uu[k];
                g = g + u * (terms[i].weight * e);
            }
            if (!any && it > 0) break;
            // 3x3 solve (Cramer; A is symmetric positive definite: stiffness > 0).
            const float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                              A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
            if (std::fabs(det) < 1e-12f) break;
            const float b[3] = {-g.x, -g.y, -g.z};
            float x[3];
            for (int col = 0; col < 3; ++col) {
                float M[3][3];
                for (int r = 0; r < 3; ++r)
                    for (int k = 0; k < 3; ++k) M[r][k] = k == col ? b[r] : A[r][k];
                x[col] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                          M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
            }
            // Backtracking: a Gauss-Newton step on a one-sided cost overshoots when a term switches on or off on the
            // way (a leg starting or stopping to pull), and an overshoot left by the iteration limit was a jump of the
            // hips between neighbouring goals (stress test: 3 cm in a 5 mm foot step). Halve it until the cost drops.
            Vec3 step{x[0], x[1], x[2]};
            float next = cost(d + step);
            for (int halve = 0; halve < 12 && next > current; ++halve) {
                step = step * 0.5f;
                next = cost(d + step);
            }
            if (next > current) break;   // no descent left: at the minimum
            d = d + step;
            current = next;
            if (Length(step) < 1e-5f) break;
        }
        if (debug) debug->hipsShift = d;
        if (Length(d) > 0) {
            SetModel(sk, pose, m_hips, pose.modelPos[m_hips] + d, pose.modelRot[m_hips]);
            sk.Fk(pose);
        }
    } else if (debug) {
        debug->hipsShift = {};
    }

    // 2-3. The legs' bending plane (used by the leg solve): the knee hinge is the foot's lateral axis, made
    // perpendicular to the hip -> ankle-goal line; the knee points along hinge x line, forward for a standing leg
    // (WrPuppets' knee-to-foot trick, immune to pointed toes and lifted heels). When the foot's lateral
    // axis comes close to the leg line (foot rolled or turned ~90 degrees to the leg) that plane fades out and the
    // hips' forward takes over, blended so the knee never pops; the foot's forward is the last resort.
    const Vec3 hipsForward = Rotate(pose.modelRot[m_hips], m_hipsForwardLocal);
    for (int side = 0; side < 2; ++side) {
        const Goal& foot = goals[side == 0 ? kGoalLeftFoot : kGoalRightFoot];
        if (debug) debug->legPlaneValid[side] = false;
        if (!foot.active || m_upLeg[side] < 0) continue;
        const Vec3 hip = pose.modelPos[m_upLeg[side]];
        const Vec3 line = Normalize(foot.pos - hip);
        if (Length(foot.pos - hip) < 1e-4f) continue;
        auto across = [&](Vec3 v) { return v - line * Dot(v, line); };   // the part of v perpendicular to the leg line
        const Vec3 footForward = Rotate(foot.rot, m_footForwardLocal[side]);
        const Vec3 n = across(Rotate(foot.rot, m_footSideLocal[side]));
        // lateral x leg line already points the knee forward (+x across a downward leg gives +y); a sign test against the
        // foot's forward flipped the knee 180 deg in deep bends, where the two are nearly perpendicular (recording).
        const Vec3 fromFoot = Normalize(Cross(n, line));
        Vec3 fallback = across(hipsForward);
        if (Length(fallback) < 0.2f) fallback = across(footForward);
        fallback = Normalize(fallback);
        const float w = std::clamp((Length(n) - kPlaneFadeEnd) / (kPlaneFadeStart - kPlaneFadeEnd), 0.0f, 1.0f);
        Vec3 knee = Normalize(across(fromFoot * w + fallback * (1.0f - w)));
        if (Length(knee) < 0.5f) knee = fallback;
        if (debug) {
            debug->legPlaneValid[side] = true;
            debug->legPlaneOrigin[side] = (hip + foot.pos) * 0.5f;
            debug->legKneeDir[side] = knee;
        }

        // 4 (legs; the hips' give-way comes with the legs' solve). Two bones, hips pinned, the knee in the plane
        // above, soft reach near full extension, the foot on the goal's rotation.
        if (!options.legs) continue;
        const float reach = SolveTwoBone(sk, pose, m_legChain[side], foot.pos, knee, foot.rot, options.legSoftness);
        if (debug) debug->legReach[side] = reach;
    }
    sk.Fk(pose);

    // 5. The spine to the head: the head on the headset goal (position by FABRIK, rotation set).
    if (options.spine && goals[kGoalHead].active && m_spine.count >= 3) {
        float error = 0;
        // Compressed (the head goal nearer the spine's root than the spine reaches), the spine first curls forward: every
        // joint but the head takes the same flexion about the hips' lateral axis, as much as brings the reach to the goal's
        // distance (bisection). A rounding back, the same fold every frame -- FABRIK alone folds the chain wherever its
        // passes happen to (the neck jumped 9 deg between frames 1 mm apart: stress test "head down"). With no
        // compression nothing curls, so the solve is unchanged and continuous across the border. The curled spine turns
        // at its root back onto the input's root -> head line, so the curl only shortens it.
        // Then distributed passes (they spread a lean or a slouch along the spine) and FABRIK from there lands the head
        // exactly: it only has the remainder to place, so little piles up in the neck.
        // Without the curl the passes fade out as the spine has to shorten (gone at 5 %): they would leave FABRIK a
        // twisted start (stress test: 136 deg flips with the hips rising under a fixed head).
        int used = 0;
        const int n = m_spine.count;
        const Vec3 root = pose.modelPos[m_spine.bones[0]];
        const float goalDist = Length(goals[kGoalHead].pos - root);
        float reach = Length(pose.modelPos[m_spine.bones[n - 1]] - root);
        if (options.spineCurl && goalDist < reach) {
            const Vec3 lateral = Normalize(Rotate(pose.modelRot[m_hips], m_hipsSideLocal));
            const Vec3 inputEnd = pose.modelPos[m_spine.bones[n - 1]];
            Quat inputLocal[16];
            for (int i = 0; i < n; ++i) inputLocal[i] = pose.localRot[m_spine.bones[i]];
            auto curl = [&](float beta) {   // flexion per joint, radians; returns the root -> end distance
                const Quat q = AxisAngle(lateral, -beta);
                for (int i = 0; i < n; ++i) {
                    const int b = m_spine.bones[i], parent = sk.BoneAt(b).parent;
                    const Vec3 pos = pose.modelPos[parent] + Rotate(pose.modelRot[parent], pose.localPos[b]);
                    const Quat rot = Normalize(pose.modelRot[parent] * inputLocal[i]);
                    SetModel(sk, pose, b, pos, i + 1 < n ? Normalize(q * rot) : rot);
                }
                return Length(pose.modelPos[m_spine.bones[n - 1]] - root);
            };
            // The total flexion stays under 180 deg (further, the chain curls back on itself and its reach grows again).
            float lo = 0, hi = 3.0f / static_cast<float>(n - 1);
            if (curl(hi) > goalDist) lo = hi;   // nearer than even that: the tightest curl, FABRIK finishes from it
            for (int it = 0; it < 30 && lo < hi; ++it) {
                const float mid = (lo + hi) * 0.5f;
                if (curl(mid) > goalDist) lo = mid; else hi = mid;
            }
            reach = curl(lo);
            // Back onto the input's line: the curl only shortens the spine, the passes below aim it as without one.
            const Vec3 curled = pose.modelPos[m_spine.bones[n - 1]] - root;
            const Quat back = FromTo(curled, inputEnd - root);
            const int b0 = m_spine.bones[0];
            SetModel(sk, pose, b0, root, Normalize(back * pose.modelRot[b0]));
            for (int i = 1; i < n; ++i) {
                const int b = m_spine.bones[i], parent = sk.BoneAt(b).parent;
                pose.modelPos[b] = pose.modelPos[parent] + Rotate(pose.modelRot[parent], pose.localPos[b]);
                pose.modelRot[b] = Normalize(pose.modelRot[parent] * pose.localRot[b]);
            }
        }
        if (options.spineDistributed) {
            const float ratio = reach > 1e-3f ? goalDist / reach : 1.0f;
            const float t = std::clamp((ratio - 0.95f) / 0.05f, 0.0f, 1.0f), strength = t * t * (3.0f - 2.0f * t);
            if (strength > 0)
                used = SolveLimbDistributed(sk, pose, m_spine, m_spineWeight, goals[kGoalHead].pos, goals[kGoalHead].rot, strength, 4,
                                            options.limbPrecision, &error);
        }
        used += SolveLimbFabrik(sk, pose, m_spine, goals[kGoalHead].pos, goals[kGoalHead].rot, options.limbIterations,
                                options.limbPrecision, &error);
        if (debug) {
            debug->spineIterations = used;
            debug->spineError = error;
            debug->spineReach = m_spine.total > 0 ? Length(goals[kGoalHead].pos - pose.modelPos[m_spine.bones[0]]) / m_spine.total : 0;
        }
        sk.Fk(pose);
    }

    // The arms scaled to the player's: the forearm's and the hand's offsets along their bones (the twist helpers between
    // keep theirs). Done here, after the spine, so the reach checks below see the scaled lengths.
    float armLen[2];
    for (int side = 0; side < 2; ++side) {
        const TwoBone& c = m_armChain[side];
        const float k = std::clamp(options.armScale[side] * options.armLengthTune, 0.5f, 2.0f);
        for (int b : {c.lower, c.end}) pose.localPos[b] = sk.BoneAt(b).refLocalPos * k;
        armLen[side] = (c.a + c.b) * k;
    }
    sk.Fk(pose);

    // 5b. The shoulders protract: a hand beyond the arm's reach pulls the shoulder joint
    // towards it by the excess, on the sphere about the clavicle's root, within the limit; the arm solve then reaches
    // from there (straight: the soft reach arrives at the length).
    for (int side : {1, 0}) {
        const Goal& hand = goals[side == 0 ? kGoalLeftHand : kGoalRightHand];
        if (debug) debug->armExtension[side] = debug->clavicleDeg[side] = 0;
        const int cl = m_clavicle[side];
        if (!options.arms || !hand.active || cl < 0) continue;
        const TwoBone& c = m_armChain[side];
        const Vec3 root = pose.modelPos[cl], shoulder0 = pose.modelPos[c.upper];
        const float radius = Length(shoulder0 - root), d0 = Length(hand.pos - shoulder0), L = armLen[side];
        if (debug) debug->armExtension[side] = L > 1e-3f ? d0 / L : 0;
        if (!options.clavicle || radius < 1e-3f || d0 < 1e-3f || L < 1e-3f) continue;
        // Exactly the excess: nearer than that the move would bend the elbow more (the soft reach bends it until
        // the length), further the hand would float off its goal.
        const float shift = d0 - L;
        if (shift <= 1e-5f) continue;
        Vec3 sh = shoulder0;
        for (int it = 0; it < 4; ++it) {   // the remaining excess each time (the sphere takes part of a step back)
            const Vec3 toHand = hand.pos - sh;
            const float len = Length(toHand), excess = len - L;
            if (len < 1e-4f || excess <= 1e-5f) break;
            sh = root + Normalize(sh + toHand * (excess / len) - root) * radius;   // towards the hand, back on the sphere
        }
        Quat swing = FromTo(shoulder0 - root, sh - root);
        float angle = 2.0f * std::acos(std::clamp(swing.w, -1.0f, 1.0f));
        const float limit = options.clavicleLimitDeg * 0.0174533f;
        if (angle > limit) { swing = AxisAngle({swing.x, swing.y, swing.z}, limit); angle = limit; }
        if (debug) debug->clavicleDeg[side] = angle * 57.29578f;
        if (angle < 1e-5f) continue;
        SetModel(sk, pose, cl, root, Normalize(swing * pose.modelRot[cl]));
        // The arm hangs from the clavicle: its model transforms follow (the solve below reads the upper arm's).
        for (int b : {c.upper, c.lower, c.end}) {
            const int parent = sk.BoneAt(b).parent;
            pose.modelPos[b] = pose.modelPos[parent] + Rotate(pose.modelRot[parent], pose.localPos[b]);
            pose.modelRot[b] = Normalize(pose.modelRot[parent] * pose.localRot[b]);
        }
    }

    // 6-7. The arms (right, then left, as in WrPuppets): upper arm -> forearm -> hand on the hand goal. The elbow points
    // down and a little back -- in WrPuppets the elbow followed the hand through the twist solver's roll (8-9). With
    // options.elbowFromThumb it points away from the hand's thumb instead, pulled down by options.elbowDown (palm down
    // and forward: out and down; palm up: tucked in). Where the direction runs along the arm, down then back take over.
    for (int side : {1, 0}) {
        const Goal& hand = goals[side == 0 ? kGoalLeftHand : kGoalRightHand];
        if (debug) debug->elbowValid[side] = false;
        if (!options.arms || !hand.active) continue;
        const TwoBone& c = m_armChain[side];
        const Vec3 shoulder = pose.modelPos[c.upper];
        if (Length(hand.pos - shoulder) < 1e-4f) continue;
        const Vec3 line = Normalize(hand.pos - shoulder);
        auto across = [&](Vec3 v) { return v - line * Dot(v, line); };
        const Vec3 thumb = Rotate(hand.rot, options.elbowFromRadial ? m_handRadialLocal[side] : m_handThumbLocal[side]);
        const float palmUp = std::max(0.0f, Rotate(hand.rot, m_handPalmLocal[side]).z);
        const float downPull = options.elbowDown + options.elbowDownPalmUp * palmUp;
        Vec3 bend = options.elbowFromThumb ? across(thumb * -1.0f + Vec3{0, 0, -1} * downPull) : across({0, -0.3f, -1});
        if (options.elbowFromPosition) {
            // The elbow from VRArmIK's swivel model (Parger et al.), with its tuning for a standing player: a
            // pure function of the hand's position in the body's frame, so nothing in it can flip with the hand's
            // rotation. A symmetric rest -- the arm hanging slightly forward and out, the elbow back, a little out and
            // down -- swung onto the arm's direction, then turned about it by a zone offset: hands high ease the elbow
            // down, hands low push it back, hands across the body push it out; never forward.
            const Quat body = pose.modelRot[m_hips];
            const Vec3 right = Rotate(body, m_hipsSideLocal), fwd = Rotate(body, m_hipsForwardLocal), up = Cross(right, fwd);
            const Vec3 out = right * (side == 0 ? -1.0f : 1.0f);
            const Vec3 armRest = Normalize(up * -1.0f + fwd * 0.15f + out * 0.10f);
            const Vec3 bendRest = Normalize(fwd * -0.90f + out * 0.35f + up * -0.25f);
            // The swing taking the rest arm onto the arm's direction. Towards straight up it is a 180 deg turn with no
            // axis of its own (the stress test's reach arc stepped 4 deg at the apex): from above horizontal a second
            // swing, through the front -- an arm rises in front -- is blended in with the arm's height, as the bend
            // directions the two give (the rotations themselves can differ by a half turn: blending those jumps); where
            // the two oppose, the blend shrinks and the fade below takes over.
            const Quat swing = FromTo(armRest, line);
            Vec3 ref = across(Rotate(swing, bendRest));
            {
                const float t = std::clamp((-Dot(armRest, line) - 0.3f) / 0.6f, 0.0f, 1.0f), wv = t * t * (3.0f - 2.0f * t);
                if (wv > 0) {
                    const Vec3 front = Normalize(fwd + out * 0.1f);
                    const Vec3 ref2 = across(Rotate(Normalize(FromTo(front, line) * FromTo(armRest, front)), bendRest));
                    ref = ref * (1.0f - wv) + ref2 * wv;
                }
            }
            {
                const Vec3 v = hand.pos - shoulder;
                const float L = armLen[side];
                const float nx = Dot(v, right) / L, ny = Dot(v, up) / L, nz = Dot(v, fwd) / L;
                const float inward = nx * (side == 0 ? 1.0f : -1.0f);
                float dev = -60.0f * ny;
                const float zGate = std::max(0.0f, 0.6f - nz);
                dev += ny > 0 ? 70.0f * zGate * ny : -100.0f * zGate * (-ny);
                dev += -50.0f * std::max(0.0f, inward + 0.1f);
                dev = std::clamp(dev, -60.0f, 25.0f);
                if (!options.elbowZones) dev = 0;   // the rest bend swung onto the arm, nothing else (what a plain chain IK gives on a straight input)
                const float a = dev * 0.0174533f * (side == 0 ? 1.0f : -1.0f);
                const float mag = std::min(Length(ref), 1.0f);   // carried: the fade below reads it
                const Vec3 refN = Normalize(mag > 1e-3f ? ref : across(up * -1.0f)), cxv = Cross(line * -1.0f, refN);   // VRArmIK's xDir is hand -> shoulder
                bend = (refN * std::cos(a) + cxv * std::sin(a)) * std::max(mag, 1e-3f);
                const float fc = Dot(bend, fwd);   // never forward
                if (fc > 0) bend = across(bend - fwd * fc);
            }
            {   // Little left -- the swung rest along the arm line, or stripped of a forward part that was most of it (the
                // hand behind and above the shoulder): it fades into down across the line (or out, where down runs along
                // it too), so it passes through that instead of coming back reversed (lab recording: a 180 deg flip).
                const Vec3 dn = across(up * -1.0f), ot = across(out);
                const Vec3 fb = Normalize(Length(dn) > 0.2f ? dn : ot);
                const float wr = std::clamp((Length(bend) - 0.10f) / 0.25f, 0.0f, 1.0f);
                bend = Normalize(bend) * wr + fb * (1.0f - wr);
                if (Length(bend) < 0.3f) bend = fb;
            }
        }
        // Where the thumb's axis runs near the shoulder -> hand line, "away from it" shrinks and whips round the line
        // (the hand flat on the chest, fingers up: 100 deg in three frames, lab recording): it fades into the
        // arm's rest direction -- down, back and a little out, a relaxed arm -- as it shortens, over a wide band.
        const Vec3 rest = across(Vec3{side == 0 ? -options.elbowRestOut : options.elbowRestOut, -options.elbowRestBack, -1.0f});
        const Vec3 down = across({0, 0, -1}), back = across({0, -1, 0});
        const Vec3 fallback = Normalize(Length(rest) > 0.2f ? rest : (Length(down) > 0.2f ? down : back));
        const float w = std::clamp((Length(bend) - options.elbowFadeEnd) / (options.elbowFadeStart - options.elbowFadeEnd), 0.0f, 1.0f);
        bend = Normalize(Normalize(bend) * w + fallback * (1.0f - w));
        if (Length(bend) < 0.5f) bend = fallback;
        TwoBone scaled = c;   // the arm as scaled to the player's above
        scaled.a = Length(pose.localPos[c.lower]);
        scaled.b = Length(pose.localPos[c.end]);
        const float reach = SolveTwoBone(sk, pose, scaled, hand.pos, bend, hand.rot, options.armSoftness);
        if (debug) {
            debug->elbowValid[side] = true;
            debug->elbowOrigin[side] = (shoulder + hand.pos) * 0.5f;
            debug->elbowDir[side] = bend;
            debug->armReach[side] = reach;
        }
    }
    sk.Fk(pose);

    // 8-9. Twist (WrPuppets' twist solver, both arms): measure the wrist's twist relative to the forearm (against
    // the reference pose: the avatar's rest hand is not aligned with its forearm), roll the elbow about the shoulder -> wrist line by
    // -twist/2 clamped to +-45 deg, then give the forearm half the twist about its own axis. The hand keeps its pose
    // throughout, so it keeps the other half.
    for (int side : {1, 0}) {
        if (debug) debug->twistDeg[side] = debug->elbowRollDeg[side] = 0;
        if (!options.twist || !options.arms || !goals[side == 0 ? kGoalLeftHand : kGoalRightHand].active) continue;
        const TwoBone& c = m_armChain[side];
        const Quat handModel = pose.modelRot[c.end];
        const Vec3 handPos = pose.modelPos[c.end];
        const Quat local = Normalize(Conjugate(pose.modelRot[c.lower]) * handModel);
        const Quat delta = Normalize(local * Conjugate(m_handRefLocal[side]));   // in the forearm's frame
        const Vec3 axis = Normalize(Rotate(local, m_handAlongLocal[side]));
        const float twist = 2.0f * std::atan2(Dot({delta.x, delta.y, delta.z}, axis), delta.w);
        float wrapped = std::remainder(twist, 6.2831853f);
        // The twist about the hand's axis is ill-defined once the hand is bent far against the forearm (the projection
        // above wraps by 180 deg past a 90 deg bend: the hand grabbing the shoulder from above, fingers down the arm --
        // lab recording, 9 forearm flips): it fades out from a 60 deg bend, gone at 90 (the wrist limit is 75).
        {
            const Vec3 handAxis = Rotate(handModel, m_handAlongLocal[side]), foreAxis0 = Normalize(handPos - pose.modelPos[c.lower]);
            const float bend = std::fabs(std::acos(std::clamp(Dot(handAxis, foreAxis0), -1.0f, 1.0f)) - m_handRefBend[side]);
            const float t = std::clamp((bend - 1.0471976f) / 0.5235988f, 0.0f, 1.0f);
            wrapped *= 1.0f - t * t * (3.0f - 2.0f * t);
        }
        // The roll: -twist/2 clamped to +-45 deg (WrPuppets), or periodic -45 deg sin(twist): the same at 90 deg, 0 at
        // 180 where the clamped one wraps from +45 to -45 (a flip of the forearm).
        const float roll = options.twistRollPeriodic ? -0.78539816f * std::sin(wrapped) : std::clamp(-wrapped * 0.5f, -0.78539816f, 0.78539816f);
        const Vec3 shoulder = pose.modelPos[c.upper];
        const Quat elbowRoll = AxisAngle(handPos - shoulder, roll);
        SetModel(sk, pose, c.upper, shoulder, Normalize(elbowRoll * pose.modelRot[c.upper]));
        const Vec3 elbow = shoulder + Rotate(elbowRoll, pose.modelPos[c.lower] - shoulder);
        const Quat foreRolled = Normalize(elbowRoll * pose.modelRot[c.lower]);
        const Vec3 foreAxis = Rotate(foreRolled, pose.localPos[c.end]);
        SetModel(sk, pose, c.lower, elbow, Normalize(AxisAngle(foreAxis, wrapped * 0.5f) * foreRolled));
        SetModel(sk, pose, c.end, handPos, handModel);
        if (debug) {
            debug->twistDeg[side] = wrapped * 57.29578f;
            debug->elbowRollDeg[side] = roll * 57.29578f;
        }
    }
    sk.Fk(pose);

    // 10. Wrist limit: the hand's rotation against the forearm, beyond the avatar's reference, split into swing (the bend) and
    // twist about the hand's long axis; the swing is capped at options.wristLimitDeg (its direction kept), the twist
    // left to the twist solver. The hand keeps its position; past the limit it stops following the goal's bend.
    for (int side = 0; side < 2; ++side) {
        if (debug) { debug->wristBendDeg[side] = 0; debug->wristClamped[side] = false; }
        if (!options.arms || options.wristLimitDeg <= 0 || !goals[side == 0 ? kGoalLeftHand : kGoalRightHand].active) continue;
        const TwoBone& c = m_armChain[side];
        const Quat fore = pose.modelRot[c.lower];
        const Quat rel = Normalize(Conjugate(fore) * pose.modelRot[c.end] * Conjugate(m_handRefLocal[side]));   // forearm frame
        const Vec3 axis = Normalize(Rotate(m_handRefLocal[side], m_handAlongLocal[side]));   // the hand's long axis at rest
        const float along = Dot({rel.x, rel.y, rel.z}, axis);
        Quat twist = Normalize(Quat{axis.x * along, axis.y * along, axis.z * along, rel.w});
        if (std::fabs(along) < 1e-9f && std::fabs(rel.w) < 1e-9f) twist = {};
        Quat swing = Normalize(rel * Conjugate(twist));
        if (swing.w < 0) swing = {-swing.x, -swing.y, -swing.z, -swing.w};
        const float bend = 2.0f * std::acos(std::clamp(swing.w, -1.0f, 1.0f)), limit = options.wristLimitDeg * 0.01745329f;
        if (debug) debug->wristBendDeg[side] = bend * 57.29578f;
        if (bend <= limit) continue;
        swing = AxisAngle({swing.x, swing.y, swing.z}, limit);
        SetModel(sk, pose, c.end, pose.modelPos[c.end], Normalize(fore * swing * twist * m_handRefLocal[side]));
        if (debug) debug->wristClamped[side] = true;
    }
    sk.Fk(pose);
}

}  // namespace wrpuppets::fbt
