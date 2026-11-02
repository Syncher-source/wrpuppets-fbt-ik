#pragma once

// The avatar's skeleton for the full-body IK, independent of any engine: bones (name, parent, reference local and model
// pose) and the roles the solvers need. Loaded from a rig file (README.md, "The rig file"), which a game's adapter
// writes from its skeleton and offline tools load too.
#include "Fbt/FbtMath.hpp"
#include <string>
#include <vector>

namespace wrpuppets::fbt {

struct Bone {
    std::string name;
    int parent = -1;
    Vec3 refLocalPos, refModelPos;
    Quat refLocalRot, refModelRot;
};

// A pose: one local transform per bone (parent-relative), and the model-space transforms Fk() derives from it.
struct Pose {
    std::vector<Vec3> localPos, modelPos;
    std::vector<Quat> localRot, modelRot;
};

class Skeleton {
public:
    bool Load(const char* path, std::string* error = nullptr);
    int Count() const { return static_cast<int>(m_bones.size()); }
    const Bone& BoneAt(int i) const { return m_bones[i]; }
    // A role from the file ("hips", "head", "left_foot", "spine0", ...); -1 when missing.
    int Role(const char* name) const;
    // The eyes' midpoint in the reference pose, model space: the left_eye and right_eye roles when the rig has them,
    // else the head joint plus kEyeFromHead (an adult's eyes sit about 8-9 cm forward and 5-6 cm up of it). (0,0,0)
    // without a head.
    Vec3 EyePoint() const;
    static constexpr float kEyeFromHeadForward = 0.085f, kEyeFromHeadUp = 0.055f;
    int Find(const char* boneName) const;
    // The reference pose (the T-pose), with its model transforms.
    Pose ReferencePose() const;
    // Model transforms from the local ones; parents always come before their children in the file.
    void Fk(Pose& pose) const;

private:
    std::vector<Bone> m_bones;
    std::vector<std::pair<std::string, int>> m_roles;
};

}  // namespace wrpuppets::fbt
