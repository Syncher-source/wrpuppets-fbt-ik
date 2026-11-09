#include "Fbt/FbtSkeleton.hpp"
#include <utility>
#include <cstdio>
#include <cstring>

namespace wrpuppets::fbt {

bool Skeleton::Load(const char* path, std::string* error) {
    m_bones.clear();
    m_roles.clear();
    FILE* f = nullptr;
    if (fopen_s(&f, path, "r") != 0 || !f) {
        if (error) *error = std::string("cannot open ") + path;
        return false;
    }
    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        char name[256]{};
        int index = 0, parent = 0;
        if (std::strncmp(line, "role ", 5) == 0) {
            if (sscanf_s(line, "role %255s %d", name, static_cast<unsigned>(sizeof(name)), &index) == 2)
                m_roles.emplace_back(name, index);
            continue;
        }
        if (std::strncmp(line, "bone ", 5) != 0) continue;
        Bone b;
        if (sscanf_s(line, "bone %d %d %255s %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &index, &parent, name,
                     static_cast<unsigned>(sizeof(name)), &b.refLocalPos.x, &b.refLocalPos.y, &b.refLocalPos.z,
                     &b.refLocalRot.x, &b.refLocalRot.y, &b.refLocalRot.z, &b.refLocalRot.w, &b.refModelPos.x,
                     &b.refModelPos.y, &b.refModelPos.z, &b.refModelRot.x, &b.refModelRot.y, &b.refModelRot.z,
                     &b.refModelRot.w) != 17)
            continue;
        if (index < 0 || index > 4096) continue;
        // Bones without a valid reference are left out of the file: keep the indices, fill the gap as roots.
        if (index >= Count()) m_bones.resize(index + 1);
        b.name = name;
        b.parent = parent < index ? parent : -1;
        m_bones[index] = b;
    }
    fclose(f);
    if (m_bones.empty()) {
        if (error) *error = std::string("no bones in ") + path;
        return false;
    }
    return true;
}

bool Skeleton::Build(std::vector<Bone> bones, std::vector<std::pair<std::string, int>> roles, std::string* error) {
    m_bones = std::move(bones);
    m_roles = std::move(roles);
    for (int i = 0; i < Count(); ++i)
        if (m_bones[i].parent >= i) m_bones[i].parent = -1;   // as Load: a parent must come before its child
    if (m_bones.empty()) {
        if (error) *error = "no bones";
        return false;
    }
    return true;
}

int Skeleton::Role(const char* name) const {
    for (const auto& r : m_roles)
        if (r.first == name) return r.second >= 0 && r.second < Count() ? r.second : -1;
    return -1;
}

Vec3 Skeleton::EyePoint() const {
    const int le = Role("left_eye"), re = Role("right_eye");
    if (le >= 0 && re >= 0) return (m_bones[le].refModelPos + m_bones[re].refModelPos) * 0.5f;
    const int head = Role("head");
    if (head < 0) return {};
    return m_bones[head].refModelPos + Vec3{0, kEyeFromHeadForward, kEyeFromHeadUp};
}

int Skeleton::Find(const char* boneName) const {
    for (int i = 0; i < Count(); ++i)
        if (m_bones[i].name == boneName) return i;
    return -1;
}

Pose Skeleton::ReferencePose() const {
    Pose p;
    p.localPos.resize(Count());
    p.localRot.resize(Count());
    for (int i = 0; i < Count(); ++i) {
        p.localPos[i] = m_bones[i].refLocalPos;
        p.localRot[i] = m_bones[i].refLocalRot;
    }
    Fk(p);
    return p;
}

void Skeleton::Fk(Pose& pose) const {
    pose.modelPos.resize(Count());
    pose.modelRot.resize(Count());
    for (int i = 0; i < Count(); ++i) {
        const int parent = m_bones[i].parent;
        if (parent < 0) {
            pose.modelPos[i] = pose.localPos[i];
            pose.modelRot[i] = pose.localRot[i];
        } else {
            pose.modelPos[i] = pose.modelPos[parent] + Rotate(pose.modelRot[parent], pose.localPos[i]);
            pose.modelRot[i] = Normalize(pose.modelRot[parent] * pose.localRot[i]);
        }
    }
}

}  // namespace wrpuppets::fbt
