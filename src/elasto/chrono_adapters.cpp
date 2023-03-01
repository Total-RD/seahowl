#include <seahowl/elasto/chrono_adapters.h>

namespace seahowl {
namespace elasto {

chrono::ChVector<double> vec2ch(Vector3d vector_in) {
    return chrono::ChVector<double>(vector_in[0], vector_in[1], vector_in[2]);
}
Vector3d ch2vec(chrono::ChVector<double> vector_in) {
    return Vector3d(vector_in[0], vector_in[1], vector_in[2]);
}

chrono::ChQuaternion<double> quat2ch(Quaternion quaternion_in) {
    return chrono::ChQuaternion<double>(quaternion_in.w(), quaternion_in.x(), quaternion_in.y(), quaternion_in.z());
}

Quaternion ch2quat(chrono::ChQuaternion<double> quaternion_in) {
    return Quaternion(quaternion_in[0], quaternion_in[1], quaternion_in[2], quaternion_in[3]);
}

}  // namespace elasto
}  // namespace seahowl
