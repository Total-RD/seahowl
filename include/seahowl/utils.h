#pragma once

#include <chrono/core/ChQuaternion.h>

#include <vector>
#include <memory>

namespace seahowl {

const double PI = (double)chrono::CH_C_PI;

using Vector3d = Eigen::Vector3d;
using Vector2d = Eigen::Vector2d;

using Quaternion = Eigen::Quaterniond;

using AngleAxisd = Eigen::AngleAxisd;

// class Vector3d : public chrono::ChVector<double> {
//   public:
//     Vector3d() : chrono::ChVector<double>(){};
//     Vector3d(double x, double y, double z) : chrono::ChVector<double>(x, y, z){};
//     Vector3d(const chrono::ChVector<double>& chvector) : chrono::ChVector<double>(chvector){};
//
//     Vector3d cross(const Vector3d& right) { return Vector3d(*this % right); };
//
//     double dot(const Vector3d& right) { return *this ^ right; };
//
//     Vector3d cwiseProduct(const Vector3d& right) { return *this * right; };
//
//     Vector3d normalized() { return Vector3d(this->GetNormalized()); }
//
//     double norm() { return Vector3d(this->Length()); }
//
//     // operators
//     inline Vector3d Vector3d::operator+() const { return *this; }
//     inline Vector3d Vector3d::operator-() const {
//         chrono::ChVector<double> v1 = *this;
//         return Vector3d(-v1);
//     }
//     inline Vector3d Vector3d::operator+(const Vector3d& other) const {
//         chrono::ChVector<double> v1 = *this;
//         chrono::ChVector<double> v2 = other;
//         return Vector3d(v1 + v2);
//     }
//     inline Vector3d Vector3d::operator-(const Vector3d& other) const {
//         chrono::ChVector<double> v1 = *this;
//         chrono::ChVector<double> v2 = other;
//         return Vector3d(v1 - v2);
//     }
//     inline Vector3d Vector3d::operator*(const Vector3d& other) const {
//         chrono::ChVector<double> v1 = *this;
//         chrono::ChVector<double> v2 = other;
//         return Vector3d(v1 * v2);
//     }
//     inline Vector3d Vector3d::operator*(double s) const {
//         chrono::ChVector<double> v1 = *this;
//         return Vector3d(v1 * s);
//     }
//     inline Vector3d Vector3d::operator/(double s) const {
//         chrono::ChVector<double> v1 = *this;
//         return Vector3d(v1 / s);
//     }
//     inline Vector3d Vector3d::operator+(double s) const {
//         chrono::ChVector<double> v1 = *this;
//         return Vector3d(v1 + s);
//     }
//     inline Vector3d Vector3d::operator-(double s) const {
//         chrono::ChVector<double> v1 = *this;
//         return Vector3d(v1 - s);
//     }
// };
//
// class Quaternion : public chrono::ChQuaternion<double> {
//  public:
//    Quaternion() : chrono::ChQuaternion<double>(){};
//    Quaternion(double e0, double e1, double e2, double e3) : chrono::ChQuaternion<double>(e0, e1, e2, e3){};
//    Quaternion(const chrono::ChQuaternion<double>& chquaternion) : chrono::ChQuaternion<double>(chquaternion){};
//
//    Quaternion normalized() const { return Quaternion(this->GetNormalized()); };
//    Quaternion inverse() const { return Quaternion(this->GetInverse()); };
//    Vector3d vec() const {
//        auto vec = this->GetVector();
//        return Vector3d(vec[0], vec[1], vec[2]);
//    };
//
//    inline Quaternion Quaternion::operator*(const Quaternion& other) const {
//        chrono::ChQuaternion<double> q;
//        q.Cross(*this, other);
//        return Quaternion(q);
//    }
//    inline Vector3d Quaternion::operator*(const Vector3d& other) const {
//        auto vec = this->Rotate(chrono::ChVector<double>(other[0], other[1], other[2]));
//        return Vector3d(vec[0], vec[1], vec[2]);
//    }
//
//    double w() { return this->e0(); };
//    double x() { return this->e1(); };
//    double y() { return this->e2(); };
//    double z() { return this->e3(); };
//};
//
// Quaternion AngleAxisd(double angle, const Vector3d& axis) {
//    chrono::ChQuaternion<double> chquat;
//    chquat.Q_from_AngAxis(angle, chrono::ChVector<double>(axis[0], axis[1], axis[2]);
//    return Quaternion(chquat[0], chquat[1], chquat[2], chquat[3]);
//};

}  // namespace seahowl
