#pragma once

#include <chrono/core/ChVector.h>
#include <chrono/core/ChVector2.h>
#include <chrono/core/ChMatrix.h>
#include <chrono/core/ChQuaternion.h>
#include <chrono/physics/ChBody.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>

#include <vector>
#include <memory>

namespace seahowl {

// class Vector3d : public chrono::ChVector<double> {
//  public:
//    Vector3d(double x, double y, double z) : ChVector<double>(x, y, z){};
//    Vector3d normalized() { this->GetNormalized(); };
//};

const double PI = (double)chrono::CH_C_PI;

class Vector3d : public chrono::ChVector<double> {
  public:
    Vector3d() : chrono::ChVector<double>(){};
    Vector3d(double x, double y, double z) : chrono::ChVector<double>(x, y, z){};
    Vector3d(const chrono::ChVector<double>& chvector) : chrono::ChVector<double>(chvector){};

    Vector3d cross(const Vector3d& right) { return Vector3d(*this % right); };

    double dot(const Vector3d& right) { return *this ^ right; };

    Vector3d normalized() { return Vector3d(this->GetNormalized()); }

    // operators
    inline Vector3d Vector3d::operator+() const { return *this; }
    inline Vector3d Vector3d::operator-() const {
        chrono::ChVector<double> v1 = *this;
        return Vector3d(-v1);
    }
    inline Vector3d Vector3d::operator+(const Vector3d& other) const {
        chrono::ChVector<double> v1 = *this;
        chrono::ChVector<double> v2 = other;
        return Vector3d(v1 + v2);
    }
    inline Vector3d Vector3d::operator-(const Vector3d& other) const {
        chrono::ChVector<double> v1 = *this;
        chrono::ChVector<double> v2 = other;
        return Vector3d(v1 - v2);
    }
    inline Vector3d Vector3d::operator*(const Vector3d& other) const {
        chrono::ChVector<double> v1 = *this;
        chrono::ChVector<double> v2 = other;
        return Vector3d(v1 * v2);
    }
    inline Vector3d Vector3d::operator*(double s) const {
        chrono::ChVector<double> v1 = *this;
        return Vector3d(v1 * s);
    }
    inline Vector3d Vector3d::operator/(double s) const {
        chrono::ChVector<double> v1 = *this;
        return Vector3d(v1 / s);
    }
    inline Vector3d Vector3d::operator+(double s) const {
        chrono::ChVector<double> v1 = *this;
        return Vector3d(v1 + s);
    }
    inline Vector3d Vector3d::operator-(double s) const {
        chrono::ChVector<double> v1 = *this;
        return Vector3d(v1 - s);
    }
};
// using Vector3d = chrono::ChVector<double>;
using Vector2d = chrono::ChVector2<double>;

class Quaternion : public chrono::ChQuaternion<double> {
  public:
    Quaternion() : chrono::ChQuaternion<double>(){};
    Quaternion(double e0, double e1, double e2, double e3) : chrono::ChQuaternion<double>(e0, e1, e2, e3){};
    Quaternion(const chrono::ChQuaternion<double>& chquaternion) : chrono::ChQuaternion<double>(chquaternion){};

    Quaternion normalized() const { return Quaternion(this->GetNormalized()); };
    Quaternion inverse() const { return Quaternion(this->GetInverse()); };

    inline Quaternion Quaternion::operator*(const Quaternion& other) const {
        chrono::ChQuaternion<double> q;
        q.Cross(*this, other);
        return Quaternion(q);
    }
    inline Vector3d Quaternion::operator*(const Vector3d& other) const { return this->Rotate(other); }
};

// using Vector3d = Eigen::Vector3d;
// using Vector2d = Eigen::Vector2d;
}  // namespace seahowl
