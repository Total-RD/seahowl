#pragma once

#include <Eigen/Dense>

namespace seahowl {

const double PI = 3.14159265358979323846;

using Vector3d = Eigen::Vector3d;
using Vector2d = Eigen::Vector2d;

using Quaternion = Eigen::Quaterniond;

using AngleAxisd = Eigen::AngleAxisd;

class Entity {
  public:
    /**
     * @brief Sets position of entity.
     *
     * @param[in] Position of entity.
     */
    virtual void set_position(Vector3d position) = 0;

    /**
     * @brief Returns position of entity.
     */
    virtual Vector3d get_position() const = 0;

    /**
     * @brief Sets rotation of entity.
     *
     * @param[in] Rotation of entity.
     */
    virtual void set_rotation(Quaternion rotation) = 0;

    /**
     * @brief Returns rotation of entity.
     */
    virtual Quaternion get_rotation() const = 0;
};

class EntityDynamic : public Entity {
  public:
    /**
     * @brief Returns rotation of entity.
     */
    virtual Vector3d get_velocity() const = 0;

    /**
     * @brief Returns acceleration of entity.
     */
    virtual Vector3d get_acceleration() const = 0;

    /**
     * @brief Returns direction of entity.
     */
    virtual Vector3d get_direction() const = 0;

    /**
     * @brief Returns rotational velocity of entity (local reference frame).
     */
    virtual Vector3d get_rotational_velocity_local() const = 0;

    /**
     * @brief Returns rotational acceleration of entity (local reference frame).
     */
    virtual Vector3d get_rotational_acceleration_local() const = 0;

    /**
     * @brief Returns rotational velocity of entity (global reference frame).
     */
    virtual Vector3d get_rotational_velocity_global() const = 0;

    /**
     * @brief Returns rotational acceleration of entity (global reference frame).
     */
    virtual Vector3d get_rotational_acceleration_global() const = 0;
};

}  // namespace seahowl
