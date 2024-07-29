#pragma once

// Disable inherits via dominance warning when there is multiple inheritance
#pragma warning(disable : 4250)

#include "seahowl/commons/numerics.h"

namespace seahowl {

/**
 * @brief Base entity, with position and rotation.
 */
class Entity {
  public:
    /**
     * @brief Sets position of entity.
     *
     * @param[in] position Position of entity.
     */
    virtual void set_position(const Vector3d& position) = 0;

    /**
     * @brief Returns position of entity.
     */
    virtual Vector3d get_position() const = 0;

    /**
     * @brief Sets rotation of entity.
     *
     * @param[in] rotation Rotation of entity.
     */
    virtual void set_rotation(const Quaternion& rotation) = 0;

    /**
     * @brief Returns rotation of entity.
     */
    virtual Quaternion get_rotation() const = 0;

    /**
     * @brief Returns rotation matrix of entity.
     */
    Eigen::Matrix<double, 3, 3> get_rotation_matrix() const { return get_rotation().toRotationMatrix(); };

    /**
     * @brief Sets rotation matrix of entity.
     *
     * @param[in] rotation Rotation matrix of entity.
     */
    void set_rotation_matrix(Eigen::Matrix<double, 3, 3> rotation) { set_rotation(Quaternion(rotation)); };

    /**
     * @brief Rotates the entity.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    void rotate(double angle, const Vector3d& axis);

    /**
     * @brief Translates the entity.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    void translate(const Vector3d& translation_vector);
};

/**
 * @brief Base dynamic entity, with position, rotation, velocity, acceleration.
 */
class EntityDynamic : public virtual Entity {
  public:
    /**
     * @brief Sets velocity of entity.
     *
     * @param[in] velocity Velocity of entity.
     */
    virtual void set_velocity(const Vector3d& velocity) = 0;

    /**
     * @brief Returns rotation of entity.
     */
    virtual Vector3d get_velocity() const = 0;

    /**
     * @brief Sets acceleration of entity.
     *
     * @param[in] acceleration Acceleration of entity.
     */
    virtual void set_acceleration(const Vector3d& acceleration) = 0;

    /**
     * @brief Returns acceleration of entity.
     */
    virtual Vector3d get_acceleration() const = 0;

    /**
     * @brief Sets rotational velocity of entity (global reference frame).
     *
     * @param[in] rotational_velocity_global of entity.
     */
    virtual void set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local = true) = 0;

    /**
     * @brief Returns rotational velocity of entity (global reference frame).
     */
    virtual Vector3d get_rotational_velocity(bool is_local = true) const = 0;

    /**
     * @brief Sets rotational acceleration of entity (global reference frame).
     *
     * @param[in] rotational_acceleration_global of entity.
     */
    virtual void set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local = true) = 0;

    /**
     * @brief Returns rotational acceleration of entity (global reference frame).
     */
    virtual Vector3d get_rotational_acceleration(bool is_local = true) const = 0;
};

/**
 * @brief Entity implemented with Eigen types.
 */
class EntityEigen : public virtual Entity {
  protected:
    /** @brief Position of entity. */
    Vector3d position{0.0, 0.0, 0.0};
    /** @brief Rotation of entity. */
    Quaternion rotation{0.0, 0.0, 0.0, 0.0};

  public:
    virtual void set_position(const Vector3d& position) override;
    virtual Vector3d get_position() const override;
    virtual void set_rotation(const Quaternion& rotation) override;
    virtual Quaternion get_rotation() const override;
};

/**
 * @brief Dynamic entity implemented with Eigen types.
 */
class EntityDynamicEigen : public EntityDynamic, public EntityEigen {
  protected:
    /** @brief Velocity of entity. */
    Vector3d velocity{0.0, 0.0, 0.0};
    /** @brief Acceleration of entity. */
    Vector3d acceleration{0.0, 0.0, 0.0};
    /** @brief Rotational velocity of entity (in global reference frame). */
    Vector3d rotational_velocity{0.0, 0.0, 0.0};
    /** @brief Rotational acceleration of entity (in global reference frame). */
    Vector3d rotational_acceleration{0.0, 0.0, 0.0};

  public:
    virtual void set_velocity(const Vector3d& velocity) override;
    virtual Vector3d get_velocity() const override;
    virtual void set_acceleration(const Vector3d& acceleration) override;
    virtual Vector3d get_acceleration() const override;
    virtual void set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local = true) override;
    virtual Vector3d get_rotational_velocity(bool is_local = true) const override;
    virtual void set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local = true) override;
    virtual Vector3d get_rotational_acceleration(bool is_local = true) const override;
};

}  // namespace seahowl
