#pragma once

#include <seahowl/commons/numerics.h>

namespace seahowl {

class Entity {
  public:
    /**
     * @brief Sets position of entity.
     *
     * @param[in] position Position of entity.
     */
    virtual void set_position(Vector3d position) = 0;

    /**
     * @brief Returns position of entity.
     */
    virtual Vector3d get_position() const = 0;

    /**
     * @brief Sets rotation of entity.
     *
     * @param[in] rotation Rotation of entity.
     */
    virtual void set_rotation(Quaternion rotation) = 0;

    /**
     * @brief Returns rotation of entity.
     */
    virtual Quaternion get_rotation() const = 0;
};

class EntityDynamic : public virtual Entity {
  public:
    /**
     * @brief Sets velocity of entity.
     *
     * @param[in] velocity Velocity of entity.
     */
    virtual void set_velocity(Vector3d velocity) = 0;

    /**
     * @brief Returns rotation of entity.
     */
    virtual Vector3d get_velocity() const = 0;

    /**
     * @brief Sets acceleration of entity.
     *
     * @param[in] acceleration Acceleration of entity.
     */
    virtual void set_acceleration(Vector3d acceleration) = 0;

    /**
     * @brief Returns acceleration of entity.
     */
    virtual Vector3d get_acceleration() const = 0;

    /**
     * @brief Sets rotational velocity of entity (local frame).
     *
     * @param[in] rotational_velocity_local of entity.
     */
    virtual void set_rotational_velocity_local(Vector3d rotational_velocity_local) = 0;

    /**
     * @brief Returns rotational velocity of entity (local reference frame).
     */
    virtual Vector3d get_rotational_velocity_local() const = 0;

    /**
     * @brief Sets rotational acceleration of entity (local frame).
     *
     * @param[in] rotational_acceleration_local of entity.
     */
    virtual void set_rotational_acceleration_local(Vector3d rotational_acceleration_local) = 0;

    /**
     * @brief Returns rotational acceleration of entity (local reference frame).
     */
    virtual Vector3d get_rotational_acceleration_local() const = 0;

    /**
     * @brief Sets rotational velocity of entity (global frame).
     *
     * @param[in] rotational_velocity_global of entity.
     */
    virtual void set_rotational_velocity_global(Vector3d rotational_velocity_global) = 0;

    /**
     * @brief Returns rotational velocity of entity (global reference frame).
     */
    virtual Vector3d get_rotational_velocity_global() const = 0;

    /**
     * @brief Sets rotational acceleration of entity (global frame).
     *
     * @param[in] rotational_acceleration_global of entity.
     */
    virtual void set_rotational_acceleration_global(Vector3d rotational_acceleration_global) = 0;

    /**
     * @brief Returns rotational acceleration of entity (global reference frame).
     */
    virtual Vector3d get_rotational_acceleration_global() const = 0;
};

class EntityEigen : public virtual Entity {
  protected:
    /** @brief Position of entity. */
    Vector3d position{0.0, 0.0, 0.0};
    /** @brief Rotation of entity. */
    Quaternion rotation{0.0, 0.0, 0.0, 0.0};

  public:
    virtual void set_position(Vector3d position) override;
    virtual Vector3d get_position() const override;
    virtual void set_rotation(Quaternion rotation) override;
    virtual Quaternion get_rotation() const override;
};

class EntityDynamicEigen : public EntityDynamic, public EntityEigen {
  protected:
    /** @brief Velocity of entity. */
    Vector3d velocity{0.0, 0.0, 0.0};
    /** @brief Acceleration of entity. */
    Vector3d acceleration{0.0, 0.0, 0.0};
    /** @brief Rotational velocity of entity (local). */
    Vector3d rotational_velocity_local{0.0, 0.0, 0.0};
    /** @brief Rotational acceleration of entity (local). */
    Vector3d rotational_acceleration_local{0.0, 0.0, 0.0};
    /** @brief Rotational velocity of entity (global). */
    Vector3d rotational_velocity_global{0.0, 0.0, 0.0};
    /** @brief Rotational acceleration of entity (global). */
    Vector3d rotational_acceleration_global{0.0, 0.0, 0.0};

  public:
    // need to explicitly declare that some functions come from EntityEigen and not EntityDynamic to disable warnings
    virtual void set_position(Vector3d position) override { EntityEigen::set_position(position); };
    virtual Vector3d get_position() const override { return EntityEigen::get_position(); };
    virtual void set_rotation(Quaternion rotation) override { EntityEigen::set_rotation(rotation); };
    virtual Quaternion get_rotation() const override { return EntityEigen::get_rotation(); };

    virtual void set_velocity(Vector3d velocity) override;
    virtual Vector3d get_velocity() const override;
    virtual void set_acceleration(Vector3d acceleration) override;
    virtual Vector3d get_acceleration() const override;
    virtual void set_rotational_velocity_local(Vector3d rotational_velocity_local) override;
    virtual Vector3d get_rotational_velocity_local() const override;
    virtual void set_rotational_acceleration_local(Vector3d rotational_acceleration_local) override;
    virtual Vector3d get_rotational_acceleration_local() const override;
    virtual void set_rotational_velocity_global(Vector3d rotational_velocity_global) override;
    virtual Vector3d get_rotational_velocity_global() const override;
    virtual void set_rotational_acceleration_global(Vector3d rotational_acceleration_global) override;
    virtual Vector3d get_rotational_acceleration_global() const override;
};

}  // namespace seahowl
