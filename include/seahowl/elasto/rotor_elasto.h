#pragma once

#include <seahowl/elasto/elasto.h>

#include <vector>
#include <memory>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLinkRevolute.h>

namespace seahowl {
namespace elasto {
class BladeElasto;
class TowerElasto;  ///@todo move out of rotor
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace elasto {

/**@brief Hub properties


\image html NREL_ad_driver_geom.png "source image: NREL/Openfast" width=300cm
*/
struct HubProperties {
    double center_of_mass = 0.0;  ///< COG Center Of Gravity
    double mass = 0.0;            ///< Total mass of the hub
    double inertia = 0.0;         ///< Coefficient of inertia
    double overhang = 0.0;        ///< Overhang
    double radius = 0.0;          ///< Radius of the hub (from hub apex to hub edge in rotor plane)
};

/**@brief Nacelle properties */
struct NacelleProperties {
    chrono::ChVector<double> center_of_mass{0.0, 0.0, 0.0};  ///< COG Center Of Gravity @todo Initialize in constructor
    double mass = 0.0;                                       ///< Mass of the nacelle (without bearing)
    double inertia = 0.0;                                    ///< Inertia @todo include 3x3 inertia
    double yaw_bearing_mass = 0.0;                           ///< Bearing mass
};

/**@brief Shaft properties */
struct ShaftProperties {
    double tilt = 0.0;                    ///< Tilt angle (radians)
    double distance_from_towertop = 0.0;  ///< Distance from Tower top reference point
};

/**@brief Rotor properties

Implemented as collection of rigid bodies + blades
*/
class RotorElasto : public seahowl::elasto::ElastoComponent {
  public:
    std::vector<double> blade_precones;  ///< Blade precones (radians)
    double pitch_collective;             ///< Collective pitch (for all blades)

    // bodies
    ///@{
    std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades;  ///<@brief Finite Element Blades
    std::shared_ptr<chrono::ChBody> body_hub;                           ///< Hub rigid body
    std::shared_ptr<chrono::ChBody> body_shaft;                         ///< Shaft rigid body
    std::shared_ptr<chrono::ChBody> body_nacelle;                       ///< Nacelle rigid body
    std::shared_ptr<chrono::ChBody> body_yaw_bearing;                   ///< Yaw bearing rigid body
    ///@}

    // links
    ///@{
    std::vector<std::shared_ptr<chrono::ChLinkMateFix>> links_blades;
    std::shared_ptr<chrono::ChLinkRevolute> link_shaft_hub;
    std::shared_ptr<chrono::ChLinkMateFix> link_shaft_nacelle;
    std::shared_ptr<chrono::ChLinkMateFix> link_shaft_yaw_bearing;
    std::shared_ptr<chrono::ChLinkMateFix> link_towertop_yaw_bearing;  ///< External link with TowerElasto
    ///@}

    // properties
    ///@{
    ShaftProperties shaft;      ///< Shaft properties
    NacelleProperties nacelle;  ///< Nacelle properties
    HubProperties hub;          ///< Hub properties
    ///@}

    RotorElasto();
    ~RotorElasto();

    void assemble(chrono::ChSystemSMC& system);
    void build(std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades);
    void link_tower(TowerElasto& tower, chrono::ChSystemSMC& system);

    ///@{
    void rotate(double angle, chrono::ChVector<double> axis) const override;     ///< @see ElastoComponent::rotate
    void translate(chrono::ChVector<double> translation_vector) const override;  ///< @see ElastoComponent::translate
    double get_mass() const override;                                            ///< @see ElastoComponent::get_mass
    ///@}

    ///@{
    void apply_collective_pitch_increment(double pitch_increment);  ///< Applies pitch on all blades
    double get_rpm() const;                                         ///< Rotation speed @todo in general class Rotor
    double get_axial_torque() const;  ///< Reaction torque on hub @todo in general class Rotor
    double get_azimuth() const;       ///< Azimuth @todo in general class Rotor
    double get_axial_thrust() const;  ///< Reaction thrust on hub @todo in general class Rotor
    ///@}
};

}  // namespace elasto
}  // namespace seahowl
