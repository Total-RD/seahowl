import seahowl
import numpy as np

seahowl.set_log_level_global("debug")

simulation = seahowl.core.Simulation()
simulation.dt = 0.01

mean_water_level = 0.0
water_depth = 200.0

system_core = simulation.system_core

fluid_model = seahowl.env.StillWater()
system_core.fluid_model = fluid_model
fluid_model.density = 1025.0
fluid_model.mean_water_level = mean_water_level
fluid_model.water_depth = water_depth

soil_model = seahowl.env.LinearSoilModel()
system_core.soil_model = soil_model
soil_model.soil_position = mean_water_level - water_depth
soil_model.soil_normal = np.array([0.0, 0.0, 1.0])
soil_model.stiffness_normal = 1e6
soil_model.stiffness_shear = 0.0


# make bodies to attach mooring line (fairlead + anchor)

# fairlead
fairlead_body = seahowl.elasto.BodyElastoChrono()
system_core.elasto.add(fairlead_body)
fairlead_body.set_position(np.array([0.0, 0.0, mean_water_level - 14.0]))
fairlead_body.set_fixed(True)

# anchor
anchor_body = seahowl.elasto.BodyElastoChrono()
system_core.elasto.add(anchor_body)
anchor_body.set_position(np.array([0.0, 58.0 - 837.60, mean_water_level - water_depth]))
anchor_body.set_fixed(True)

# create mooring line

# elasto
mooring_elasto = seahowl.elasto.MooringElastoFEA(fairlead_body, anchor_body)
system_core.elasto.add(mooring_elasto)
# hydro
mooring_hydro = seahowl.hydro.MooringHydro()
system_core.aero.add(mooring_hydro)
# core
mooring = seahowl.core.Mooring(mooring_elasto, mooring_hydro)
system_core.add(mooring)

# set mooring properties
nelements = 40
discretization_fractions = np.linspace(0, 1, 40)
# elasto
mooring.elasto.stiffness_axial = 3270e6
mooring.elasto.stiffness_bending = 0.0
mooring.elasto.density_linear = 685.0
mooring.elasto.discretization_fractions = discretization_fractions
# hydro
mooring.hydro.coefficients.drag_normal = 2.0
mooring.hydro.coefficients.drag_axial = 1.15
mooring.hydro.coefficients.added_mass_normal = 1.0
mooring.hydro.coefficients.added_mass_axial = 1.0
mooring.hydro.discretization_fractions = discretization_fractions
# core (elasto + hydro)
mooring.set_length(850.0)
mooring.set_diameter(0.333)


system_core.build()
simulation.initialize()
system_core.run_presimulation(100.0, 0.01, True, True)

for ii in range(10000):
    simulation.step()
    print(
        "Tension at fairlead: {}.".format(
            np.linalg.norm(mooring.elasto.fairlead_link.get_reaction_force())
        )
    )
