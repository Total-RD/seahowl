import seahowl

# options
turbine_filepath = (
    "../../../../../data/IEA15MW/onshore/turbine.json"  # change to actual filepath
)
output_folder = "./output"
seahowl.set_log_level_global("info")  # log levels: critical, info, debug, warn, trace

# make simulation object
simulation = seahowl.core.Simulation()
simulation.dt = 0.05
simulation.duration = 20.0
simulation.outputs.dt_output = 0.0
simulation.outputs.has_gui = True
simulation.outputs.has_vtk = False
simulation.outputs.has_csv = True
simulation.outputs.set_output_folder(output_folder)

# add turbine to system
system_core = simulation.system_core
seahowl.io.add_turbine_to_system_from_file(turbine_filepath, system_core)
turbine = system_core.turbines[0]

# fix tower bottom nodes and statics step
system_elasto = system_core.elasto
system_elasto.do_statics(True, 10)

# add fluid model
wind_model = seahowl.env.ConstantWind()
system_core.env_model = seahowl.env.EnvModel()
system_core.env_model.add_model(wind_model)
wind_model.set_wind_velocity(
    [12.0, 0.0, 0.0],
)
wind_model.shear_coefficient = 0.12
hub_pos = turbine.elasto.rna.rotor.body_hub.get_position()
wind_model.reference_height = hub_pos[2]  # put reference height at hub height
wind_model.air_density = 1.225

# initialize simulation
simulation.initialize()

# simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
