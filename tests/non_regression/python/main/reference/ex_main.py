import seahowl

# options
main_filepath = (
    "../../../../../data/IEA15MW/onshore/main.json"  # change to actual filepath
)
seahowl.set_log_level_global("info")  # log levels: critical, info, debug, warn, trace

# make simulation object
output_folder = "./output"
simulation = seahowl.core.Simulation()
simulation.populate_from_file(main_filepath)
simulation.initialize_from_config()
simulation.outputs.dt_output = 0.0
simulation.outputs.has_gui = True
simulation.outputs.has_vtk = False
simulation.outputs.has_csv = True
simulation.outputs.set_output_folder(output_folder)

# simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
