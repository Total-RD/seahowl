import seahowl

# options
main_filepath = "../../data/IEA15MW/onshore/main.json"  # change to actual filepath
seahowl.set_log_level_global("info")  # log levels: critical, info, debug, warn, trace

output_folder = "./output"
# make simulation object
simulation = seahowl.core.Simulation()
simulation.populate_from_file(main_filepath)
simulation.outputs.set_output_folder(output_folder)
simulation.duration = 100.0
simulation.initialize_from_config()
# simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
