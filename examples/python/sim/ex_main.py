import seahowl
import pathlib

# change filepaths accordingly
# we set them relative to this script file for it to run from anywhere
thispath = pathlib.Path(__file__).parent.resolve()  # path of this file

# options
main_filepath = (
    thispath / "../../../data/IEA15MW/main_onshore.json"
)  # change to actual filepath
seahowl.set_log_level_global("info")  # log levels: critical, info, debug, warn, trace

output_folder = "./output"
# make simulation object
simulation = seahowl.core.Simulation()
simulation.populate_from_file(str(main_filepath))
simulation.outputs.set_output_folder(output_folder)
simulation.duration = 100.0
simulation.initialize_from_config()
# simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
