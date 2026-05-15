import seahowl
import pathlib

# change filepaths accordingly
# we set them relative to this script file for it to run from anywhere
thispath = pathlib.Path(__file__).parent.resolve()  # path of this file

turbine_filepath = thispath / "../../data/IEA15MW/onshore/turbine/turbine.json"
environment_filepath = thispath / "../../data/IEA15MW/env/env_inflowwind.json"

#  ____       _
# / ___|  ___| |_ _   _ _ __
# \___ \ / _ \ __| | | | '_ \
#  ___) |  __/ |_| |_| | |_) |
# |____/ \___|\__|\__,_| .__/
#                      |_|
#
# model setup

# log levels: critical, error, info, debug, warn, trace
seahowl.set_log_level_global("info")

# make simulation object
simulation = seahowl.core.Simulation()
# time options
simulation.dt = 0.025
simulation.duration = 100.0
# output options
simulation.outputs.dt_output = 0.0  # a value of 0 will output all time steps
simulation.outputs.has_gui = True  # if True, will show in-situ visualization
simulation.outputs.has_vtk = (
    False  # if True, will output VTK for blades, tower, monpile
)
simulation.outputs.has_csv = (
    True  # if True, will output essential turbine info as output
)
simulation.outputs.set_output_folder(
    "./output"
)  # folder where all outputs will be stored

# add turbine to system (from JSON file)
system_core = simulation.system_core
seahowl.io.add_turbine_to_system_from_file(str(turbine_filepath), system_core)
turbine = system_core.turbines[0]

# environmental conditions (from JSON file)
system_core.env_model = seahowl.io.get_environmental_model_from_file(
    str(environment_filepath)
)

# make custom CSV
# add any lambda functions that return doubles or array of doubles
mycsv = simulation.outputs.create_new_csv("my_outputs.csv")  # name it as you like
mycsv.add_function("time [s]", lambda: system_core.get_time())  # add time column
mycsv.add_function(
    "blade1 tip position [m]",
    lambda: turbine.elasto.rna.rotor.blades[0].nodes[-1].get_position(),
)


#  ___       _ _   _       _ _          _   _
# |_ _|_ __ (_) |_(_) __ _| (_)______ _| |_(_) ___  _ __
#  | || '_ \| | __| |/ _` | | |_  / _` | __| |/ _ \| '_ \
#  | || | | | | |_| | (_| | | |/ / (_| | |_| | (_) | | | |
# |___|_| |_|_|\__|_|\__,_|_|_/___\__,_|\__|_|\___/|_| |_|
#
# initialization of model

# initialize simulation
simulation.initialize()

# statics
system_core.elasto.do_statics(True, 10)


#  ____  _                 _       _   _
# / ___|(_)_ __ ___  _   _| | __ _| |_(_) ___  _ __
# \___ \| | '_ ` _ \| | | | |/ _` | __| |/ _ \| '_ \
#  ___) | | | | | | | |_| | | (_| | |_| | (_) | | | |
# |____/|_|_| |_| |_|\__,_|_|\__,_|\__|_|\___/|_| |_|
#
# simulation loop

# run simulation
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
