import pyseahowl

# options
filepath = "../../data/IEA15MW/main.json"  # change to actual filepath
dt = 0.05
dt_output = 1.0
t_end = 200.0
t_output_next = 0.0

# get system
system_elasto = pyseahowl.elasto.SystemElastoChrono()
system_aero = pyseahowl.aero.SystemAero()
system_core = pyseahowl.core.System()
system_core.system_elasto = system_elasto
system_core.system_aero = system_aero
pyseahowl.populate_system_from_json(filepath, system_core)

# fix tower bottom nodes
for turbine in system_core.turbines:
    turbine.tower.elasto.nodes[0].set_fixed(True)
# statics
system_elasto.do_statics(True, 10)
# initialize system
system_core.initialize(system_elasto.get_time(), dt)

# simulation loop
step = 0
t_sim = 0
while t_sim < t_end:
    system_core.prestep(t_sim, dt)
    system_core.step(dt)
    step += 1
    system_core.poststep(t_sim, dt)

    t_sim = system_core.system_elasto.get_time()
    if t_sim >= t_output_next - 1e-6:
        turbine = system_core.turbines[0]
        print(
            "time: {time}, step: {step}, rpm: {rpm}, pitch: {pitch}".format(
                time=t_sim,
                step=step,
                rpm=turbine.rotor.elasto.get_rpm(),
                pitch=turbine.rotor.elasto.pitch_collective,
            )
        )
        t_output_next += dt_output
