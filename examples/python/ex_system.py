import seahowl

# options
filepath = "../../data/IEA15MW/onshore/main.json"  # change to actual filepath
dt = 0.05
dt_output = 1.0
t_end = 200.0
t_output_next = 0.0

# get system
system_elasto = seahowl.elasto.SystemElastoChrono()
system_aero = seahowl.aero.SystemAero()
system_core = seahowl.core.System(system_elasto, system_aero)
seahowl.io.populate_system_from_json(filepath, system_core)

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

    t_sim = system_core.get_time()
    if t_sim >= t_output_next - 1e-6:
        turbine = system_core.turbines[0]
        print(
            "time: {time:.3f}, step: {step}, rpm: {rpm:.3f}, pitch: {pitch:.3f}".format(
                time=t_sim,
                step=step,
                rpm=turbine.rna.elasto.get_rpm(),
                pitch=turbine.rna.elasto.rotor.pitch_collective,
            )
        )
        t_output_next += dt_output
