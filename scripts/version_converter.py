#!/usr/bin/env python3

import pathlib
import argparse
import json
import jsbeautifier
import pandas as pd

options = jsbeautifier.default_options
options.indent_size = 2


def save_json(json_dict, path):
    mydirectory = pathlib.Path(path).parent
    mydirectory.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write(jsbeautifier.beautify(json.dumps(json_dict), options))


VERSIONS_IN = [0.8]
VERSIONS_OUT = [0.9]


def convert_version(main_filepath, version_in=0.8, version_out=0.9):
    if version_in not in VERSIONS_IN:
        raise RuntimeError(f"Input version '{version_in}' not found in {VERSIONS_IN}.")
    if version_out not in VERSIONS_OUT:
        raise RuntimeError(
            f"Output version '{version_out}' not found in {VERSIONS_OUT}."
        )

    main_filepath = pathlib.Path(main_filepath)
    with open(main_filepath) as file_main:
        json_main = json.load(file_main)
        turbine_filepath = main_filepath.parent / json_main["turbines"][0]["file"]
        with open(turbine_filepath) as file_turbine:
            turbine_json = json.load(file_turbine)

            # turbine
            if version_in <= 0.8 and version_out >= 0.9:
                turbine_json["rna"]["yaw_actuator_dynamics"] = True
                turbine_json["rotor"]["pitch_actuator_dynamics"] = True
                if turbine_json["rotor"]["type"] == "fea":
                    if "options" in turbine_json["rotor"]:
                        if "fpm" in turbine_json["rotor"]["options"]:
                            if turbine_json["rotor"]["options"]["fpm"] is True:
                                turbine_json["rotor"]["type"] = "fpm"
                            del turbine_json["rotor"]["options"]["fpm"]
                        if not (turbine_json["rotor"]["options"]):
                            del turbine_json["rotor"]["options"]

            # blade
            blade_filepath_set = set()
            for blade in turbine_json["rotor"]["blades"]:
                blade_filepath = turbine_filepath.parent / blade["file"]
                if blade_filepath not in blade_filepath_set:
                    if version_in <= 0.8 and version_out >= 0.9:
                        convert_blade_v0_8_v0_9(blade_filepath)
                        blade_filepath_set.add(blade_filepath)

            # tower
            tower_filepath = turbine_filepath.parents[0] / turbine_json["tower"]["file"]
            if version_in <= 0.8 and version_out >= 0.9:
                convert_tower_v0_8_v0_9(tower_filepath)
                save_json(turbine_json, turbine_filepath)


def convert_tower_v0_8_v0_9(tower_filepath):
    with open(tower_filepath) as file_tower:
        csv_tower = pd.read_csv(tower_filepath)
        csv_tower["damping_foreaft"] = csv_tower["damping_x"] ** 2
        del csv_tower["damping_x"]
        csv_tower["damping_sideside"] = csv_tower["damping_y"] ** 2
        del csv_tower["damping_y"]
        csv_tower["damping_axial"] = csv_tower["damping_z"] ** 2
        del csv_tower["damping_z"]
        csv_tower["damping_torsion"] = csv_tower["damping_t"] ** 2
        del csv_tower["damping_t"]
        csv_tower["damping_mass"] = 0.0
        csv_tower["fill_density"] = 0.0
        csv_tower.to_csv(tower_filepath, index=False)


def convert_blade_v0_8_v0_9(blade_filepath):
    with open(blade_filepath) as file_blade:
        blade_json = json.load(file_blade)
        coeffs = blade_json["global_variables"]["damping_coefficients"]
        blade_json["global_variables"]["damping_flapwise"] = float(coeffs[0]) ** 2
        blade_json["global_variables"]["damping_edgewise"] = float(coeffs[1]) ** 2
        blade_json["global_variables"]["damping_axial"] = float(coeffs[2]) ** 2
        blade_json["global_variables"]["damping_torsion"] = float(coeffs[3]) ** 2
        blade_json["global_variables"]["damping_mass"] = 0.0
        del blade_json["global_variables"]["damping_coefficients"]
    save_json(blade_json, blade_filepath)


if __name__ == "__main__":
    # make command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "filepath",
        help="Path of main SEAHOWL file.",
        type=str,
    )
    parser.add_argument(
        "v_in",
        help="Version to convert from (e.g. 0.8).",
        type=float,
    )
    parser.add_argument(
        "v_out",
        help="Version to convert in (e.g. 0.9).",
        type=float,
    )
    args = parser.parse_args()

    # convert files
    convert_version(
        main_filepath=args.filepath, version_in=args.v_in, version_out=args.v_out
    )
