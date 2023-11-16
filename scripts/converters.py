#!/usr/bin/env python3


from pathlib import Path
import json
import jsbeautifier
import numpy as np
import copy
import argparse
import shutil

options = jsbeautifier.default_options
options.indent_size = 2


def save_json(json_dict, path):
    mydirectory = Path(path).parent
    mydirectory.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write(jsbeautifier.beautify(json.dumps(json_dict), options))


def merge_interpolate_points(json_points1, json_points2):
    # get fractions as lists
    fractions1 = list()
    fractions2 = list()
    for point in json_points1:
        fractions1.append(point["fraction"])
    for point in json_points2:
        fractions2.append(point["fraction"])
    tol = 6
    fractions1 = np.round(fractions1, tol)
    fractions2 = np.round(fractions2, tol)

    fractions = np.union1d(fractions1, fractions2)

    idx1 = 0
    idx2 = 0
    merged_points = list()
    for idx, fraction in enumerate(fractions):
        interp = False  # interpolate a point ?
        if fraction == fractions1[idx1]:
            point1 = copy.deepcopy(json_points1[idx1])
            idx1 += 1
        else:
            interp = True
            frange = fractions1[idx1] - fractions1[idx1 - 1]
            coeff1 = 1.0 - (fraction - fractions1[idx1 - 1]) / frange
            coeff2 = 1.0 - (fractions1[idx1] - fraction) / frange
            interp1 = json_points1[idx1 - 1]
            interp2 = json_points1[idx1]
            point1 = copy.deepcopy(json_points1[idx1])
            interp_point = point1  # pointer to point1
            if idx > len(fractions) and fractions[idx + 1] > fractions1[idx1]:
                idx1 += 1

        if fraction == fractions2[idx2]:
            point2 = copy.deepcopy(json_points2[idx2])
            idx2 += 1
        else:
            interp = True
            frange = fractions2[idx2] - fractions2[idx2 - 1]
            coeff1 = 1.0 - (fraction - fractions2[idx2 - 1]) / frange
            coeff2 = 1.0 - (fractions2[idx2] - fraction) / frange
            interp1 = json_points2[idx2 - 1]
            interp2 = json_points2[idx2]
            point2 = copy.deepcopy(json_points2[idx2])
            interp_point = point2  # pointer to point1
            if idx > len(fractions) and fractions[idx + 1] > fractions2[idx2]:
                idx2 += 1

        if interp is True:
            # get points to interpolate from
            for key in interp_point.keys():
                val1 = interp1[key]
                val2 = interp2[key]

                # if string
                if isinstance(val1, str) and isinstance(val2, str):
                    if val1 == val2:
                        # only copy if they are the same
                        interp_point[key] = val1
                    else:
                        # otherwise, empty string
                        interp_point[key] = ""
                else:
                    # if list, convert to ndarray for interpolation
                    is_list = False
                    if isinstance(val1, list) or isinstance(val2, list):
                        is_list = True
                        val1 = np.array(val1)
                        val2 = np.array(val2)

                    # interpolate values to point
                    interp_point[key] = coeff1 * val1 + coeff2 * val2

                    # convert back to list if it was converted to ndarray
                    if is_list is True:
                        interp_point[key] = interp_point[key].tolist()

        point = copy.deepcopy(point1)
        for key, val in point2.items():
            point[key] = val
        point["fraction"] = fraction

        # add point to list
        merged_points.append(point)

    return merged_points


def convert_polar_file(filename, save_directory=None):
    filepath = Path(filename)
    with open(filepath, "r") as f:
        lines = f.readlines()
        airfoils_per_reynolds = list()
        coefficients = list()
        cl = list()
        cd = list()
        cm = list()
        idx_airfoil = 0
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2:
                if ii > 10:
                    for iw, word in enumerate(words):
                        if word == "Re":
                            for _ in range(iw):
                                airfoil = dict()
                                airfoil["reynolds_number"] = float(words[0])
                                airfoils_per_reynolds.append(airfoil)
                            break
                    if words[1] == "NumAlf":
                        airfoil = airfoils_per_reynolds[idx_airfoil]
                        idx_airfoil += 1
                        npolars = int(words[0])
                        airfoil["header"] = ["alpha", "Cl", "Cd", "Cm"]
                        for jj in range(3, npolars + 3):
                            coeffs = lines[ii + jj].split()
                            coefficients.append(
                                [
                                    float(coeffs[0]),
                                    float(coeffs[1]),
                                    float(coeffs[2]),
                                    float(coeffs[3]),
                                ]
                            )
                        airfoil["coefficients"] = coefficients

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / filename.with_suffix(".json").name
        save_json(airfoils_per_reynolds, fullpath)

    return airfoils_per_reynolds


def convert_aerodyn_files(
    filename, save_directory=None, save_polar=True, save_aerodyn=False
):
    filepath = Path(filename)
    filedir = filepath.parents[0]
    aerodyn_json = dict()
    polar_filenames = list()
    airfoil_files = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2 and words[1] == "NumAFfiles":
                nfiles = int(words[0])
                for jj in range(1, nfiles + 1):
                    polar_filename = lines[ii + jj].split()[0]
                    polar_filename = polar_filename.replace('"', "")
                    polar_filename = polar_filename.replace("\n", "")
                    polar_filenames.append(str(polar_filename))
                    polar_filepath = filepath.parent / polar_filename
                    polar_filename_json = Path(polar_filename).with_suffix(".json").name
                    polar_filepath_json = polar_filename_json
                    airfoils_dir = "airfoils"
                    if save_directory is not None and save_polar is True:
                        polar_save_directory = Path(save_directory) / airfoils_dir
                        polar_filepath_json = polar_save_directory / polar_filename_json
                    else:
                        polar_save_directory = "./airfoils"
                    airfoil = convert_polar_file(
                        polar_filepath, save_directory=polar_save_directory
                    )
                    airfoil_files.append(str(Path(airfoils_dir) / polar_filename_json))

            if len(words) >= 2 and words[1] == "ADBlFile(1)":
                path_blade_file = filedir / words[0].replace('"', "")

    filepath = path_blade_file
    aerodyn_json = dict()
    polar_filenames = list()
    reference_points = list()
    fractions = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2 and words[1] == "NumBlNds":
                npoints = int(words[0])
                blade_length = float(lines[ii + 3 + npoints - 1].split()[0])
                for jj in range(3, npoints + 3):
                    vals = lines[ii + jj].split()
                    point = dict()
                    point["coordinates_aero"] = [
                        float(vals[1]),
                        float(vals[2]),
                        float(vals[0]),
                    ]
                    point["fraction"] = float(vals[0]) / blade_length
                    fractions.append(point["fraction"])
                    point["twist"] = float(vals[4])
                    point["chord"] = float(vals[5])
                    point["airfoil_file"] = airfoil_files[int(vals[6]) - 1]
                    reference_points.append(point)

    aerodyn_json["discretization_aero"] = fractions
    aerodyn_json["reference_points"] = reference_points

    # save to file
    if save_directory is not None and save_aerodyn is True:
        fullpath = Path(save_directory) / filepath.with_suffix(".json").name
        save_json(aerodyn_json, fullpath)

    return aerodyn_json


def convert_beamdyn_file(filename, save_directory=None):
    filepath = Path(filename)
    beamdyn_json = dict()
    reference_points = list()
    fractions = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) > 1 and words[1] == "kp_total":
                npoints = int(words[0])
                coords_list = lines[ii + 4 : ii + 4 + npoints]
                blade_length = float(coords_list[-1].split()[2])
                for coords in coords_list:
                    point = dict()
                    coords = coords.split()
                    point["coordinates"] = [
                        float(coords[0]),
                        float(coords[1]),
                        float(coords[2]),
                    ]
                    point["twist"] = float(coords[3])
                    point["fraction"] = float(coords[2]) / blade_length
                    fractions.append(point["fraction"])
                    reference_points.append(point)
            elif len(words) > 1 and words[1] == "BldFile":
                path_BD_blade = filepath.parents[0] / words[0].replace('"', "")
                blade_elasto_json = convert_beamdyn_blade_file(
                    filename=path_BD_blade, save_directory=save_directory
                )

    beamdyn_json["discretization_elasto"] = fractions

    # add default options
    beamdyn_json["damping_coefficients"] = [0.03, 0.03, 0.03, 0.06]

    reference_points = merge_interpolate_points(
        json_points1=reference_points,
        json_points2=blade_elasto_json["reference_points"],
    )
    beamdyn_json["reference_points"] = reference_points

    return beamdyn_json


def convert_beamdyn_blade_file(filename, save_directory=None):
    filepath = Path(filename)
    beamdyn_json = dict()
    with open(filepath, "r") as f:
        lines = f.readlines()
        start_idx = 10
        nprops = (len(lines) - start_idx) / 15
        points = list()
        for ii in range(int(nprops)):
            idx = start_idx + ii * 15
            point = dict()
            point["fraction"] = float(lines[idx].split()[0])
            sm1 = lines[idx + 1].split()
            sm2 = lines[idx + 2].split()
            sm3 = lines[idx + 3].split()
            sm4 = lines[idx + 4].split()
            sm5 = lines[idx + 5].split()
            sm6 = lines[idx + 6].split()
            point["stiffness_matrix"] = [
                [
                    float(sm1[0]),
                    float(sm1[1]),
                    float(sm1[2]),
                    float(sm1[3]),
                    float(sm1[4]),
                    float(sm1[5]),
                ],
                [
                    float(sm2[0]),
                    float(sm2[1]),
                    float(sm2[2]),
                    float(sm2[3]),
                    float(sm2[4]),
                    float(sm2[5]),
                ],
                [
                    float(sm3[0]),
                    float(sm3[1]),
                    float(sm3[2]),
                    float(sm3[3]),
                    float(sm3[4]),
                    float(sm3[5]),
                ],
                [
                    float(sm4[0]),
                    float(sm4[1]),
                    float(sm4[2]),
                    float(sm4[3]),
                    float(sm4[4]),
                    float(sm4[5]),
                ],
                [
                    float(sm5[0]),
                    float(sm5[1]),
                    float(sm5[2]),
                    float(sm5[3]),
                    float(sm5[4]),
                    float(sm5[5]),
                ],
                [
                    float(sm6[0]),
                    float(sm6[1]),
                    float(sm6[2]),
                    float(sm6[3]),
                    float(sm6[4]),
                    float(sm6[5]),
                ],
            ]

            mm1 = lines[idx + 8].split()
            mm2 = lines[idx + 9].split()
            mm3 = lines[idx + 10].split()
            mm4 = lines[idx + 11].split()
            mm5 = lines[idx + 12].split()
            mm6 = lines[idx + 13].split()
            point["mass_matrix"] = [
                [
                    float(mm1[0]),
                    float(mm1[1]),
                    float(mm1[2]),
                    float(mm1[3]),
                    float(mm1[4]),
                    float(mm1[5]),
                ],
                [
                    float(mm2[0]),
                    float(mm2[1]),
                    float(mm2[2]),
                    float(mm2[3]),
                    float(mm2[4]),
                    float(mm2[5]),
                ],
                [
                    float(mm3[0]),
                    float(mm3[1]),
                    float(mm3[2]),
                    float(mm3[3]),
                    float(mm3[4]),
                    float(mm3[5]),
                ],
                [
                    float(mm4[0]),
                    float(mm4[1]),
                    float(mm4[2]),
                    float(mm4[3]),
                    float(mm4[4]),
                    float(mm4[5]),
                ],
                [
                    float(mm5[0]),
                    float(mm5[1]),
                    float(mm5[2]),
                    float(mm5[3]),
                    float(mm5[4]),
                    float(mm5[5]),
                ],
                [
                    float(mm6[0]),
                    float(mm6[1]),
                    float(mm6[2]),
                    float(mm6[3]),
                    float(mm6[4]),
                    float(mm6[5]),
                ],
            ]

            points.append(point)

    beamdyn_json["reference_points"] = points

    if save_directory is not None:
        fullpath = Path(save_directory) / filepath.with_suffix(".json")
        save_json(beamdyn_json, fullpath)

    return beamdyn_json


def convert_elastodyn_blade_file(filename, blade_length, save_directory=None):
    filepath = Path(filename)
    elastodyn_json = dict()
    fractions = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        start_idx = 16
        points = list()
        nprops = int(lines[3].split()[0])
        for ii in range(int(nprops)):
            idx = start_idx + ii
            props = lines[idx].split()
            point = dict()
            point["fraction"] = float(props[0])
            fractions.append(point["fraction"])
            point["coordinates"] = [0.0, 0.0, point["fraction"] * blade_length]
            point["twist"] = float(props[2])
            point["mass_matrix"] = np.zeros((6, 6)).tolist()
            point["mass_matrix"][2][2] = float(props[3])
            point["stiffness_matrix"] = np.zeros((6, 6)).tolist()
            point["stiffness_matrix"][2][2] = 1e9
            point["stiffness_matrix"][3][3] = float(props[5])
            point["stiffness_matrix"][4][4] = float(props[4])
            point["stiffness_matrix"][5][5] = 1e9
            points.append(point)

    elastodyn_json["reference_points"] = points

    # add default options
    elastodyn_json["discretization_elasto"] = fractions
    elastodyn_json["damping_coefficients"] = [0.03, 0.03, 0.03, 0.06]

    if save_directory is not None:
        fullpath = Path(save_directory) / filepath.with_suffix(".json")
        save_json(elastodyn_json, fullpath)

    return elastodyn_json


def merge_beamdyn2aerodyn(beamdyn_json, aerodyn_json, save_directory=None):
    merged_json = copy.deepcopy(beamdyn_json)
    merged_json.pop("discretization_aero", None)
    merged_json.pop("discretization_elasto", None)
    reference_points = merge_interpolate_points(
        json_points1=beamdyn_json["reference_points"],
        json_points2=aerodyn_json["reference_points"],
    )
    merged_json["reference_points"] = reference_points
    for reference_point in merged_json["reference_points"]:
        reference_point["offset_aero"] = (
            np.array(reference_point["coordinates_aero"])
            - np.array(reference_point["coordinates"])
        ).tolist()[:2]

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / "blade.json"
        save_json(merged_json, fullpath)

    return merged_json


def convert_elastodyn_tower_file(
    filename_elastodyn, filename_elastodyn_tower, save_directory=None
):
    elastodyn_json = dict()
    elastodyn_json["type"] = "cylinder"
    elastodyn_json["damping_coefficients"] = [0.02, 0.02, 0.02, 0.02]

    # elastodyn (general info)
    filepath = Path(filename_elastodyn)
    points = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) > 1:
                if words[1] == "TowerHt":
                    elastodyn_json["height"] = float(words[0])
                if words[1] == "TowerBsHt":
                    elastodyn_json["base_height"] = float(words[0])

    # elastodyn tower (reference points)
    filepath = Path(filename_elastodyn_tower)
    points = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        npoints = 0
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) > 1:
                if words[1] == "TowerHt":
                    elastodyn_json["height"] = float(words[0])
                if words[1] == "TowerBsHt":
                    elastodyn_json["base_height"] = float(words[0])
            if len(words) >= 2 and words[1] == "NTwInpSt":
                npoints = int(words[0])
                break

    assert npoints != 0, "Could not find tower ElastoDyn info in given file."
    start_idx = 19
    fractions = list()

    # assume basic steel properties and cylinder shape
    young_modulus = 210e9
    density = 7850
    poisson_ratio = 0.3
    options = {}
    options["density"] = density
    options["young_modulus"] = young_modulus
    options["poisson_ratio"] = poisson_ratio
    elastodyn_json["options"] = options
    for line in lines[start_idx : start_idx + npoints]:
        point = dict()
        words = line.split()
        point["fraction"] = float(words[0])
        fractions.append(point["fraction"])
        density_linear = float(words[1])
        inertia = float(words[2]) / young_modulus  # area moment of inertia
        # inner
        area_inner = (
            (4 * np.pi * inertia - (density_linear / density) ** 2)
            * (density / density_linear)
            * 0.5
        )
        diameter_inner = np.sqrt(4 * area_inner / np.pi)
        # outer
        area_outer = density_linear / density + area_inner
        diameter_outer = np.sqrt(4 * area_outer / np.pi)
        point["diameter"] = diameter_outer
        # thickness
        thickness = 0.5 * (diameter_outer - diameter_inner)
        point["thickness"] = thickness

        point["drag_coefficient"] = 0.5  # cylinders
        points.append(point)

    elastodyn_json["reference_points"] = points

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / "tower.json"
        save_json(elastodyn_json, fullpath)

    return elastodyn_json


def convert_aerodyn_tower_file(filename, save_directory=None):
    filepath = Path(filename)
    aerodyn_json = dict()

    points = list()
    fractions = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        npoints = 0
        start_idx = 0
        npoints = 0
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2 and words[1] == "NumTwrNds":
                npoints = int(words[0])
                start_idx = ii + 3
                break

        assert start_idx != 0, "Could not find tower AeroDyn info in given file."
        for line in lines[start_idx : start_idx + npoints]:
            point = dict()
            words = line.split()
            point["elevation"] = float(words[0])
            point["diameter"] = float(words[1])
            point["drag_coefficient"] = float(words[2])
            if len(words) > 3:
                point["TI"] = float(words[3])
            else:
                point["TI"] = 0.0
            points.append(point)

    tower_bottom = points[0]["elevation"]
    tower_top = points[-1]["elevation"]
    tower_length = tower_top - tower_bottom
    for point in points:
        point["fraction"] = (point["elevation"] - tower_bottom) / tower_length
        fractions.append(point["fraction"])
        point.pop("elevation", None)
        point.pop("TI", None)

    aerodyn_json["discretization_aero"] = fractions
    aerodyn_json["reference_points"] = points

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / filepath.with_suffix(".json")
        save_json(aerodyn_json, fullpath)

    return aerodyn_json


def merge_elastodyn2aerodyn_tower(elastodyn_json, aerodyn_json, save_directory=None):
    merged_json = copy.deepcopy(elastodyn_json)
    reference_points = merge_interpolate_points(
        json_points1=elastodyn_json["reference_points"],
        json_points2=aerodyn_json["reference_points"],
    )
    merged_json["reference_points"] = reference_points
    merged_json.pop("discretization_aero", None)
    merged_json.pop("discretization_elasto", None)

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / "tower.json"
        save_json(merged_json, fullpath)

    return merged_json


def convert_elastodyn_rna_file(
    elastodyn_filename, servodyn_filename=None, save_directory=None
):
    filepath = Path(elastodyn_filename)
    filedir = filepath.parents[0]

    rna_json = dict()
    # prepare entries
    rna_json["precones"] = list()
    rna_json["shaft"] = dict()
    rna_json["nacelle"] = dict()
    rna_json["nacelle"]["CM"] = [0, 0, 0]
    rna_json["drivetrain"] = dict()
    rna_json["hub"] = dict()

    with open(filepath, "r") as f:
        lines = f.readlines()
        # populate dicts
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) > 1:
                # precones
                if (
                    words[1] == "PreCone(1)"
                    or words[1] == "PreCone(2)"
                    or words[1] == "PreCone(3)"
                ):
                    rna_json["precones"].append(float(words[0]))
                # shaft
                elif words[1] == "ShftTilt":
                    rna_json["shaft"]["tilt"] = float(words[0])
                elif words[1] == "Twr2Shft":
                    rna_json["shaft"]["distance_from_towertop"] = float(words[0])
                # nacelle
                elif words[1] == "NacMass":
                    rna_json["nacelle"]["mass"] = float(words[0])
                elif words[1] == "NacYIner":
                    rna_json["nacelle"]["inertia"] = float(words[0])
                elif words[1] == "NacCMxn":
                    rna_json["nacelle"]["CM"][0] = float(words[0])
                elif words[1] == "NacCMyn":
                    rna_json["nacelle"]["CM"][1] = float(words[0])
                elif words[1] == "NacCMzn":
                    rna_json["nacelle"]["CM"][2] = float(words[0])
                elif words[1] == "YawBrMass":
                    rna_json["nacelle"]["yaw_bearing_mass"] = float(words[0])
                # drivetrain
                elif words[1] == "GenIner":
                    rna_json["drivetrain"]["generator_inertia"] = float(words[0])
                elif words[1] == "GBoxEff":
                    rna_json["drivetrain"]["gearbox_efficiency"] = float(words[0])
                elif words[1] == "GBRatio":
                    rna_json["drivetrain"]["gearbox_ratio"] = float(words[0])
                # hub
                elif words[1] == "HubRad":
                    rna_json["hub"]["radius"] = float(words[0])
                elif words[1] == "HubMass":
                    rna_json["hub"]["mass"] = float(words[0])
                elif words[1] == "HubIner":
                    rna_json["hub"]["inertia"] = float(words[0])
                elif words[1] == "OverHang":
                    rna_json["hub"]["overhang"] = float(words[0])
                elif words[1] == "HubCM":
                    rna_json["hub"]["CM"] = float(words[0])

    # extra info from servo file
    if servodyn_filename is not None:
        filepath = Path(servodyn_filename)
        with open(filepath, "r") as f:
            lines = f.readlines()
            for ii, line in enumerate(lines):
                words = line.split()
                if len(words) > 1:
                    if words[1] == "GenEff":
                        rna_json["drivetrain"]["generator_efficiency"] = float(words[0])

    if save_directory is not None:
        # remove precones for RNA file (will be added to turbine file)
        rna_json2 = copy.deepcopy(rna_json)
        del rna_json2["precones"]
        fullpath = Path(save_directory) / "rna.json"
        save_json(rna_json2, fullpath)

    return rna_json


def convert_openfast_fst(filename, save_directory=None, use_beamdyn=True):
    filepath = Path(filename)
    filedir = filepath.parents[0]

    # main json with default values to overwrite when parsing OpenFAST files
    main_json = {
        "numerics": {
            "dt": 0.1,
            "t_end": 2000.0,
            "statics": {
                "linear_step": True,
                "nonlinear_steps": 10,
            },
        },
        "outputs": {"dt": 0.1, "VTK": False, "log_level": "info"},
        "environment": {"file": "./environment.json"},
        "turbines": [
            {
                "translation": [0, 0, 0],
                "rotation": 0,
                "use_aerodyn": False,
                "file": str(Path("./turbine.json")),
            }
        ],
    }

    environment_json = {
        "gravity": [0.0, 0.0, -9.81],
        "wind": {
            "air_density": 1.225,
            "type": "ramp",
            "options": {
                "reference_height": 150,
                "shear_coefficient": 0.12,
                "velocity_start": [12, 0, 0],
                "velocity_end": [25, 0, 0],
                "time_start": 500,
                "time_end": 1700,
            },
        },
    }

    with open(filepath, "r") as f:
        lines = f.readlines()
        npoints = 0
        start_idx = 0
        npoints = 0
        blade_length = 0
        blade_elasto_json = None
        for ii, line in enumerate(lines):
            words = line.split()
            # Main
            if len(words) > 1:
                if words[1] == "DT":
                    main_json["numerics"]["dt"] = float(words[0])
                elif words[1] == "TMax":
                    main_json["numerics"]["t_end"] = float(words[0])
                elif words[1] == "Gravity":
                    environment_json["gravity"][2] = -float(words[0])
                elif words[1] == "AirDens":
                    environment_json["air_density"] = float(words[0])
                elif words[1] == "WrVTK":
                    if float(words[0]) != 0.0:
                        main_json["outputs"]["VTK"] = True
                elif words[1] == "VTK_fps":
                    main_json["outputs"]["dt"] = 1.0 / float(words[0])
                # ElastoDyn
                elif words[1] == "EDFile":
                    path_ED = filedir / words[0].replace('"', "")
                    with open(path_ED, "r") as f2:
                        lines2 = f2.readlines()
                        for line2 in lines2:
                            words2 = line2.split()
                            # blade length
                            if len(words2) > 1 and words2[1] == "TipRad":
                                blade_length += float(words2[0])
                            elif len(words2) > 1 and words2[1] == "HubRad":
                                blade_length -= float(words2[0])
                            # ElastoDyn tower
                            elif len(words2) > 1 and words2[1] == "TwrFile":
                                path_ED_tower = path_ED.parents[0] / words2[0].replace(
                                    '"', ""
                                )
                            elif len(words2) > 1 and (
                                words2[1] == "BldFile1" or words2[1] == "BldFile(1)"
                            ):
                                path_ED_blade = path_ED.parents[0] / words2[0].replace(
                                    '"', ""
                                )
                    tower_elasto_json = convert_elastodyn_tower_file(
                        filename_elastodyn=path_ED,
                        filename_elastodyn_tower=path_ED_tower,
                        save_directory=save_directory,
                    )
                    if blade_elasto_json is None:
                        blade_elasto_json = convert_elastodyn_blade_file(
                            filename=path_ED_blade,
                            save_directory=None,
                            blade_length=blade_length,
                        )
                elif words[1] == "TwrFile":
                    path_ED_tower = filedir / words[0].replace('"', "")
                    tower_elasto_json = {}
                    tower_aero_json = {}
                elif words[1] == "BldFile1" or words[1] == "BldFile(1)":
                    path_ED_blade = filedir / words[0].replace('"', "")
                    blade_elasto_json = convert_elastodyn_blade_file(
                        filename=path_ED_blade,
                        save_directory=save_directory,
                    )

                # BeamDyn
                elif words[1] == "BDBldFile(1)" and use_beamdyn is True:
                    if (
                        words[0] != '""'
                        and words[0] != '"unused"'
                        and words[0] != '"none"'
                    ):
                        path_BD = filedir / words[0].replace('"', "")
                        blade_elasto_json = convert_beamdyn_file(
                            filename=path_BD, save_directory=None
                        )
                # AeroDyn
                elif words[1] == "AeroFile":
                    path_AD = filedir / words[0].replace('"', "")
                    blade_aero_json = convert_aerodyn_files(
                        filename=path_AD,
                        save_directory=save_directory,
                        save_polar=True,
                        save_aerodyn=False,
                    )
                    tower_aero_json = convert_aerodyn_tower_file(
                        filename=path_AD, save_directory=None
                    )
                # ServoDyn
                elif words[1] == "ServoFile":
                    path_servo = filedir / words[0].replace('"', "")

        # rna
        rna_json = convert_elastodyn_rna_file(
            elastodyn_filename=path_ED,
            servodyn_filename=path_servo,
            save_directory=save_directory,
        )

        # blade
        blade_json = merge_beamdyn2aerodyn(
            beamdyn_json=blade_elasto_json,
            aerodyn_json=blade_aero_json,
            save_directory=save_directory,
        )

        # turbine
        # get DISCON path
        filepath = Path(path_servo)
        path_DISCON_new = ""
        controller_json = {
            "type": "",
        }
        with open(filepath, "r") as f:
            lines = f.readlines()
            for ii, line in enumerate(lines):
                words = line.split()
                if (
                    len(words) > 1
                    and words[1] == "DLL_InFile"
                    and words[0] != '"unused"'
                ):
                    controller_json["type"] = "DISCON"
                    path_DISCON = filepath.parent / words[0].replace('"', "")
                    path_DISCON_new = Path("controller") / path_DISCON.name
                    (Path(save_directory) / path_DISCON_new).parent.mkdir(
                        parents=True, exist_ok=True
                    )
                    controller_json["options"] = {
                        "infile": str(path_DISCON_new),
                        "libfile": "./controller/libdiscon.so",
                    }
                    # open DISCON file
                    with open(path_DISCON, "r") as f2:
                        lines2 = f2.readlines()
                        for jj, line2 in enumerate(lines2):
                            words2 = line2.split()
                            if len(words2) > 2 and words2[2] == "PerfFileName":
                                path_PerFile = path_DISCON.parent / words2[0].replace(
                                    '"', ""
                                )
                                path_PerFile_new = (
                                    path_DISCON_new.parent / path_PerFile.name
                                )
                                shutil.copyfile(
                                    path_PerFile,
                                    Path(save_directory) / path_PerFile_new,
                                )
                                # replace path of PerFile i new DISCON file
                                lines2[jj] = lines2[jj].replace(
                                    words2[0].replace('"', ""), str(path_PerFile.name)
                                )
                                break
                        with open(Path(save_directory) / path_DISCON_new, "w") as f3:
                            f3.writelines(lines2)

        turbine_json = {
            "aero": {
                "solver": "BEMT",
                "options": {"hub_loss": True, "tip_loss": True, "tower_shadow": True},
            },
            "rotor": {
                "type": "fea",
                "options": {
                    "fpm": False,
                },
                "discretization": {
                    "elasto": blade_elasto_json["discretization_elasto"],
                    "aero": blade_aero_json["discretization_aero"],
                },
                "blades": [
                    {
                        "file": str(Path("./blade.json")),
                        "initial_pitch": 0.0,
                        "precone": rna_json["precones"][ii],
                    }
                    for ii in range(3)
                ],
            },
            "rna": {
                "initial_pitch_collective": 0.0,
                "file": str(Path("./rna.json")),
            },
            "tower": {
                "discretization": {
                    "elasto": [],
                    "aero": [],
                },
                "file": str(Path("./tower.json")),
            },
            "controller": controller_json,
        }

        if save_directory is not None:
            # save turbine json
            fullpath = Path(save_directory) / "turbine.json"
            save_json(turbine_json, fullpath)

            # save environement json
            fullpath = Path(save_directory) / "environment.json"
            save_json(environment_json, fullpath)

            # save main json
            fullpath = Path(save_directory) / "main.json"
            save_json(main_json, fullpath)


if __name__ == "__main__":
    # make command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--filename",
        help="Name of OpenFAST .fst file.",
        default="./IEA-15-240-RWT-Monopile.fst",
    )
    parser.add_argument(
        "--save-directory",
        help="Directory to save output files.",
        default="./converted",
    )
    args = parser.parse_args()

    # get arguments
    filename = args.filename
    save_directory = Path(args.save_directory)

    # convert files
    convert_openfast_fst(
        filename=filename, save_directory=save_directory, use_beamdyn=True
    )
