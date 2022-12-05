#!/usr/bin/env python3


from pathlib import Path
import json
import jsbeautifier
import numpy as np
import copy
import argparse

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

    # interpolate and merge data
    idx1 = 0
    idx2 = 0
    merged_points = list()
    merge_finished = False
    fractions = list()
    while merge_finished is False:
        # get fraction values
        f1_1 = fractions1[idx1]
        f2_1 = fractions2[idx2]

        if idx1 + 1 == len(fractions1):
            merge_finished = True
            # finished, get last point at 1.0
            assert f1_1 == fractions2[idx2 + 1], "wrong"
            point = copy.deepcopy(json_points1[idx1])
            for key, val in json_points2[idx2 + 1].items():
                point[key] = copy.deepcopy(val)
        elif idx2 + 1 == len(fractions2):
            merge_finished = True
            # finished, get last point at 1.0
            assert f2_1 == fractions1[idx1 + 1], "wrong"
            point = copy.deepcopy(json_points2[idx2])
            for key, val in json_points1[idx1 + 1].items():
                point[key] = copy.deepcopy(val)
        else:
            # keep going
            f1_2 = fractions1[idx1 + 1]
            f2_2 = fractions2[idx2 + 1]

        if merge_finished is False:
            if f1_1 == f2_1:
                # no interpolation needed (just merge data)
                point = copy.deepcopy(json_points1[idx1])
                for key, val in json_points2[idx2].items():
                    point[key] = copy.deepcopy(val)
                # increment indices
                idx1 += 1
                idx2 += 1
            else:
                if f1_1 > f2_1:
                    # start from point in json_points1
                    point = copy.deepcopy(json_points1[idx1])
                    # find interpolation coefficients
                    frange = f2_2 - f2_1
                    coeff1 = 1.0 - (f1_1 - f2_1) / frange
                    coeff2 = 1.0 - (f2_2 - f1_1) / frange
                    # get points to interpolate from
                    point_interp1 = json_points2[idx2]
                    point_interp2 = json_points2[idx2 + 1]
                    # increment index
                    idx2 += 1
                    if fractions2[idx2] > f1_2:
                        idx1 += 1
                elif f1_1 < f2_1:
                    # start from point in json_points2
                    point = copy.deepcopy(json_points2[idx2])
                    # find interpolation coefficients
                    frange = f1_2 - f1_1
                    coeff1 = 1.0 - (f2_1 - f1_1) / frange
                    coeff2 = 1.0 - (f1_2 - f2_1) / frange
                    # get points to interpolate from
                    point_interp1 = json_points1[idx1]
                    point_interp2 = json_points1[idx1 + 1]
                    # increment index
                    idx1 += 1
                    if fractions1[idx1] > f2_2:
                        idx2 += 1

                for key in point_interp1.keys():
                    val1 = point_interp1[key]
                    val2 = point_interp2[key]

                    # if string
                    if isinstance(val1, str) and isinstance(val2, str):
                        if val1 == val2:
                            # only copy if they are the same
                            point[key] = val1
                        else:
                            # otherwise, empty string
                            point[key] = ""
                    else:
                        # if list, convert to ndarray for interpolation
                        is_list = False
                        if isinstance(val1, list) or isinstance(val2, list):
                            is_list = True
                            val1 = np.array(val1)
                            val2 = np.array(val2)

                        # interpolate values to point
                        point[key] = coeff1 * val1 + coeff2 * val2

                        # convert back to list if it was converted to ndarray
                        if is_list is True:
                            point[key] = point[key].tolist()

            # sanity check
            if (
                idx2 + 1 < len(fractions2) - 1
                and fractions1[idx1] > fractions2[idx2 + 1]
            ):
                idx1 -= 1
            elif (
                idx1 + 1 < len(fractions1) - 1
                and fractions2[idx2] > fractions1[idx1 + 1]
            ):
                idx2 -= 1

        # add point to list
        merged_points.append(point)

        fractions.append(point["fraction"])

    sort_order = np.argsort(fractions)
    sorted_points = list()
    for idx in sort_order:
        sorted_points.append(merged_points[idx])

    return sorted_points


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
    beamdyn_json["fpm_mode"] = False
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


def merge_beamdyn2aerodyn(beamdyn_json, aerodyn_json, save_directory=None):

    merged_json = copy.deepcopy(beamdyn_json)
    reference_points = merge_interpolate_points(
        json_points1=beamdyn_json["reference_points"],
        json_points2=aerodyn_json["reference_points"],
    )
    merged_json["reference_points"] = reference_points
    merged_json["discretization_aero"] = aerodyn_json["discretization_aero"]

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / "blade.json"
        save_json(merged_json, fullpath)

    return merged_json


def convert_elastodyn_tower_file(
    filename_elastodyn, filename_elastodyn_tower, save_directory=None
):

    elastodyn_json = dict()
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
    for line in lines[start_idx : start_idx + npoints]:
        point = dict()
        words = line.split()
        point["fraction"] = float(words[0])
        fractions.append(point["fraction"])
        point["density"] = float(words[1])
        point["stiffness_foreaft"] = float(words[2])
        point["stiffness_sideside"] = float(words[3])
        points.append(point)

    elastodyn_json["discretization_elasto"] = fractions
    elastodyn_json["reference_points"] = points

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / filepath.with_suffix(".json")
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
            point["TI"] = float(words[3])
            points.append(point)

    tower_bottom = points[0]["elevation"]
    tower_top = points[-1]["elevation"]
    tower_length = tower_top - tower_bottom
    for point in points:
        point["fraction"] = (point["elevation"] - tower_bottom) / tower_length
        fractions.append(point["fraction"])

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
    merged_json["discretization_aero"] = aerodyn_json["discretization_aero"]

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
        fullpath = Path(save_directory) / "rna.json"
        save_json(rna_json, fullpath)

    return rna_json


def convert_openfast_fst(filename, save_directory=None):
    filepath = Path(filename)
    filedir = filepath.parents[0]

    with open(filepath, "r") as f:
        lines = f.readlines()
        npoints = 0
        start_idx = 0
        npoints = 0
        for ii, line in enumerate(lines):
            words = line.split()
            # ElastoDyn
            if words[1] == "EDFile":
                path_ED = filedir / words[0].replace('"', "")
                with open(path_ED, "r") as f2:
                    lines2 = f2.readlines()
                    for line2 in lines2:
                        words2 = line2.split()
                        # ElastoDyn tower
                        if words2[1] == "TwrFile":
                            path_ED_tower = path_ED.parents[0] / words2[0].replace(
                                '"', ""
                            )
                            break
                tower_elasto_json = convert_elastodyn_tower_file(
                    filename_elastodyn=path_ED,
                    filename_elastodyn_tower=path_ED_tower,
                    save_directory=None,
                )
            # BeamDyn
            if words[1] == "BDBldFile(1)":
                path_BD = filedir / words[0].replace('"', "")
                blade_elasto_json = convert_beamdyn_file(
                    filename=path_BD, save_directory=None
                )
            # AeroDyn
            if words[1] == "AeroFile":
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
            if words[1] == "ServoFile":
                path_servo = filedir / words[0].replace('"', "")

        # tower
        tower_json = merge_elastodyn2aerodyn_tower(
            elastodyn_json=tower_elasto_json,
            aerodyn_json=tower_aero_json,
            save_directory=save_directory,
        )

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
    convert_openfast_fst(filename=filename, save_directory=save_directory)
