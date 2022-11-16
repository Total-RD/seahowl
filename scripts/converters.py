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
        fullpath = Path(save_directory) / filename.with_suffix(".json")
        save_json(airfoils_per_reynolds, fullpath)

    return airfoils_per_reynolds


def convert_aerodyn_files(filename, save_directory=None):
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
                    airfoil = convert_polar_file(
                        polar_filepath, save_directory=save_directory
                    )
                    airfoil_files.append(str(polar_filepath.with_suffix(".json")))

            if len(words) >= 2 and words[1] == "ADBlFile(1)":
                path_blade_file = filedir / words[0].replace('"', '')

    filepath = path_blade_file
    aerodyn_json = dict()
    polar_filenames = list()
    reference_points = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2 and words[1] == "NumBlNds":
                npoints= int(words[0])
                for jj in range(3, npoints + 3):
                    vals = lines[ii + jj].split()
                    point = dict()
                    point["coordinates"] = [
                        float(vals[1]),
                        float(vals[2]),
                        float(vals[0]),
                    ]
                    point["twist"] = float(vals[4])
                    point["chord"] = float(vals[5])
                    point["airfoil_file"] = airfoil_files[int(vals[6]) - 1]
                    reference_points.append(point)
    aerodyn_json["reference_points"] = reference_points

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / filepath.with_suffix(".json")
        save_json(aerodyn_json, fullpath)

    return aerodyn_json


def convert_beamdyn_file(filename, save_directory=None):
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
            point["fraction"] = lines[idx].split()[0]
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
    blade_length = aerodyn_json["reference_points"][-1]["coordinates"][2]
    aerodyn_fractions = list()
    beamdyn_fractions = list()
    for point in aerodyn_json["reference_points"]:
        aerodyn_fractions.append(point["coordinates"][2] / blade_length)
    for point in beamdyn_json["reference_points"]:
        beamdyn_fractions.append(point["fraction"])

    idx = 0
    merged_json = copy.deepcopy(aerodyn_json)
    for point in merged_json["reference_points"]:
        afraction = float(point["coordinates"][2]) / blade_length
        bfraction0 = float(beamdyn_fractions[idx])
        bfraction1 = float(beamdyn_fractions[idx + 1])
        while afraction > bfraction1:
            idx += 1
            bfraction0 = float(beamdyn_fractions[idx])
            bfraction1 = float(beamdyn_fractions[idx + 1])
        bfraction_range = bfraction1 - bfraction0
        mm1 = np.array(beamdyn_json["reference_points"][idx]["mass_matrix"])
        mm2 = np.array(beamdyn_json["reference_points"][idx + 1]["mass_matrix"])
        sm1 = np.array(beamdyn_json["reference_points"][idx]["stiffness_matrix"])
        sm2 = np.array(beamdyn_json["reference_points"][idx + 1]["stiffness_matrix"])
        coeff1 = 1.0 - (afraction - bfraction0) / bfraction_range
        coeff2 = 1.0 - (bfraction1 - afraction) / bfraction_range
        mass_matrix = coeff1 * mm1 + coeff2 * mm2
        stiffness_matrix = coeff1 * sm1 + coeff2 * sm2

        point["mass_matrix"] = mass_matrix.tolist()
        point["coordinates"][1] = 0.0
        point["stiffness_matrix"] = stiffness_matrix.tolist()

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / "blade.json"
        save_json(merged_json, fullpath)

    return merged_json


def convert_elastodyn_tower_file(filename, save_directory=None):
    filepath = Path(filename)
    elastodyn_json = dict()

    points = list()
    with open(filepath, "r") as f:
        lines = f.readlines()
        npoints = 0
        for ii, line in enumerate(lines):
            words = line.split()
            if len(words) >= 2 and words[1] == "NTwInpSt":
                npoints = int(words[0])
                break

    assert npoints != 0, "Could not find tower ElastoDyn info in given file."
    start_idx = 19
    for line in lines[start_idx : start_idx + npoints]:
        point = dict()
        words = line.split()
        point["fraction"] = float(words[0])
        point["density"] = float(words[1])
        point["stiffness_foreaft"] = float(words[2])
        point["stiffness_sideside"] = float(words[3])
        points.append(point)

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
    tower_length = tower_top-tower_bottom
    for point in points:
        point["fraction"] = (point["elevation"] - tower_bottom) / tower_length

    aerodyn_json["reference_points"] = points

    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / filepath.with_suffix(".json")
        save_json(aerodyn_json, fullpath)

    return aerodyn_json

def merge_elastodyn2aerodyn_tower(elastodyn_json, aerodyn_json, save_directory=None):
    aerodyn_fractions = list()
    elastodyn_fractions = list()
    for point in aerodyn_json["reference_points"]:
        aerodyn_fractions.append(point["fraction"])
    for point in elastodyn_json["reference_points"]:
        elastodyn_fractions.append(point["fraction"])

    idx = 0
    merged_json = copy.deepcopy(elastodyn_json)
    for idx_b, point in enumerate(merged_json["reference_points"]):
        afraction = float(elastodyn_fractions[idx_b])
        bfraction0 = float(aerodyn_fractions[idx])
        bfraction1 = float(aerodyn_fractions[idx + 1])
        while afraction > bfraction1:
            idx += 1
            bfraction0 = float(aerodyn_fractions[idx])
            bfraction1 = float(aerodyn_fractions[idx + 1])
        bfraction_range = bfraction1 - bfraction0
        dd1 = np.array(aerodyn_json["reference_points"][idx]["diameter"])
        dd2 = np.array(aerodyn_json["reference_points"][idx + 1]["diameter"])
        cd1 = np.array(aerodyn_json["reference_points"][idx]["drag_coefficient"])
        cd2 = np.array(aerodyn_json["reference_points"][idx + 1]["drag_coefficient"])
        coeff1 = 1.0 - (afraction - bfraction0) / bfraction_range
        coeff2 = 1.0 - (bfraction1 - afraction) / bfraction_range
        dd = coeff1 * dd1 + coeff2 * dd2
        cd = coeff1 * cd1 + coeff2 * cd2

        point["diameter"] = dd
        point["drag_coefficient"] = cd
    
    # save to file
    if save_directory is not None:
        fullpath = Path(save_directory) / "tower.json"
        save_json(merged_json, fullpath)

    return merged_json


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
            if words[1] == 'EDFile':
                path_ED = filedir / words[0].replace('"', '')
                with open(path_ED, "r") as f2:
                    lines2 = f2.readlines()
                    for line2 in lines2:
                        words2 = line2.split()
                        if words2[1] == 'TwrFile':
                            path_ED_tower = path_ED.parents[0] / words2[0].replace('"', '')
                            break
                tower_elasto_json = convert_elastodyn_tower_file(filename=path_ED_tower, save_directory=save_directory)
            if words[1] == 'BDBldFile(1)':
                path_BD = filedir / words[0].replace('"', '')
                with open(path_BD, "r") as f2:
                    lines2 = f2.readlines()
                    for line2 in lines2:
                        words2 = line2.split()
                        if words2[1] == 'BldFile':
                            path_BD_blade = path_BD.parents[0] / words2[0].replace('"', '')
                            break
                blade_elasto_json = convert_beamdyn_file(filename=path_BD_blade, save_directory=save_directory)
            if words[1] == 'AeroFile':
                path_AD = filedir / words[0].replace('"', '')
                blade_aero_json = convert_aerodyn_files(filename=path_AD, save_directory=save_directory)
                tower_aero_json = convert_aerodyn_tower_file(filename=path_AD, save_directory=save_directory)
        blade_json = merge_beamdyn2aerodyn(beamdyn_json=blade_elasto_json, aerodyn_json=blade_aero_json, save_directory=save_directory)
        tower_json = merge_elastodyn2aerodyn_tower(elastodyn_json=tower_elasto_json, aerodyn_json=tower_aero_json, save_directory=save_directory)


if __name__ == "__main__":

    # make command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--filename", help="Name of OpenFAST .fst file.", default="./IEA-15-240-RWT-Monopile.fst"
    )
    parser.add_argument(
        "--save-directory", help="Directory to save output files.", default="./converted"
    )
    args = parser.parse_args()

    # get arguments
    filename = args.filename
    save_directory = Path(args.save_directory)

    # convert files
    convert_openfast_fst(filename=filename, save_directory=save_directory)
