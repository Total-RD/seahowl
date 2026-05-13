# SPDX-License-Identifier: Apache-2.0
# Copyright 2022–2026 TotalEnergies
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

import os
import unittest
import subprocess
import tempfile
import pathlib

from scilens import StandaloneTaskRunner
from scilens.helpers.assets import Assets

TEST_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(TEST_DIR))

VAL_PY_DIR = os.path.join(os.getcwd(), "build")
REF_PY_DIR = os.path.join(os.getcwd(), "ref/bin")
EXAMPLES_DIR = os.path.join(os.getcwd(), "examples/python")

ENV_PYTHONPATH = "PYTHONPATH"

test_only = True
only_compare = False


def clean_directory(directory):
    """Recursively delete all files and directories in the given directory."""
    for root, dirs, files in os.walk(directory, topdown=False):
        for name in files:
            if name.startswith("."):
                continue
            os.remove(os.path.join(root, name))
        for name in dirs:
            clean_directory(os.path.join(root, name))
            os.rmdir(os.path.join(root, name))


def run_modified_script(script_path, modifications):
    # Read the original script
    with open(script_path, "r") as f:
        code = f.read()
    # Apply the modifications (replace a string)
    for old, new in modifications.items():
        if os.name == "nt":
            new = new.replace("\\", "/")
        code = code.replace(old, new)
    # Write to a temporary file
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as tmp:
        tmp.write(code)
        tmp_script_path = tmp.name
    # Execute the modified script
    try:
        subprocess.run(["python3", tmp_script_path], check=True, cwd=EXAMPLES_DIR)
    finally:
        # Clean up the temporary file
        os.remove(tmp_script_path)


class TestNonRegressionPy(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):

        # Setup Assets Helper with the target folder
        assets = Assets(
            os.path.join(TEST_DIR, "test_assets_py"),
            force_clean=True,
            force_create=True,
        )

        # Copy the tree where we found reports
        assets.copy(assets.report_discover(os.path.join(TEST_DIR, "python")))

        # Create an index of those reports
        assets.create_html_index(
            os.path.join(assets.path, "index.html"),
            assets.path,
            assets.report_get_name(),
            logo_file=os.path.join(TEST_DIR, "logo.png"),
        )

    def _generic_test(self, input_dir, script_name, decription):
        # Prepare the test directory and environment
        test_dir = os.path.join(TEST_DIR, input_dir)

        # specify the yml file and override if it exists
        yml_file = f"{TEST_DIR}/scilens_py.yml"
        yml_file_override = f"{test_dir}/scilens_py.yml"
        if not os.path.exists(yml_file_override):
            yml_file_override = None

        if not only_compare:
            VAL_DIR = os.path.join(test_dir, "validation")
            REF_DIR = os.path.join(test_dir, "reference")

            os.environ[ENV_PYTHONPATH] = VAL_PY_DIR

            script_dir = pathlib.Path(script_name).parent
            print(script_name)
            modifications = {
                '"./output"': f'"{VAL_DIR}"',
                "thispath /": f'pathlib.Path("{EXAMPLES_DIR}").parent.resolve() / "{script_dir}" /',
            }
            clean_directory(VAL_DIR)
            run_modified_script(script_name, modifications)

            if not test_only:
                os.environ[ENV_PYTHONPATH] = REF_PY_DIR
                modifications = {'"./output"': f'"{REF_DIR}"'}
                clean_directory(REF_DIR)
                run_modified_script(script_name, modifications)

        runner = StandaloneTaskRunner(yml_file, config_override=yml_file_override)
        runner.config.report.description = decription
        results = runner.process(test_dir, origin_working_dir=ROOT_DIR)

        if results.error:
            raise Exception(results.error)
        else:
            assert (
                not results.processor_results.errors
                and not results.processor_results.warnings
            )

    def test_simulation(self):
        script = os.path.join(EXAMPLES_DIR, "sim/ex_simulation.py")
        description = (
            "Non regression Test for Seahowl for simulation case with duration 100s"
        )
        self._generic_test("python/simulation", script, description)

    def test_main(self):
        script = os.path.join(EXAMPLES_DIR, "sim/ex_main.py")
        description = (
            "Non regression Test for Seahowl for Onshore case with duration 100s"
        )
        self._generic_test("python/main", script, description)


if __name__ == "__main__":

    unittest.main()
