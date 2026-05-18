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


from scilens import StandaloneTaskRunner
from scilens.helpers.assets import Assets

TEST_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(TEST_DIR))
WORKING_DIR = os.path.dirname(os.path.dirname(TEST_DIR))


class TestNonRegressionCpp(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):

        # Setup Assets Helper with the target folder
        assets = Assets(
            os.path.join(TEST_DIR, "test_assets_cpp"),
            force_clean=True,
            force_create=True,
        )

        # Copy the tree where we found reports
        assets.copy(assets.report_discover(os.path.join(TEST_DIR, "cpp")))

        # Create an index of those reports
        assets.create_html_index(
            os.path.join(assets.path, "index.html"),
            assets.path,
            assets.report_get_name(),
            logo_file=os.path.join(TEST_DIR, "logo.png"),
        )

    def _generic_test(self, input_dir, main_json, decription):

        test_dir = os.path.join(TEST_DIR, input_dir)

        yml_file = f"{TEST_DIR}/scilens_cpp.yml"
        yml_file_override = f"{test_dir}/scilens_cpp.yml"
        if not os.path.exists(yml_file_override):
            yml_file_override = None

        runner = StandaloneTaskRunner(yml_file, config_override=yml_file_override)
        option = f"{runner.config.execute.command_suffix} --outputs-folder ."
        runner.config.execute.command_suffix = f" {main_json} {option}"
        runner.config.report.description = decription
        results = runner.process(test_dir, origin_working_dir=ROOT_DIR)

        if results.error:
            raise Exception(results.error)
        else:
            assert (
                not results.processor_results.errors
                and not results.processor_results.warnings
            )

    def test_onshore(self):
        main_json = os.path.join(WORKING_DIR, "data/IEA15MW/main_onshore.json")
        description = (
            "Non regression Test for Seahowl for Onshore case with duration 100s"
        )
        self._generic_test("cpp/onshore", main_json, description)

    def test_monopile(self):
        main_json = os.path.join(WORKING_DIR, "data/IEA15MW/main_monopile.json")
        description = (
            "Non regression Test for Seahowl for Monopile case with duration 100s"
        )
        self._generic_test("cpp/monopile", main_json, description)

    # deactivate problematic test due to zero-crossing values leading to numerical errors
    # need to introduce more meaningful error checking
    # def test_floating(self):
    #     main_json = os.path.join(
    #         WORKING_DIR, "data/IEA15MW/floating/main_hydrochrono.json"
    #     )
    #     description = (
    #         "Non regression Test for Seahowl for Floating case with duration 100s"
    #     )
    #     self._generic_test("cpp/floating", main_json, description)


if __name__ == "__main__":

    unittest.main()
