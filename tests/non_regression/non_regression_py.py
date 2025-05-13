import os
import unittest
import subprocess

from scilens import StandaloneTaskRunner
from scilens.helpers.assets import Assets

TEST_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(TEST_DIR))

VAL_PY_DIR = os.path.join(os.getcwd(), "build")
REF_PY_DIR = os.path.join(os.getcwd(), "install/bin")

ENV_PYTHONPATH = "PYTHONPATH"

test_only = True


class TestNonRegressionPy(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):

        # Setup Assets Helper with the target folder
        assets = Assets(
            os.path.join(TEST_DIR, "test_assets"), force_clean=True, force_create=True
        )

        # Copy the tree where we found reports
        assets.copy(assets.report_discover(TEST_DIR))

        # Create an index of those reports
        assets.create_html_index(
            os.path.join(assets.path, "index.html"),
            assets.path,
            assets.report_get_name(),
            logo_file=os.path.join(TEST_DIR, "logo.png"),
        )

    def _generic_test(self, input_dir, script_name):

        test_dir = os.path.join(TEST_DIR, input_dir)

        outpu_dir = os.path.join(test_dir, "output")
        if os.path.exists(outpu_dir):
            os.rmdir(outpu_dir)

        VAL_DIR = os.path.join(test_dir, "validation")
        REF_DIR = os.path.join(test_dir, "reference")

        os.environ[ENV_PYTHONPATH] = VAL_PY_DIR
        subprocess.run(["python3", script_name], check=True, cwd=VAL_DIR)

        if not test_only:
            os.environ[ENV_PYTHONPATH] = REF_PY_DIR
            subprocess.run(["python3", script_name], check=True, cwd=REF_DIR)

        runner = StandaloneTaskRunner(f"{TEST_DIR}/scilens_py.yml")
        results = runner.process(test_dir, origin_working_dir=ROOT_DIR)

        if results.error:
            raise Exception(results.error)
        else:
            assert (
                not results.processor_results.errors
                and not results.processor_results.warnings
            )

    def test_simulation(self):
        self._generic_test("python/simulation", "ex_simulation.py")

    @unittest.skip("Not yet implemented")
    def test_mooring(self):
        self._generic_test("python/mooring", "ex_mooring.py")

    @unittest.skip("Not yet implemented")
    def test_system(self):
        self._generic_test("python/system", "ex_system.py")

    def test_main(self):
        self._generic_test("python/main", "ex_main.py")

    def test_load_sims(self):
        self._generic_test("python/load_sims", "load_sims.py")


if __name__ == "__main__":

    unittest.main()
