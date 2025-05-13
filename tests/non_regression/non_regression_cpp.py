import os
import unittest


from scilens import StandaloneTaskRunner
from scilens.helpers.assets import Assets

TEST_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(TEST_DIR))


class TestNonRegressionCpp(unittest.TestCase):

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

    def _generic_test(self, input_dir):

        test_dir = os.path.join(TEST_DIR, input_dir)
        runner = StandaloneTaskRunner(f"{TEST_DIR}/scilens_cpp.yml")
        results = runner.process(test_dir, origin_working_dir=ROOT_DIR)

        if results.error:
            raise Exception(results.error)
        else:
            assert (
                not results.processor_results.errors
                and not results.processor_results.warnings
            )

    def test_onshore(self):

        self._generic_test("cpp/onshore")

    def test_monopile(self):

        self._generic_test("cpp/monopile")


if __name__ == "__main__":

    unittest.main()
