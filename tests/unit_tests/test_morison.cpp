#include "fixture_components.h"

#include <seahowl/hydro/morison.h>

#include <gtest/gtest.h>
#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;

// The fixture for testing
class TestMorison : public FixtureComponents {
  protected:
    TestMorison() : FixtureComponents() {
        ref_dir /= "test_morison/ref";
        test_dir /= "test_morison/test";
    }
};

TEST_F(TestMorison, MCF_Table) {
    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_morison_MCF_Table.values.csv").generic_string(),
                                       (test_dir / "test_morison_MCF_Table.values.test.csv").generic_string(),
                                       {"DNV_table Y"}});

    seahowl::hydro::MacCamyFuchsTable mytable = seahowl::hydro::MacCamyFuchsTable();
    mytable.wave_peak_period = 10.0;

    // general options
    seahowl::hydro::HydroCoefficients coefficients;
    auto node1 = seahowl::hydro::MorisonNode();
    coefficients.use_MacCamyFuchs_correction = true;

    node1.coefficients = coefficients;

    double lambda = 1.56 * mytable.wave_peak_period * mytable.wave_peak_period;

    double DNV_table_X0 = 0.0018;
    double DNV_table_Y0 = 2.003;

    node1.diameter = 0.0018 * lambda;
    double Table_Y0 = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.add_row_data({Table_Y0});

    double DNV_table_X2 = 0.128;
    double DNV_table_Y2 = 2.060;
    node1.diameter = 0.128 * lambda;
    double Table_Y2 = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.add_row_data({Table_Y2});

    double DNV_table_X4 = 0.322;
    double DNV_table_Y4 = 1.360;
    node1.diameter = 0.322 * lambda;
    double Table_Y4 = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.add_row_data({Table_Y4});

    double DNV_table_X6 = 0.576;
    double DNV_table_Y6 = 0.655;
    node1.diameter = 0.576 * lambda;
    double Table_Y6 = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.add_row_data({Table_Y6});

    double DNV_table_X8 = 0.864;
    double DNV_table_Y8 = 0.360;
    node1.diameter = 0.864 * lambda;
    double Table_Y8 = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.add_row_data({Table_Y8});

    EvaluateTest(test_dataset);
}

TEST_F(TestMorison, Cd_Table) {
    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_morison_Cd_Table.values.csv").generic_string(),
                                       (test_dir / "test_morison_Cd_Table.values.test.csv").generic_string(),
                                       {"Cd"}});

    seahowl::hydro::MacCamyFuchsTable mytable = seahowl::hydro::MacCamyFuchsTable();
    mytable.wave_peak_period = 10.0;

    // general options
    seahowl::hydro::HydroCoefficients coefficients;
    auto node1 = seahowl::hydro::MorisonNode();
    coefficients.use_Cd_correction = true;

    node1.coefficients = coefficients;

    double fluid_velocity = 0.08431;
    node1.diameter = 8.1;
    double CD1 = mytable.getCd(node1.diameter, mytable.wave_peak_period,
                               fluid_velocity);  // double diameter, double wave_period, double fluid_velocity

    double CD_ref = 1.045;

    test_dataset.add_row_data({CD1});
    EvaluateTest(test_dataset);
}
