#include <gtest/gtest.h>
#include <seahowl/hydro/morison.h>
#include "fixture_components.h"

#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;

// The fixture for testing
class Test_morison : public Fixture_components {
  protected:
    Test_morison() : Fixture_components() {
        ref_dir /= "test_morison/ref";
        test_dir /= "test_morison/test";
    }
};

TEST_F(Test_morison, MCF_Table) {

    // Setup TestFwDataSet
    TestFwDataSet test_dataset({.debug = false,
                                .reference_filepath = (ref_dir / "test_morison_MCF_Table.values.csv").generic_string(),
                                .test_filepath = (test_dir / "test_morison_MCF_Table.values.test.csv").generic_string(),
                                .dimensions = {"DNV_table Y"}});

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
    ASSERT_NEAR(DNV_table_Y0, Table_Y0, 0.01);

    test_dataset.testAddRow({Table_Y0});

    double DNV_table_X2 = 0.128;
    double DNV_table_Y2 = 2.060;
    node1.diameter = 0.128 * lambda;
    double Table_Y2 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y2, Table_Y2, 0.01);

    test_dataset.testAddRow({Table_Y2});

    double DNV_table_X4 = 0.322;
    double DNV_table_Y4 = 1.360;
    node1.diameter = 0.322 * lambda;
    double Table_Y4 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y4, Table_Y4, 0.01);

    test_dataset.testAddRow({Table_Y4});

    double DNV_table_X6 = 0.576;
    double DNV_table_Y6 = 0.655;
    node1.diameter = 0.576 * lambda;
    double Table_Y6 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y6, Table_Y6, 0.01);

    test_dataset.testAddRow({Table_Y6});

    double DNV_table_X8 = 0.864;
    double DNV_table_Y8 = 0.360;
    node1.diameter = 0.864 * lambda;
    double Table_Y8 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y8, Table_Y8, 0.01);

    test_dataset.testAddRow({Table_Y8});

    evaluate_test(test_dataset);
}

TEST_F(Test_morison, Cd_Table) {

    // Setup TestFwDataSet
    TestFwDataSet test_dataset({.debug = false,
                                .reference_filepath = (ref_dir / "test_morison_Cd_Table.values.csv").generic_string(),
                                .test_filepath = (test_dir / "test_morison_Cd_Table.values.test.csv").generic_string(),
                                .dimensions = {"Cd"}});

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
    ASSERT_NEAR(CD_ref, CD1, 0.01);

    test_dataset.testAddRow({CD1});
    evaluate_test(test_dataset);
}
