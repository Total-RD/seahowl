# Test framework - dataset  Compare

Class for test dataset manipulation.

**Features**

- multiple test data fullfillement methods:
  - by dataset
  - by iterations row values
  - by iterations lambdas functions
- multiple comparison methods:
  - between a ref dataset and the test dataset, calculate the error dataset for individual assert
  - between a ref dataset and the test dataset, calculate the error dataset and check each value against an error threshold for a global assert
- debug traces mechanism
- read csv reference dataset values
- write csv test values => for easy change of reference values by test values

## Usage

### Case: test data iteration lambda functions

```cpp

// Setup TestFwDataSet
TestFwDataSet test_dataset({
    .debug = true,
    .reference_filepath = "../ref_with_headers.csv",
    .test_filepath = "..../test_with_headers.csv",
    .dimensions={"dim1", "dim2", "dim3"},
    .test_functions = {
        []() -> std::vector<double> { /* PUT BUSINESS LOGIC HERE WITH CAST TO double if needed */ return {1.0, 2.0}; },
        []() -> std::vector<double> { /* PUT BUSINESS LOGIC HERE WITH CAST TO double if needed */ return {6.0}; },
    }
});

// Simulate iterations
test_dataset.testAdd();
test_dataset.testAdd();
test_dataset.testAdd();

// Exemple for detecting absolute errors
auto [error, result] = test_dataset.differencesAbsErrCount(reference_filepath, 1e-4);
if (error.empty()) {

    // DO ASSERT STATEMENT HERE WITH result
    std::cout << "SUCCESS : nb errors = " << to_string(result) << std::endl;

} else {

    // DO ERROR MANAGEMENT HERE WITH error
    std::cout << "ERROR : " << error << std::endl;
}
```


### Case: test data iteration values

```cpp
// TestFwDataSet
TestFwDataSet::Options options;
options.debug = true;
options.reference_filepath = "../ref.csv";

TestFwDataSet test_dataset(options);

// populate test matrix row by row
test_dataset.addRow({0.12 , 0.13, 0.14});
test_dataset.addRow({0.13 , 0.14, 0.15});
test_dataset.addRow({0.14 , 0.15, 0.16});

// Exemple for detecting absolute errors
auto [error, result] = test_dataset.differencesAbsErrCount(reference_filepath, 1e-4);
if (error.empty()) {

    // DO ASSERT STATEMENT HERE WITH result
    std::cout << "SUCCESS : nb errors = " << to_string(result) << std::endl;

} else {

    // DO ERROR MANAGEMENT HERE WITH error
    std::cout << "ERROR : " << error << std::endl;
}
```
