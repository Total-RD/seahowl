# Test README

This folder contains the tests for the **seahowl** project.

### Prerequisites

- python 3.11 or higher
- [pytest](https://docs.pytest.org)
- [SciLens](https://scilens.dev)

## Test Structure
- `tests/unit_test/`
    Contains unit tests.
    These tests validate the behavior of individual functions and modules.

- `tests/non_regression/`
    Contains non-regression tests.
    These tests ensure that existing features continue to work after code changes.

## Running the Tests

### unit tests
Use the following command to run unit tests:

```bash
./build/test_components
```

Use the following command to generate html report:

```bash
cd tests/unit_tests/data
scilens run --collect-depth 1 --export-html-add-index .
```

### non-regression tests
Use the following command the non-regression tests:

```bash
python -m unittest discover -v -s tests/non_regression -p "*.py"
```
The HTML reports are automatically generated in the following directories: `test_components/test_assets_cpp` and `test_components/test_assets_py`.

For more launch options, please refer to [unittest — Unit testing framework Python 3 documentation](https://docs.python.org/3/library/unittest.html)

## Best Practices

- Add a test for every new feature or bug fix.
- Update non-regression tests after major changes.
- Make sure all tests pass before committing changes.
