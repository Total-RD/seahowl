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

The HTML reports are automatically generated : `tests/unit_tests/data/index.html`

### non-regression tests
Use the following command the non-regression tests:

```bash
python -m unittest discover -v -s tests/non_regression -p "*.py"
```
The HTML reports are automatically generated in the following directories: `test_components/test_assets_cpp/index.html` and `test_components/test_assets_py/index.html`.

For more launch options, please refer to [unittest — Unit testing framework Python 3 documentation](https://docs.python.org/3/library/unittest.html)

## Best Practices

- Add a test for every new feature or bug fix.
- Update non-regression tests after major changes.
- Make sure all tests pass before committing changes.

## Visual Studio Code

If you use VS Code, please be aware that:

- the workspace configuration is defined in `.vscode/settings.json`
- useful extensions are necessary

These elements can be installed using the following helper scripts.

### Linux

```bash
cd scripts
./setup_vscode_env.sh
```

### Windows

```bash
cd scripts
./setup_vscode_env.bat
```
