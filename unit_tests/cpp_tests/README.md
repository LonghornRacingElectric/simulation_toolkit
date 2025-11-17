# C++ Test Results Generator

## Usage

To generate the HTML test results file from actual C++ test runs:

**From the project root directory:**
```bash
python3 unit_tests/cpp_tests/update_test_results.py
```

**Or from anywhere (using absolute path):**
```bash
cd /home/keertan/simulation_toolkit
python3 unit_tests/cpp_tests/update_test_results.py
```

**Or if you're in the cpp_tests directory:**
```bash
cd unit_tests/cpp_tests
python3 update_test_results.py
```

**Note:** The script should be run from the project root (`simulation_toolkit`) directory, as it uses relative paths to find source files.

This script will:
1. Compile the C++ unit tests
2. Run the tests
3. Parse the Google Test XML output
4. Generate/update `unit_tests/cpp_test_results/cpp_test_results.html` with actual results

## Requirements

- g++ compiler
- Google Test library (libgtest) built and available
- Python 3
- Blaze library in `kin_cpp/third_party/blaze`

## How it works

The script:
- Compiles `node.cpp`, `beam.cpp`, `misc_linalg.cpp`, and test files
- Links them with Google Test
- Runs the test executable with XML output
- Parses the XML to extract test results (pass/fail, duration, error messages)
- Generates an HTML file with the results embedded in JavaScript

When you open the HTML file, it will show the actual test results from the last run.

## Viewing Results

**Option 1: Open from Windows Explorer**
Navigate to:
```
\\wsl.localhost\Ubuntu\home\keertan\simulation_toolkit\unit_tests\cpp_test_results\cpp_test_results.html
```
Double-click the file to open it in your default browser.

**Option 2: Open from PowerShell/Command Prompt**
```powershell
Start-Process "\\wsl.localhost\Ubuntu\home\keertan\simulation_toolkit\unit_tests\cpp_test_results\cpp_test_results.html"
```

**Option 3: Open from WSL terminal**
```bash
# If you have a browser in WSL or want to copy the path:
wslview unit_tests/cpp_test_results/cpp_test_results.html
# Or just copy the path and paste it in Windows Explorer
```

**Option 4: Direct Windows path**
The file is located at:
```
\\wsl.localhost\Ubuntu\home\keertan\simulation_toolkit\unit_tests\cpp_test_results\cpp_test_results.html
```
You can paste this path directly into Windows Explorer's address bar or your browser's address bar.


