#!/bin/bash

# Script to run C++ tests and generate HTML report from actual test results

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TEST_DIR="$SCRIPT_DIR"
HTML_OUTPUT_DIR="$PROJECT_ROOT/unit_tests/cpp_test_results"
HTML_FILE="$HTML_OUTPUT_DIR/cpp_test_results.html"

# Compiler settings
CXX=g++
CXXFLAGS="-Wall -Wextra -std=c++17 -I$PROJECT_ROOT/kin_cpp/third_party/blaze -I$PROJECT_ROOT/kin_cpp -I$PROJECT_ROOT/kin_cpp/third_party/googletest/googletest/include"
LDFLAGS="-L$PROJECT_ROOT/kin_cpp/third_party/googletest/build/lib -lgtest -lgtest_main -pthread -llapack -lblas"

# Source files
NODE_SRC="$PROJECT_ROOT/kin_cpp/primary_elements/node.cpp"
BEAM_SRC="$PROJECT_ROOT/kin_cpp/primary_elements/beam.cpp"
MISC_LINALG_SRC="$PROJECT_ROOT/kin_cpp/assets/misc_linalg.cpp"
TEST_NODE_SRC="$TEST_DIR/test_node.cpp"
TEST_BEAM_SRC="$TEST_DIR/test_beam.cpp"

# Build directory
BUILD_DIR="$TEST_DIR/build"
mkdir -p "$BUILD_DIR"

# Test executable
TEST_EXEC="$BUILD_DIR/test_runner"

echo -e "${YELLOW}Building C++ tests...${NC}"

# Compile test executable
$CXX $CXXFLAGS -c "$NODE_SRC" -o "$BUILD_DIR/node.o" 2>&1 | tee "$BUILD_DIR/build.log"
$CXX $CXXFLAGS -c "$BEAM_SRC" -o "$BUILD_DIR/beam.o" 2>&1 | tee -a "$BUILD_DIR/build.log"
$CXX $CXXFLAGS -c "$MISC_LINALG_SRC" -o "$BUILD_DIR/misc_linalg.o" 2>&1 | tee -a "$BUILD_DIR/build.log"
$CXX $CXXFLAGS -c "$TEST_NODE_SRC" -o "$BUILD_DIR/test_node.o" -I"$TEST_DIR" 2>&1 | tee -a "$BUILD_DIR/build.log"
$CXX $CXXFLAGS -c "$TEST_BEAM_SRC" -o "$BUILD_DIR/test_beam.o" -I"$TEST_DIR" 2>&1 | tee -a "$BUILD_DIR/build.log"

# Link
$CXX $CXXFLAGS -o "$TEST_EXEC" "$BUILD_DIR/node.o" "$BUILD_DIR/beam.o" "$BUILD_DIR/misc_linalg.o" "$BUILD_DIR/test_node.o" "$BUILD_DIR/test_beam.o" $LDFLAGS 2>&1 | tee -a "$BUILD_DIR/build.log"

if [ ! -f "$TEST_EXEC" ]; then
    echo -e "${RED}Build failed! Check $BUILD_DIR/build.log${NC}"
    exit 1
fi

echo -e "${GREEN}Build successful!${NC}"
echo -e "${YELLOW}Running tests...${NC}"

# Run tests with XML output
XML_OUTPUT="$BUILD_DIR/test_results.xml"
"$TEST_EXEC" --gtest_output=xml:"$XML_OUTPUT" 2>&1 | tee "$BUILD_DIR/test_output.txt"

# Parse XML and generate HTML
echo -e "${YELLOW}Generating HTML report...${NC}"

python3 << 'PYTHON_SCRIPT'
import xml.etree.ElementTree as ET
import json
import sys
from datetime import datetime

xml_file = sys.argv[1]
html_file = sys.argv[2]

try:
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    tests = []
    total_tests = int(root.attrib.get('tests', 0))
    failures = int(root.attrib.get('failures', 0))
    skipped = int(root.attrib.get('disabled', 0))
    passed = total_tests - failures - skipped
    
    for testcase in root.findall('.//testcase'):
        test_name = testcase.attrib.get('name', '')
        class_name = testcase.attrib.get('classname', '')
        full_name = f"{class_name}.{test_name}"
        duration_ms = float(testcase.attrib.get('time', 0)) * 1000
        
        # Check for failure
        failure = testcase.find('failure')
        if failure is not None:
            result = "Failed"
            details = failure.text or failure.attrib.get('message', '')
        else:
            result = "Passed"
            details = ""
        
        tests.append({
            'testId': full_name,
            'result': result,
            'duration': f"{duration_ms:.0f} ms",
            'details': details
        })
    
    # Sort tests by name
    tests.sort(key=lambda x: x['testId'])
    
    # Generate HTML
    html_template = '''<!DOCTYPE html>
<html>
  <head>
    <meta charset="utf-8"/>
    <title id="head-title">C++ Test Results</title>
      <style type="text/css">body {
  font-family: Helvetica, Arial, sans-serif;
  font-size: 12px;
  min-width: 800px;
  color: #999;
}

h1 {
  font-size: 24px;
  color: black;
}

h2 {
  font-size: 16px;
  color: black;
}

p {
  color: black;
}

a {
  color: #999;
}

table {
  border-collapse: collapse;
}

span.passed,
.passed .col-result {
  color: green;
}

span.skipped,
span.xfailed,
span.rerun,
.skipped .col-result,
.xfailed .col-result,
.rerun .col-result {
  color: orange;
}

span.error,
span.failed,
span.xpassed,
.error .col-result,
.failed .col-result,
.xpassed .col-result {
  color: red;
}

#results-table {
  border: 1px solid #e6e6e6;
  color: #999;
  font-size: 12px;
  width: 100%;
}
#results-table th,
#results-table td {
  padding: 5px;
  border: 1px solid #e6e6e6;
  text-align: left;
}
#results-table th {
  background-color: #f6f6f6;
  font-weight: bold;
  position: sticky;
  top: 0;
  z-index: 10;
}

.col-result {
  width: 110px;
}

.col-testId {
  width: 60%;
}

.col-duration {
  width: 80px;
}

.summary {
  margin: 20px 0;
}
.summary__data {
  display: inline-block;
  vertical-align: top;
  width: 100%;
}
.summary__spacer {
  display: inline-block;
  vertical-align: top;
  width: 49%;
}
.summary__reload {
  display: inline-block;
  vertical-align: top;
}
.summary__reload__button {
  padding: 10px;
  background-color: #fff3cd;
  border: 1px solid #ffc107;
  border-radius: 3px;
  color: #856404;
}
.summary__reload__button.hidden {
  display: none;
}
.controls {
  margin: 20px 0;
}
.filters {
  margin: 10px 0;
}
.filters input[type="checkbox"] {
  margin-right: 5px;
}
.run-count {
  font-weight: bold;
}
.filter {
  font-style: italic;
}

.sortable {
  cursor: pointer;
  user-select: none;
}
.sortable:hover {
  background-color: #e6e6e6;
}
.sortable::after {
  content: ' ↕';
  color: #ccc;
  font-size: 12px;
}

@media (prefers-color-scheme: dark) {
  body {
    background-color: #1e1e1e;
    color: #d4d4d4;
  }
  h1, h2, p {
    color: #d4d4d4;
  }
  #results-table th {
    background-color: #2d2d2d;
    color: #d4d4d4;
  }
  #results-table td {
    border-color: #3d3d3d;
  }
}
    </style>
  </head>
  <body>
    <h1 id="title">C++ Test Results</h1>
    <p>Report generated on <span id="report-date">{report_date}</span> by <a href="https://github.com/google/googletest">Google Test</a></p>

    <div class="summary">
      <div class="summary__data">
        <h2>Summary</h2>
        <div class="additional-summary prefix">
        </div>
        <p class="run-count" id="run-count">{total_tests} tests took 00:00:00.</p>
        <p class="filter">(Un)check the boxes to filter the results.</p>
        <div class="summary__reload">
          <div class="summary__reload__button hidden" onclick="location.reload()">
            <div>There are still tests running. <br />Reload this page to get the latest results!</div>
          </div>
        </div>
        <div class="summary__spacer"></div>
        <div class="controls">
          <div class="filters">
            <input checked="true" class="filter" name="filter_checkbox" type="checkbox" data-test-result="failed" />
            <span class="failed">{failures} Failed,</span>
            <input checked="true" class="filter" name="filter_checkbox" type="checkbox" data-test-result="passed" />
            <span class="passed">{passed} Passed,</span>
            <input checked="true" class="filter" name="filter_checkbox" type="checkbox" data-test-result="skipped" />
            <span class="skipped">{skipped} Skipped</span>
          </div>
        </div>
      </div>
    </div>

    <table id="results-table">
      <thead>
        <tr>
          <th class="sortable col-result" data-column-type="result">Result</th>
          <th class="sortable col-testId" data-column-type="testId">Test</th>
          <th class="sortable col-duration" data-column-type="duration">Duration</th>
        </tr>
      </thead>
      <tbody id="results-table-body">
        <!-- Test results will be inserted here by JavaScript -->
      </tbody>
    </table>

    <script>
      // Test data from actual test run
      const testData = {{
        tests: {tests_json}
      }};

      // Initialize
      let sortColumn = sessionStorage.getItem('sortColumn') || 'testId';
      let sortAscending = JSON.parse(sessionStorage.getItem('sortAscending')) || true;
      let visibleFilters = {{
        passed: true,
        failed: true,
        skipped: true
      }};

      // Set report date
      document.getElementById('report-date').textContent = '{report_date}';

      // Filter tests
      function filterTests() {{
        const checkboxes = document.querySelectorAll('input[name="filter_checkbox"]');
        checkboxes.forEach(checkbox => {{
          const resultType = checkbox.dataset.testResult;
          visibleFilters[resultType] = checkbox.checked;
        }});
        renderTests();
      }}

      // Sort tests
      function sortTests(column) {{
        if (sortColumn === column) {{
          sortAscending = !sortAscending;
        }} else {{
          sortColumn = column;
          sortAscending = true;
        }}
        sessionStorage.setItem('sortColumn', sortColumn);
        sessionStorage.setItem('sortAscending', sortAscending);
        renderTests();
      }}

      // Render tests
      function renderTests() {{
        const tbody = document.getElementById('results-table-body');
        tbody.innerHTML = '';

        // Filter and sort tests
        let filteredTests = testData.tests.filter(test => {{
          const resultLower = test.result.toLowerCase();
          if (resultLower === 'passed') return visibleFilters.passed;
          if (resultLower === 'failed') return visibleFilters.failed;
          if (resultLower === 'skipped') return visibleFilters.skipped;
          return true;
        }});

        // Sort tests
        filteredTests.sort((a, b) => {{
          let aVal, bVal;
          if (sortColumn === 'testId') {{
            aVal = a.testId;
            bVal = b.testId;
          }} else if (sortColumn === 'duration') {{
            aVal = parseFloat(a.duration);
            bVal = parseFloat(b.duration);
          }} else {{
            aVal = a.result;
            bVal = b.result;
          }}
          
          if (aVal < bVal) return sortAscending ? -1 : 1;
          if (aVal > bVal) return sortAscending ? 1 : -1;
          return 0;
        }});

        // Render test rows
        filteredTests.forEach(test => {{
          const row = tbody.insertRow();
          
          const resultCell = row.insertCell(0);
          resultCell.className = 'col-result ' + test.result.toLowerCase();
          resultCell.textContent = test.result;
          
          const testIdCell = row.insertCell(1);
          testIdCell.className = 'col-testId';
          testIdCell.textContent = test.testId;
          
          const durationCell = row.insertCell(2);
          durationCell.className = 'col-duration';
          durationCell.textContent = test.duration;
        }});

        // Update summary
        const passedCount = testData.tests.filter(t => t.result.toLowerCase() === 'passed').length;
        const failedCount = testData.tests.filter(t => t.result.toLowerCase() === 'failed').length;
        const skippedCount = testData.tests.filter(t => t.result.toLowerCase() === 'skipped').length;
        document.querySelector('.passed').textContent = passedCount + ' Passed,';
        document.querySelector('.failed').textContent = failedCount + ' Failed,';
        document.querySelector('.skipped').textContent = skippedCount + ' Skipped';
      }}

      // Add event listeners
      document.querySelectorAll('input[name="filter_checkbox"]').forEach(checkbox => {{
        checkbox.addEventListener('change', filterTests);
      }});

      document.querySelectorAll('.sortable').forEach(header => {{
        header.addEventListener('click', () => {{
          sortTests(header.dataset.columnType);
        }});
      }});

      // Initial render
      renderTests();
    </script>
  </footer>
</html>'''
    
    # Escape details for JSON
    for test in tests:
        test['details'] = json.dumps(test['details'])[1:-1]  # Remove quotes, keep escaped
    
    tests_json = json.dumps(tests, indent=10)
    report_date = datetime.now().strftime('%d-%b-%Y at %H:%M:%S')
    
    html_content = html_template.format(
        report_date=report_date,
        total_tests=total_tests,
        failures=failures,
        passed=passed,
        skipped=skipped,
        tests_json=tests_json
    )
    
    with open(html_file, 'w') as f:
        f.write(html_content)
    
    print(f"HTML report generated: {html_file}")
    print(f"Total: {total_tests}, Passed: {passed}, Failed: {failures}, Skipped: {skipped}")

except Exception as e:
    print(f"Error generating HTML: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
PYTHON_SCRIPT

python3 "$BUILD_DIR/test_results.xml" "$HTML_FILE"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}HTML report generated successfully!${NC}"
    echo -e "${GREEN}Open: $HTML_FILE${NC}"
else
    echo -e "${RED}Failed to generate HTML report${NC}"
    exit 1
fi

