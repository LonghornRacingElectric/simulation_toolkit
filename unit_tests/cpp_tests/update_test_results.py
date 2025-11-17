#!/usr/bin/env python3
"""
Script to run C++ unit tests and update the HTML file with actual results.
Run this script before opening the HTML file to see current test results.
"""

import subprocess
import json
import os
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

# Get script directory
SCRIPT_DIR = Path(__file__).parent.absolute()
PROJECT_ROOT = SCRIPT_DIR.parent.parent
TEST_DIR = SCRIPT_DIR
HTML_OUTPUT_DIR = PROJECT_ROOT / "unit_tests" / "cpp_test_results"
HTML_FILE = HTML_OUTPUT_DIR / "cpp_test_results.html"
BUILD_DIR = TEST_DIR / "build"
BUILD_DIR.mkdir(exist_ok=True)

# Compiler settings
CXX = "g++"
# Compile from kin_cpp directory like the makefile does
KIN_CPP_DIR = PROJECT_ROOT / "kin_cpp"
CXXFLAGS = [
    "-Wall", "-Wextra", "-std=c++17",
    "-Ithird_party/blaze",  # Relative to kin_cpp directory
    f"-I{PROJECT_ROOT}/kin_cpp/third_party/googletest/googletest/include",
    f"-I{PROJECT_ROOT}/unit_tests/cpp_tests"
]
# Try to find lapack/blas, but make them optional
LDFLAGS_BASE = [
    "-L" + str(PROJECT_ROOT / "kin_cpp" / "third_party" / "googletest" / "build" / "lib"),
    "-lgtest", "-pthread"
]

# Add lapack/blas - try to find them dynamically
# First try pkg-config, then fall back to direct paths
try:
    result = subprocess.run(["pkg-config", "--libs", "lapack", "blas"], 
                           capture_output=True, text=True, timeout=2)
    if result.returncode == 0:
        libs = result.stdout.strip().split()
        LDFLAGS_BASE.extend(libs)
    else:
        # Try with full paths
        LDFLAGS_BASE.extend([
            "/usr/lib/x86_64-linux-gnu/liblapack.so.3",
            "/usr/lib/x86_64-linux-gnu/libblas.so.3"
        ])
except:
    # Fallback: try direct paths
    LDFLAGS_BASE.extend([
        "/usr/lib/x86_64-linux-gnu/liblapack.so.3",
        "/usr/lib/x86_64-linux-gnu/libblas.so.3"
    ])

LDFLAGS = LDFLAGS_BASE

# Source files (relative to kin_cpp directory for compilation)
NODE_SRC = "primary_elements/node.cpp"
BEAM_SRC = "primary_elements/beam.cpp"
MISC_LINALG_SRC = "assets/misc_linalg.cpp"
WISHBONE_SRC = "secondary_elements/wishbone.cpp"
PUSHROD_SRC = "secondary_elements/pushrod.cpp"
KINGPIN_SRC = "secondary_elements/kingpin.cpp"
TEST_NODE_SRC = PROJECT_ROOT / "unit_tests" / "cpp_tests" / "test_node.cpp"
TEST_BEAM_SRC = PROJECT_ROOT / "unit_tests" / "cpp_tests" / "test_beam.cpp"
TEST_WISHBONE_SRC = PROJECT_ROOT / "unit_tests" / "cpp_tests" / "test_wishbone.cpp"
TEST_PUSHROD_SRC = PROJECT_ROOT / "unit_tests" / "cpp_tests" / "test_pushrod.cpp"
TEST_KINGPIN_SRC = PROJECT_ROOT / "unit_tests" / "cpp_tests" / "test_kingpin.cpp"

TEST_EXEC = BUILD_DIR / "test_runner"
XML_OUTPUT = BUILD_DIR / "test_results.xml"

def build_tests():
    """Compile the test executable."""
    print("Building C++ tests...")
    
    objects = []
    # Compile from kin_cpp directory (like makefile does)
    kin_cpp_sources = [
        (NODE_SRC, "node.o"),
        (BEAM_SRC, "beam.o"),
        (MISC_LINALG_SRC, "misc_linalg.o"),
        (WISHBONE_SRC, "wishbone.o"),
        (PUSHROD_SRC, "pushrod.o"),
        (KINGPIN_SRC, "kingpin.o")
    ]
    
    # Compile all kin_cpp sources from kin_cpp directory
    for src, obj_name in kin_cpp_sources:
        obj_file = BUILD_DIR / obj_name
        cmd = [CXX] + CXXFLAGS + ["-c", src, "-o", str(obj_file.absolute())]
        result = subprocess.run(cmd, cwd=str(KIN_CPP_DIR), capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error compiling {src}:")
            print(result.stderr)
            return False
        objects.append(str(obj_file.absolute()))
    
    # Compile test files from project root with proper include paths
    test_sources = [
        (TEST_NODE_SRC, "test_node.o"),
        (TEST_BEAM_SRC, "test_beam.o"),
        (TEST_WISHBONE_SRC, "test_wishbone.o"),
        (TEST_PUSHROD_SRC, "test_pushrod.o"),
        (TEST_KINGPIN_SRC, "test_kingpin.o")
    ]
    
    for src, obj_name in test_sources:
        obj_file = BUILD_DIR / obj_name
        # Test files need kin_cpp and blaze include paths
        test_cxxflags = [
            "-Wall", "-Wextra", "-std=c++17",
            f"-I{PROJECT_ROOT}/kin_cpp/third_party/blaze",  # For <blaze/Math.h>
            f"-I{PROJECT_ROOT}/kin_cpp",
            f"-I{PROJECT_ROOT}/kin_cpp/third_party/googletest/googletest/include",
            f"-I{PROJECT_ROOT}/unit_tests/cpp_tests"
        ]
        cmd = [CXX] + test_cxxflags + ["-c", str(src), "-o", str(obj_file.absolute())]
        result = subprocess.run(cmd, cwd=str(PROJECT_ROOT), capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error compiling {src}:")
            print(result.stderr)
            return False
        objects.append(str(obj_file.absolute()))
    
    # Link (from project root)
    cmd = [CXX] + ["-std=c++17"] + ["-o", str(TEST_EXEC)] + objects + LDFLAGS
    result = subprocess.run(cmd, cwd=str(PROJECT_ROOT), capture_output=True, text=True)
    if result.returncode != 0:
        print("Error linking:")
        print(result.stderr)
        return False
    
    print("Build successful!")
    return True

def run_tests():
    """Run the tests and return XML output."""
    print("Running tests...")
    
    if not TEST_EXEC.exists():
        print("Test executable not found. Build failed.")
        return None
    
    # Run tests with XML output
    cmd = [str(TEST_EXEC), "--gtest_output=xml:" + str(XML_OUTPUT)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    print(result.stdout)
    if result.stderr:
        print(result.stderr)
    
    if not XML_OUTPUT.exists():
        print("Test XML output not generated.")
        return None
    
    return XML_OUTPUT

def parse_test_results(xml_file):
    """Parse Google Test XML output and return test data."""
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
            duration_seconds = float(testcase.attrib.get('time', 0))
            duration_ms = duration_seconds * 1000
            duration_us = duration_seconds * 1000000  # microseconds
            
            # Check for failure
            failure = testcase.find('failure')
            if failure is not None:
                result = "Failed"
                details = (failure.text or failure.attrib.get('message', '')).strip()
                # Escape for JSON
                details = details.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')
            else:
                result = "Passed"
                details = ""
            
            # Format duration: show microseconds if < 1ms, otherwise show ms with 1 decimal
            if duration_ms < 1.0:
                duration_str = f"{duration_us:.0f} μs"
            else:
                duration_str = f"{duration_ms:.1f} ms"
            
            tests.append({
                'testId': full_name,
                'result': result,
                'duration': duration_str,
                'details': details
            })
        
        # Sort tests by name
        tests.sort(key=lambda x: x['testId'])
        
        return {
            'tests': tests,
            'total': total_tests,
            'passed': passed,
            'failed': failures,
            'skipped': skipped
        }
    except Exception as e:
        print(f"Error parsing XML: {e}")
        return None

def generate_html(test_data):
    """Generate HTML file with embedded test results."""
    print("Generating HTML report...")
    
    report_date = datetime.now().strftime('%d-%b-%Y at %H:%M:%S')
    tests_json = json.dumps(test_data['tests'], indent=10)
    
    html_content = f'''<!DOCTYPE html>
<html>
  <head>
    <meta charset="utf-8"/>
    <title id="head-title">C++ Test Results</title>
      <style type="text/css">body {{
  font-family: Helvetica, Arial, sans-serif;
  font-size: 12px;
  min-width: 800px;
  color: #999;
}}

h1 {{
  font-size: 24px;
  color: black;
}}

h2 {{
  font-size: 16px;
  color: black;
}}

p {{
  color: black;
}}

a {{
  color: #999;
}}

table {{
  border-collapse: collapse;
}}

span.passed,
.passed .col-result {{
  color: green;
}}

span.skipped,
span.xfailed,
span.rerun,
.skipped .col-result,
.xfailed .col-result,
.rerun .col-result {{
  color: orange;
}}

span.error,
span.failed,
span.xpassed,
.error .col-result,
.failed .col-result,
.xpassed .col-result {{
  color: red;
}}

#results-table {{
  border: 1px solid #e6e6e6;
  color: #999;
  font-size: 12px;
  width: 100%;
}}
#results-table th,
#results-table td {{
  padding: 5px;
  border: 1px solid #e6e6e6;
  text-align: left;
}}
#results-table th {{
  background-color: #f6f6f6;
  font-weight: bold;
  position: sticky;
  top: 0;
  z-index: 10;
}}

.col-result {{
  width: 110px;
}}

.col-testId {{
  width: 60%;
}}

.col-duration {{
  width: 80px;
}}

.summary {{
  margin: 20px 0;
}}
.summary__data {{
  display: inline-block;
  vertical-align: top;
  width: 100%;
}}
.summary__spacer {{
  display: inline-block;
  vertical-align: top;
  width: 49%;
}}
.summary__reload {{
  display: inline-block;
  vertical-align: top;
}}
.summary__reload__button {{
  padding: 10px;
  background-color: #fff3cd;
  border: 1px solid #ffc107;
  border-radius: 3px;
  color: #856404;
}}
.summary__reload__button.hidden {{
  display: none;
}}
.controls {{
  margin: 20px 0;
}}
.filters {{
  margin: 10px 0;
}}
.filters input[type="checkbox"] {{
  margin-right: 5px;
}}
.run-count {{
  font-weight: bold;
}}
.filter {{
  font-style: italic;
}}

.sortable {{
  cursor: pointer;
  user-select: none;
}}
.sortable:hover {{
  background-color: #e6e6e6;
}}
.sortable::after {{
  content: ' ↕';
  color: #ccc;
  font-size: 12px;
}}

@media (prefers-color-scheme: dark) {{
  body {{
    background-color: #1e1e1e;
    color: #d4d4d4;
  }}
  h1, h2, p {{
    color: #d4d4d4;
  }}
  #results-table th {{
    background-color: #2d2d2d;
    color: #d4d4d4;
  }}
  #results-table td {{
    border-color: #3d3d3d;
  }}
}}
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
        <p class="run-count" id="run-count">{test_data['total']} tests took 00:00:00.</p>
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
            <span class="failed">{test_data['failed']} Failed,</span>
            <input checked="true" class="filter" name="filter_checkbox" type="checkbox" data-test-result="passed" />
            <span class="passed">{test_data['passed']} Passed,</span>
            <input checked="true" class="filter" name="filter_checkbox" type="checkbox" data-test-result="skipped" />
            <span class="skipped">{test_data['skipped']} Skipped</span>
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
      // Test data from actual test run - embedded directly in HTML
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
    
    HTML_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    with open(HTML_FILE, 'w') as f:
        f.write(html_content)
    
    print(f"\n✓ HTML report generated: {HTML_FILE}")
    print(f"  Total: {test_data['total']}, Passed: {test_data['passed']}, Failed: {test_data['failed']}, Skipped: {test_data['skipped']}")
    return True

def main():
    """Main function."""
    print("=" * 60)
    print("C++ Test Results Generator")
    print("=" * 60)
    
    if not build_tests():
        print("\n✗ Build failed!")
        return 1
    
    xml_file = run_tests()
    if not xml_file:
        print("\n✗ Tests failed to run!")
        return 1
    
    test_data = parse_test_results(xml_file)
    if not test_data:
        print("\n✗ Failed to parse test results!")
        return 1
    
    if generate_html(test_data):
        print("\n✓ Success! Open the HTML file to view results.")
        return 0
    
    return 1

if __name__ == "__main__":
    sys.exit(main())

