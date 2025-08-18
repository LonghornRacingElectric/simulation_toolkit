from pypdf import PdfReader, PdfWriter

# List of (filepath, title) pairs
pdfs = [
    ("/home/rhorvath/Documents/LHR_design_reports/LHR_Design_Workflows.pdf", "1. Design Workflows and Studies"),
    ("/home/rhorvath/Documents/Github/simulation_toolkit/simulations/qss/qss_outputs/qss_report.pdf", "2. QSS Metrics"),
    ("/home/rhorvath/Documents/LHR_design_reports/Tire_Metrics.pdf", "3. Tire Metrics"),
    ("/home/rhorvath/Documents/Github/simulation_toolkit/simulations/kin/kin_outputs/kin_report.pdf", "4. Kin Metrics"),
]

writer = PdfWriter()
page_index = 0  # Track page offset for bookmark destinations

for filepath, title in pdfs:
    reader = PdfReader(filepath)
    num_pages = len(reader.pages)

    # Add pages to writer
    for page in reader.pages:
        writer.add_page(page)

    # Add top-level bookmark pointing to first page of this PDF section
    writer.add_outline_item(title, page_index)

    # Update page index offset
    page_index += num_pages

# Save output
with open("VMOD_design_report.pdf", "wb") as f:
    writer.write(f)
