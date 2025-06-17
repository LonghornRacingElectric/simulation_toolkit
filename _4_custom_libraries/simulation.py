from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure
from typing import Sequence

import subprocess


class Simulation:
    def get_git_username(self):
        try:
            name = subprocess.check_output(
                ["git", "config", "user.email"], stderr=subprocess.DEVNULL
            ).decode().strip()
            return name if name else "Unknown"
        except Exception:
            return "Unknown"
    
    def get_git_name(self):
        try:
            name = subprocess.check_output(
                ["git", "config", "user.name"], stderr=subprocess.DEVNULL
            ).decode().strip()
            return name if name else "Unknown"
        except Exception:
            return "Unknown"

    def _generate_pdf(self, figs: Sequence[Figure], save_path: str) -> Figure:
        p = PdfPages(save_path)

        for page in figs:
            page.savefig(p, format="pdf")

        p.close()
