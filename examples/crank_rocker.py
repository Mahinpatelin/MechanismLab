from mechanism_lab import FourBar
from mechanism_lab.report import export_study


linkage = FourBar(ground=100, crank=35, coupler=110, rocker=80)
export_study(linkage, "results/crank-rocker")
