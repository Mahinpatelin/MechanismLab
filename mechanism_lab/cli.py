from __future__ import annotations

import argparse

from .fourbar import FourBar
from .report import export_study


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze a planar four-bar linkage")
    parser.add_argument("--ground", type=float, default=100)
    parser.add_argument("--crank", type=float, default=35)
    parser.add_argument("--coupler", type=float, default=110)
    parser.add_argument("--rocker", type=float, default=80)
    parser.add_argument("--assembly", choices=("open", "crossed"), default="open")
    parser.add_argument("--samples", type=int, default=361)
    parser.add_argument("--output", default="results")
    args = parser.parse_args()
    linkage = FourBar(args.ground, args.crank, args.coupler, args.rocker, args.assembly)
    paths = export_study(linkage, args.output, args.samples)
    print("Generated:")
    for path in paths:
        print(f"  {path}")


if __name__ == "__main__":
    main()
