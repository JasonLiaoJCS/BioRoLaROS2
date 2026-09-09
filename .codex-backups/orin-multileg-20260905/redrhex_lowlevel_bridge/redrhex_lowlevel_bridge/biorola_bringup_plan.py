"""Retired legacy bringup-plan generator.

The former generator emitted an incomplete hardware-enablement sequence that
predated immutable policy packages, the final motor arbiter, L1 degraded-mode
contracts, and the current power-command freshness checks. Keeping those
snippets callable would provide a second, unsafe deployment path.
"""

from __future__ import annotations

import argparse


CONTINUATION_MANUAL = (
    "/home/jetson/rinbo_ros_ws/docs/redrhex_sim2real_after_rslip.md"
)
QUICKSTART_MANUAL = (
    "/home/jetson/rinbo_ros_ws/docs/redrhex_sim2real_quickstart.md"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Deprecated: the generated bringup plan was removed because it no "
            "longer satisfies the RedRHex hardware safety contract."
        )
    )
    parser.add_argument(
        "--show-manuals",
        action="store_true",
        help="Print the maintained R-Slip continuation and Sim2Real manuals.",
    )
    return parser


def retired_message() -> str:
    return "\n".join(
        (
            "ERROR: rinbo_bringup_plan/biorola_bringup_plan is retired.",
            "It must not be used to enable hardware or deploy a policy.",
            f"R-Slip continuation: {CONTINUATION_MANUAL}",
            f"Full Sim2Real quickstart: {QUICKSTART_MANUAL}",
        )
    )


def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    print(retired_message())
    if args.show_manuals:
        return
    raise SystemExit(2)


if __name__ == "__main__":
    main()
