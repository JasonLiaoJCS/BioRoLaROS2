"""Print the canonical low-level bridge semantic SHA256."""

from __future__ import annotations

import argparse

from .golden_policy import (
    bridge_config_sha256,
    bridge_params_with_disabled_legs,
    load_bridge_ros_params,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compute policy.expected_bridge_config_sha256. Temporary "
            "allow_enable overrides are deliberately excluded."
        )
    )
    parser.add_argument("--bridge-config", required=True)
    parser.add_argument(
        "--disabled-legs",
        default=None,
        help="Optional effective comma-separated disabled-leg override, e.g. L1",
    )
    args = parser.parse_args(argv)
    disabled_legs = (
        None
        if args.disabled_legs is None
        else [item.strip() for item in args.disabled_legs.split(",") if item.strip()]
    )
    params = bridge_params_with_disabled_legs(
        load_bridge_ros_params(args.bridge_config), disabled_legs
    )
    print(bridge_config_sha256(params))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
