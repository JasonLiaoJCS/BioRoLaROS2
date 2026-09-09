"""Canonical physical-leg parsing for explicit degraded hardware tests."""

from __future__ import annotations


# Policy order is RF, RM, RR, LF, LM, LR; physical names are kept explicit so
# L1 never gets confused with policy index 0 (R1).
POLICY_PHYSICAL_LEG_NAMES = ("R1", "R2", "R3", "L1", "L2", "L3")


def normalize_disabled_legs(values, max_disabled_legs: int = 1) -> list[str]:
    max_disabled_legs = int(max_disabled_legs)
    if max_disabled_legs < 1 or max_disabled_legs > 5:
        raise ValueError("hardware.max_disabled_legs must be in [1, 5]")
    normalized: list[str] = []
    for value in values:
        name = str(value).strip().upper()
        if name not in POLICY_PHYSICAL_LEG_NAMES:
            raise ValueError(
                f"Unknown hardware.disabled_legs entry {value!r}; expected L1,L2,L3,R1,R2,R3"
            )
        if name in normalized:
            raise ValueError(f"Duplicate hardware.disabled_legs entry {name!r}")
        normalized.append(name)
    if len(normalized) > max_disabled_legs:
        raise ValueError(
            f"hardware.disabled_legs has {len(normalized)} entries, "
            f"exceeding hardware.max_disabled_legs={max_disabled_legs}"
        )
    return normalized


def policy_indices_for_disabled_legs(names: list[str]) -> list[int]:
    return [POLICY_PHYSICAL_LEG_NAMES.index(name) for name in names]
