"""Locate immutable historical sources used by compatibility regression tests."""
import os
from pathlib import Path


def baseline_path(variable, relative):
    """An explicit external source overrides the checked-in regression fixture."""
    supplied = os.environ.get(variable)
    path = Path(supplied) if supplied else Path(__file__).parent/'baselines'/relative
    if not path.exists():
        raise FileNotFoundError(f'{variable} regression source is missing: {path}')
    return path
