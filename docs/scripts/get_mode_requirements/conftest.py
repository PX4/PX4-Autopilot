"""
pytest configuration for get_mode_requirements tests.

Adds the get_mode_requirements/ directory to sys.path so that
  from get_mode_requirements import ...
works inside tests/ without any path manipulation in the test files.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
