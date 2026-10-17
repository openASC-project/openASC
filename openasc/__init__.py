"""Compatibility entry point for the research tools under tools/prototype.

The engine implementation lives in the Rust workspace under crates/.
"""
from pathlib import Path

__path__ = [str(Path(__file__).resolve().parents[1] / 'tools/prototype/openasc')]
