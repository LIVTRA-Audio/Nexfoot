# SPDX-License-Identifier: Apache-2.0
"""Keep downloaded dependencies and generated build data outside the source tree."""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = Path(os.environ.get("NEXFOOT_WORKSPACE", str(ROOT.parent / ("." + ROOT.name + "-work")))).expanduser().resolve()
SDK = WORKSPACE / "cache/jl-sdk"
STAGE = WORKSPACE / "build/sdk"
BACKUPS = WORKSPACE / "backups"
