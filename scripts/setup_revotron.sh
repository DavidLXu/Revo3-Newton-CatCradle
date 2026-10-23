#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
revision=bf114a1b421ca00b80f517f5e2b074591940ddc0
destination=assets/revotron-description
if [[ -f "$destination/source.json" ]]; then
  .venv/bin/python - "$destination" "$revision" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]);manifest=json.loads((root/'source.json').read_text())
if manifest['revision']!=sys.argv[2]:raise SystemExit('Existing RevoTron assets use another revision; preserved.')
if not (root/manifest['urdf']).is_file():raise SystemExit('RevoTron URDF is missing.')
print('Pinned RevoTron assets already available.')
PY
  exit 0
fi
if [[ -e "$destination" ]]; then
  echo 'Existing unrecognized asset directory preserved.' >&2
  exit 1
fi
git init "$destination"
git -C "$destination" remote add origin https://github.com/BrainCoTech/brainco-description.git
git -C "$destination" sparse-checkout init --cone
git -C "$destination" sparse-checkout set revotron_system revo3_system
git -C "$destination" fetch --depth 1 --filter=blob:none origin "$revision"
git -C "$destination" checkout --detach FETCH_HEAD
.venv/bin/python - "$destination" "$revision" <<'PY'
import json,sys
from pathlib import Path
(Path(sys.argv[1])/'source.json').write_text(json.dumps(dict(repository='https://github.com/BrainCoTech/brainco-description',revision=sys.argv[2],urdf='revotron_system/urdf/revotron_bimanual_revo3.urdf'),indent=2))
PY
