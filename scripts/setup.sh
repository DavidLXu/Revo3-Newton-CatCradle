#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
NEWTON_REV=4a9a27a7a3419c64201ca1fb0c3660c4a0f9ac58
BRAINCO_REV=6db7e2ed1d864ee0a3ab6ff62d43a303787137f6
if ! command -v uv >/dev/null; then
  echo 'Please install uv first: https://docs.astral.sh/uv/getting-started/installation/' >&2
  exit 1
fi
mkdir -p vendor assets
if [[ ! -d vendor/newton/.git ]]; then
  git init vendor/newton
  git -C vendor/newton remote add origin https://github.com/newton-physics/newton.git
  git -C vendor/newton fetch --depth 1 origin "$NEWTON_REV"
  git -C vendor/newton checkout --detach FETCH_HEAD
fi
if [[ ! -d assets/brainco-description/.git ]]; then
  git init assets/brainco-description
  git -C assets/brainco-description remote add origin https://github.com/BrainCoTech/brainco-description.git
  git -C assets/brainco-description sparse-checkout init --cone
  git -C assets/brainco-description sparse-checkout set revo3_system
  git -C assets/brainco-description fetch --depth 1 --filter=blob:none origin "$BRAINCO_REV"
  git -C assets/brainco-description checkout --detach FETCH_HEAD
fi
for spec in "vendor/newton:$NEWTON_REV" "assets/brainco-description:$BRAINCO_REV"; do
  repo="${spec%:*}"
  revision="${spec##*:}"
  if [[ "$(git -C "$repo" rev-parse HEAD)" != "$revision" ]]; then
    echo "Version mismatch in $repo; expected $revision. Existing checkout was preserved." >&2
    exit 1
  fi
done
[[ -d .venv ]] || uv venv --python 3.12
uv pip install --python .venv/bin/python -r requirements.lock
uv pip install --python .venv/bin/python --no-deps -e ./vendor/newton
echo 'Ready: .venv/bin/python -m revo3_cat_cradle'
