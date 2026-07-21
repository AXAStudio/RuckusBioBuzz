#!/usr/bin/env bash
# One-time setup for macOS / Linux / Git Bash on Windows: installs the Pedro visualizer's
# dependencies and adds this scripts/ directory to PATH, so `visualizer` works as a bare command
# in every future terminal on this machine. Re-running this script is safe (idempotent).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VISUALIZER_DIR="$(cd "${SCRIPT_DIR}/../tools/pedro-visualizer" && pwd)"

chmod +x "${SCRIPT_DIR}/visualizer" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/pollen-camera-tester" 2>/dev/null || true

if ! command -v npm >/dev/null 2>&1; then
  echo "install-visualizer-command: npm was not found on PATH." >&2
  echo "Install Node.js (https://nodejs.org/) first, then re-run this script." >&2
  exit 1
fi

echo "Installing Pedro visualizer dependencies..."
(cd "${VISUALIZER_DIR}" && npm ci)

PATH_LINE="export PATH=\"${SCRIPT_DIR}:\$PATH\""
MARKER="# Added by RuckusBioBuzz install-visualizer-command.sh"

detect_rc_files() {
  case "${SHELL:-}" in
    */zsh) printf '%s\n' "$HOME/.zshrc" ;;
    */bash) printf '%s\n' "$HOME/.bashrc" "$HOME/.bash_profile" ;;
    *) printf '%s\n' "$HOME/.bashrc" "$HOME/.bash_profile" "$HOME/.zshrc" ;;
  esac
}

updated_any=false
while IFS= read -r rc_file; do
  [ -z "$rc_file" ] && continue
  touch "$rc_file"
  if grep -qF "${SCRIPT_DIR}" "$rc_file" 2>/dev/null; then
    echo "Already configured in $rc_file"
  else
    printf '\n%s\n%s\n' "$MARKER" "$PATH_LINE" >> "$rc_file"
    echo "Added ${SCRIPT_DIR} to PATH in $rc_file"
    updated_any=true
  fi
done < <(detect_rc_files)

echo ""
if [ "$updated_any" = true ]; then
  echo "Setup complete. Open a new terminal (or run: source ~/.zshrc / source ~/.bashrc) and type: visualizer"
else
  echo "Already set up. Type: visualizer"
fi
