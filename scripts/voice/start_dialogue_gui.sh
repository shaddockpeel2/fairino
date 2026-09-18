#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
set -u
export PYTHONPATH="${workspace}/src/fr3c_voice:${PYTHONPATH:-}"

# Tk uses the X Input Method (XIM) bridge on X11.  A terminal opened from a
# minimal session may see the running IBus/Fcitx daemon without inheriting the
# variables that connect Tk to it, which makes Chinese input impossible even
# though Chinese text renders correctly.  Preserve explicit user settings and
# only fill in missing values.
if [[ -z "${XMODIFIERS:-}" ]]; then
  if pgrep -x ibus-daemon >/dev/null 2>&1; then
    export XMODIFIERS='@im=ibus'
    export GTK_IM_MODULE="${GTK_IM_MODULE:-ibus}"
    export QT_IM_MODULE="${QT_IM_MODULE:-ibus}"
  elif pgrep -x fcitx5 >/dev/null 2>&1 || pgrep -x fcitx >/dev/null 2>&1; then
    export XMODIFIERS='@im=fcitx'
    export GTK_IM_MODULE="${GTK_IM_MODULE:-fcitx}"
    export QT_IM_MODULE="${QT_IM_MODULE:-fcitx}"
  fi
fi
exec /usr/bin/python3 -m fr3c_voice.dialogue_gui --workspace "${workspace}" "$@"
