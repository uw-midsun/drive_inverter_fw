#!/usr/bin/env bash

set -euo pipefail

COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECTS_DIR="$(dirname "${COMMON_DIR}")"
STYLE="file:${COMMON_DIR}/config/.clang-format"
CPPLINT_CFG="${COMMON_DIR}/config/CPPLINT.cfg"

CHECK=0
[[ "${1:-}" == "--check" ]] && CHECK=1

# Translate the shared CPPLINT.cfg into cpplint command line args
linelength=80
filters=""
while IFS= read -r line; do
    line="${line%%#*}"
    line="$(echo "$line" | xargs || true)"
    [[ -z "$line" ]] && continue
    key="${line%%=*}"
    val="${line#*=}"
    key="$(echo "$key" | xargs)"
    val="$(echo "$val" | xargs)"
    case "$key" in
        linelength) linelength="$val" ;;
        filter) filters="${filters:+$filters,}$val" ;;
    esac
done < "${CPPLINT_CFG}"

mapfile -t APP_DIRS < <(find "${PROJECTS_DIR}" -type d -path '*/Core/Src/app' -not -path '*/build/*' | sort)

fail=0
for app in "${APP_DIRS[@]}"; do
    mapfile -t files < <(find "$app" -type f \( -name '*.c' -o -name '*.h' \) | sort)
    [[ ${#files[@]} -eq 0 ]] && continue
    echo ">> ${app#"${PROJECTS_DIR}/"}  (${#files[@]} files)"

    if [[ $CHECK -eq 1 ]]; then
        clang-format --style="${STYLE}" --dry-run --Werror "${files[@]}" || fail=1
    else
        clang-format --style="${STYLE}" -i "${files[@]}"
    fi

    cpplint --quiet --linelength="${linelength}" --filter="${filters}" \
        "${files[@]}" || fail=1
done

exit $fail
