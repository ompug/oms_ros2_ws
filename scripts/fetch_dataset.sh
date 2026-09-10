#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
FILE_ID="1lPC1o5c58bP08gD6rVWefEAMzAF_aCNf"
NAME="2017-06-08-15-49-45_0.bag"
DEST="${ROOT}/data/raw/${NAME}"

if [[ ! -x "${ROOT}/.venv/bin/gdown" ]]; then
  echo "Run ${ROOT}/install_dependencies.sh first." >&2
  exit 1
fi
if [[ ! -s "${DEST}" ]]; then
  "${ROOT}/.venv/bin/gdown" --id "${FILE_ID}" --output "${DEST}"
fi

SIZE="$(stat -c '%s' "${DEST}")"
SHA256="$(sha256sum "${DEST}" | awk '{print $1}')"
printf 'filename=%s\nsource_file_id=%s\nsize_bytes=%s\nsha256=%s\n' \
  "${NAME}" "${FILE_ID}" "${SIZE}" "${SHA256}" \
  > "${ROOT}/data/${NAME}.metadata"
echo "Downloaded ${DEST} (${SIZE} bytes, sha256 ${SHA256})"

