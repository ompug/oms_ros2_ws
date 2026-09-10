#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
SEGMENT="${1:-0}"
case "${SEGMENT}" in
  0) FILE_ID="1lPC1o5c58bP08gD6rVWefEAMzAF_aCNf"; NAME="2017-06-08-15-49-45_0.bag" ;;
  1) FILE_ID="1od1lYuyy6aFSiSsM5w1kKCWVNO0bwDvj"; NAME="2017-06-08-15-50-45_1.bag" ;;
  2) FILE_ID="18sGKMwHS4CsEpGnOFNMjlnk_-ZHdNDhy"; NAME="2017-06-08-15-51-45_2.bag" ;;
  3) FILE_ID="1pTJAugBwHfiqd9aTjtGXgaXyvABeZe84"; NAME="2017-06-08-15-52-45_3.bag" ;;
  *) echo "Usage: $0 [0|1|2|3]" >&2; exit 2 ;;
esac
DEST="${ROOT}/data/raw/${NAME}"

if [[ ! -x "${ROOT}/.venv/bin/gdown" ]]; then
  echo "Run ${ROOT}/install_dependencies.sh first." >&2
  exit 1
fi
if [[ ! -s "${DEST}" ]]; then
  "${ROOT}/.venv/bin/gdown" "${FILE_ID}" --output "${DEST}"
fi

SIZE="$(stat -c '%s' "${DEST}")"
SHA256="$(sha256sum "${DEST}" | awk '{print $1}')"
printf 'filename=%s\nsource_file_id=%s\nsize_bytes=%s\nsha256=%s\n' \
  "${NAME}" "${FILE_ID}" "${SIZE}" "${SHA256}" \
  > "${ROOT}/data/${NAME}.metadata"
echo "Downloaded ${DEST} (${SIZE} bytes, sha256 ${SHA256})"
