#!/usr/bin/env bash
set -euo pipefail

website_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$website_dir/.." && pwd)"
output_dir="${1:-$repo_root/.site-dist}"

if [[ "$output_dir" != "$repo_root/.site-dist" && "$output_dir" != /tmp/sapphire-pages* ]]; then
  printf 'Refusing to clear unexpected output directory: %s\n' "$output_dir" >&2
  exit 1
fi

command -v ffmpeg >/dev/null || {
  printf 'ffmpeg is required to build the website media.\n' >&2
  exit 1
}

rm -rf -- "$output_dir"
mkdir -p "$output_dir/assets/media"

cp "$website_dir/index.html" "$output_dir/"
cp "$website_dir/styles.css" "$output_dir/"
cp "$website_dir/script.js" "$output_dir/"

encode_demo() {
  local name="$1"
  local input="$2"
  local poster_frame="$3"

  ffmpeg -y -loglevel error \
    -i "$input" \
    -an \
    -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" \
    -c:v libx264 \
    -preset medium \
    -crf 26 \
    -pix_fmt yuv420p \
    -movflags +faststart \
    "$output_dir/assets/media/$name.mp4"

  ffmpeg -y -loglevel error \
    -i "$input" \
    -vf "select=eq(n\\,$poster_frame),scale=trunc(iw/2)*2:trunc(ih/2)*2" \
    -frames:v 1 \
    "$output_dir/assets/media/$name.png"
}

encode_demo "s3e" "$repo_root/docs/sapphire.gif" 36
encode_demo "hilti-2022" "$repo_root/docs/hilti-2022.gif" 24
encode_demo "hilti-2023" "$repo_root/docs/hilti-2023.gif" 24
encode_demo "newer-college" "$repo_root/docs/newer_college.gif" 36

printf 'Website built at %s\n' "$output_dir"
