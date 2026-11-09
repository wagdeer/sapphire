# Sapphire website

Static project site for `https://wagdeer.github.io/sapphire/`.

## Preview locally

```bash
./build-site.sh /tmp/sapphire-pages-preview
python3 -m http.server 8000 --directory /tmp/sapphire-pages-preview
```

Open `http://localhost:8000`.

The build script converts the repository's demonstration GIFs into smaller
MP4 files plus poster images. The generated site is written to `.site-dist/`
by default and is not committed.

GitHub Pages deploys `.site-dist/` through `.github/workflows/pages.yml`.
