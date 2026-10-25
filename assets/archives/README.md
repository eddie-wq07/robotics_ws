# Saved run snapshots

`saved-runs-20261025T052146Z.tar.zst` contains the saved files under
`assets/outputs/` and the legacy `src/3d-pen/runs/` directory as captured on
2026-10-25 UTC. The preview cache is excluded. Its adjacent `.files.txt` lists
the archived paths; `.sha256` records the archive checksum.

The archive is stored in Git LFS. To restore it in a fresh checkout, install
Git LFS and `zstd`, then run from the repository root:

```bash
git lfs install
git lfs pull
sha256sum -c assets/archives/saved-runs-20261025T052146Z.sha256
tar --zstd -xf assets/archives/saved-runs-20261025T052146Z.tar.zst
```

Extraction restores the original paths and overwrites matching files. These
are historical run artifacts, including failed and incomplete attempts; the
archive does not establish that any particular run completed successfully.

Current source, configuration profiles, scripts, dependency requirements,
documentation, and reusable input/mesh assets are committed separately and do
not require archive extraction. Build/install directories, virtual environments,
dependency caches, nested Git metadata, and credentials are not included.
