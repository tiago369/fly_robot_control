Vendored, unmodified, from:

  https://github.com/google-deepmind/mujoco_menagerie
  path: skydio_x2/
  commit: 8161bba264d7fa7c99ca301e91e7fb44737676ad (2026-09-12)

License: Apache-2.0 (see LICENSE in this directory). Model assets provided by
Skydio; see README.md in this directory for attribution.

Do not hand-edit the files copied from upstream (x2.xml, scene.xml, README.md,
CHANGELOG.md, LICENSE, assets/). Project-specific MJCF additions (see
drone_scene.xml in this same directory) are separate, clearly-labeled files
that <include> scene.xml/x2.xml rather than editing them.

They live in *this* directory (not one level up) because MuJoCo resolves an
included file's relative asset paths (x2.xml's `assetdir="assets"`) against
the outermost loaded file's directory, not the including file's directory -
putting the wrapper one level up broke mesh loading. Confirmed empirically
while building M0.
