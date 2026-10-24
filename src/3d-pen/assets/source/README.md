# Source CAD

Seven unique STL assets from the team's Onshape document are in [onshape/](onshape/README.md). All were exported as **binary STL, meters, fine resolution, original Z-up** and checked for finite, watertight geometry. Their hashes and bounds are in `onshape/manifest.json`.

**Downloaded does not mean calibrated:** these files use each part's CAD coordinates. The active scene still uses nominal geometry; populate `configs/assets.yaml` only after establishing the jig-to-robot and holder-to-flange transforms. The real sphere is **40 mm radius**, while the current nominal surface is 75 mm radius. Surface configuration and CAD placement must be updated together.
