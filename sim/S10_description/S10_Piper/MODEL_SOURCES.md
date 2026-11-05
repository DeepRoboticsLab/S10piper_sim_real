# Model sources

- S10 base model, meshes, inertias, limits and collision geometry:
  `/home/weidong/deep_robotics_model-main/S10/mjcf/S10.xml`
- Piper arm subtree and meshes: the same Piper model used by the existing
  earlier Piper integration (original source is retained in the pre-edit backup).
- Piper mount pose: `0.045 0 0.117616`, matching the generated
  `S10_Piper.urdf` and the S10 training asset.

The combined MJCF has 24 actuators.  The simulator exposes only the first 16
S10 actuators and Piper joints 1--6 through UDP; gripper joints 7/8 are held
closed internally.
