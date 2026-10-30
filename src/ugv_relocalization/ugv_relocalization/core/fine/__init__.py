"""Fine-registration wrappers (libpointmatcher, Open3D, small_gicp) behind one interface.

Every registrar has ``name`` and ``register(scan_cam, T_init) -> ICPResult``
and keeps its map-side data (filtered map, KD-tree, normals, covariances)
from the constructor, so per-call runtime excludes map precomputation.
Build one from a config with ``factory.build_fine``.
"""
