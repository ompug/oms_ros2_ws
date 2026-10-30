"""Relocalisation library (global registration, fine ICP, gate, simulation).

Experiments parallelise with processes, so math libraries default to one
thread per process. Set RELOC_THREADS to override (e.g. for a single process).
"""
import os as _os

for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    _os.environ.setdefault(_var, _os.environ.get("RELOC_THREADS", "1"))
