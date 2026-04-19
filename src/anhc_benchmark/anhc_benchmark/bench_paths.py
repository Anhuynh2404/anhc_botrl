"""Default filesystem paths for benchmark artifacts."""

from pathlib import Path


def default_benchmark_output_dir() -> str:
    """``<workspace>/bench_results`` (walk up from this installed package)."""
    here = Path(__file__).resolve().parent
    cur = here
    for _ in range(16):
        if (cur / "src" / "anhc_benchmark").is_dir():
            return str((cur / "bench_results").resolve())
        parent = cur.parent
        if parent == cur:
            break
        cur = parent
    return str(Path.home() / "anhc_botrl" / "bench_results")
