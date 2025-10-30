#!/usr/bin/env python3
"""
Print HDF5 file contents (structure, datasets, shapes, dtypes, sizes, and optional samples).

Usage examples:
  python Utils/print_hdf5.py Data/realman_demo/train_data/record_data_000.hdf5
  python Utils/print_hdf5.py my.hdf5 --samples 8 --show-attrs
"""

import argparse
import sys
from pathlib import Path
from typing import Optional, Union, List

import h5py


def format_bytes(num_bytes: Optional[int]) -> str:
    if num_bytes is None:
        return "?B"
    step_unit = 1024.0
    for unit in ["B", "KB", "MB", "GB", "TB"]:
        if num_bytes < step_unit:
            return f"{num_bytes:.2f} {unit}"
        num_bytes /= step_unit
    return f"{num_bytes:.2f} PB"


def print_attrs(prefix: str, obj: Union[h5py.Dataset, h5py.Group]) -> None:
    if len(obj.attrs) == 0:
        return
    print(f"{prefix}  attrs ({len(obj.attrs)}):")
    for k, v in obj.attrs.items():
        try:
            print(f"{prefix}    - {k}: {v}")
        except Exception:
            print(f"{prefix}    - {k}: <unprintable>")


def print_dataset(path: str, dset: h5py.Dataset, prefix: str, sample_count: int, show_attrs: bool) -> None:
    shape = dset.shape
    dtype = dset.dtype
    nbytes = dset.size * (dset.dtype.itemsize if hasattr(dset.dtype, 'itemsize') else 0)
    comp = dset.compression
    chunks = dset.chunks
    print(f"{prefix}- [DATASET] {path}")
    print(
        f"{prefix}    shape={shape}, dtype={dtype}, size={dset.size}, nbytes≈{format_bytes(nbytes)}, compression={comp}, chunks={chunks}"
    )
    if show_attrs:
        print_attrs(prefix, dset)

    # Optional sampling (only for small datasets to avoid heavy IO)
    if sample_count > 0 and dset.size > 0:
        try:
            # Read a small sample from the first axis when possible
            if len(shape) == 0:
                sample = dset[()]
            else:
                # Construct a slice that grabs up to sample_count along the first axis
                first_dim = min(shape[0], sample_count)
                slicer = [slice(0, first_dim)] + [slice(0, min(4, s)) for s in shape[1:]]
                sample = dset[tuple(slicer)]
            print(f"{prefix}    sample (truncated): {repr(sample)[:512]}")
        except Exception as e:
            print(f"{prefix}    <sampling failed: {e}>")


def print_group(path: str, grp: h5py.Group, prefix: str, show_attrs: bool) -> None:
    kind = "[FILE]" if path == "/" else "[GROUP]"
    print(f"{prefix}- {kind} {path}")
    if show_attrs:
        print_attrs(prefix, grp)


def walk_hdf5(hf: h5py.File, sample_count: int, show_attrs: bool, max_depth: Optional[int]) -> None:
    def visitor(name: str, obj):
        depth = name.count('/')
        if max_depth is not None and depth > max_depth:
            return
        indent = "  " * depth
        full_path = "/" + name if not name.startswith('/') else name
        if isinstance(obj, h5py.Dataset):
            print_dataset(full_path, obj, indent, sample_count, show_attrs)
        elif isinstance(obj, h5py.Group):
            print_group(full_path, obj, indent, show_attrs)

    # Ensure root is printed first
    print_group("/", hf, prefix="", show_attrs=show_attrs)
    hf.visititems(visitor)


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Print contents of an HDF5 file")
    parser.add_argument("hdf5_path", help="Path to .hdf5 file")
    parser.add_argument("--samples", type=int, default=0, help="Print up to N samples from each dataset (default: 0)")
    parser.add_argument("--show-attrs", action="store_true", help="Show attributes for groups/datasets")
    parser.add_argument("--max-depth", type=int, default=None, help="Limit traversal depth (root=/ is 0)")

    args = parser.parse_args(argv)

    path = Path(args.hdf5_path)
    if not path.exists():
        print(f"Error: file not found: {path}")
        return 1

    try:
        with h5py.File(path, "r") as hf:
            walk_hdf5(
                hf,
                sample_count=args.samples,
                show_attrs=args.show_attrs,
                max_depth=args.max_depth,
            )
    except OSError as e:
        print(f"Error opening HDF5 file: {e}")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
