#!/usr/bin/env python3
"""Offline slope raster computer (paper-faithful, no ROS required).

Implements the same height-based slope calculation strategy as the C++ code
(see ``GroundSegmentation::compute_slope_map``) but on a plain NumPy height
raster, so users can verify the algorithm, post-process saved ground rasters,
or run it on synthetic DEMs from outside the ROS pipeline.

Reference: TS-SatMVSNet: Slope Aware Height Estimation for Large-Scale Earth
Terrain MVS, Zhang et al., arXiv:2501.01049, 2025. Equations 1 and 7.

Usage:
    python compute_slope_offline.py input_height.npy output_dir \\
            --resolution 0.33 --clip_deg 45 \\
            --height_correction --iterations 1

Inputs:
    input_height        Path to a NumPy ``.npy`` or 32-bit ``.tif`` height map
                        (units: metres). NaN is treated as no-data.

Outputs (in ``output_dir``):
    slope_deg.tif       Slope angle in degrees (float32, GeoTIFF if possible).
    slope_deg.png       8-bit grayscale slope image (clipped at ``clip_deg``).
    slope_color.png     JET color-mapped slope image (if cv2 is available).
    slope_dir.png       8-bit visualisation of the paper's 0..8 direction code.
    slope_maxdiff.tif   |max(p3x3) - center| height delta (paper Eq. 1).
    ground_corrected.tif Smoothed height used as the slope source.
"""

import argparse
import os
import sys

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

try:
    import rasterio
    from rasterio.transform import from_origin
    _HAS_RASTERIO = True
except Exception:
    _HAS_RASTERIO = False


# Paper Fig. 4 directional codes; matches GroundSegmentation::DIR_CODE.
#   (di=-1,dj=-1)=8 (di=-1,dj=0)=7 (di=-1,dj=+1)=6
#   (di= 0,dj=-1)=5 (di= 0,dj=0)=4 (di= 0,dj=+1)=3
#   (di=+1,dj=-1)=2 (di=+1,dj=0)=1 (di=+1,dj=+1)=0
DIR_CODE = np.array([[8, 7, 6],
                     [5, 4, 3],
                     [2, 1, 0]], dtype=np.int16)

# Paper Eq. 7: Gaussian-shaped 3x3 height correction kernel.
GAUSS_K = np.array([[1/16, 1/8, 1/16],
                    [1/8 , 1/4, 1/8 ],
                    [1/16, 1/8, 1/16]], dtype=np.float32)


def load_height_map(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".npy":
        return np.load(path).astype(np.float32)
    if ext in (".tif", ".tiff"):
        if not _HAS_RASTERIO:
            raise RuntimeError("rasterio is required to read GeoTIFF inputs.")
        with rasterio.open(path) as src:
            return src.read(1).astype(np.float32)
    if _HAS_CV2:
        img = cv2.imread(path, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError("Failed to read %s" % path)
        return img.astype(np.float32)
    raise RuntimeError("Unsupported input format: %s" % ext)


def height_correction(height, iterations=1, confidence=None, conf_exp=2.0):
    """Apply the paper Eq. 7 3x3 Gaussian smoothing operator.

    If a confidence map is given it is used to blend smoothed vs. raw heights
    (high confidence -> keep raw value), matching the C++ implementation.
    """
    result = height.astype(np.float32, copy=True)
    h, w = result.shape
    if confidence is not None:
        c = np.clip(confidence.astype(np.float32), 0.0, 1.0) ** conf_exp
    else:
        c = np.zeros_like(result)

    for _ in range(max(1, int(iterations))):
        padded = np.pad(result, 1, mode="edge")
        smoothed = np.zeros_like(result)
        for di in range(3):
            for dj in range(3):
                smoothed += GAUSS_K[di, dj] * padded[di:di+h, dj:dj+w]
        result = c * result + (1.0 - c) * smoothed
    return result


def compute_slope(height, resolution=1.0):
    """Compute slope map, paper-style slope direction codes and gradients.

    Returns a dict with keys:
        slope_deg, slope_max_diff, slope_x, slope_y, slope_dir
    """
    h, w = height.shape
    padded = np.pad(height, 1, mode="edge")

    # 3x3 neighbour stack: shape (9, h, w).
    stack = np.empty((9, h, w), dtype=np.float32)
    code_stack = np.empty((9, h, w), dtype=np.int16)
    k = 0
    for di in range(-1, 2):
        for dj in range(-1, 2):
            stack[k] = padded[1+di:1+di+h, 1+dj:1+dj+w]
            code_stack[k] = DIR_CODE[di+1, dj+1]
            k += 1

    # Paper Eq. 1: max - center.
    center = stack[4]
    # argmax over the 9-neighbourhood, ignoring NaN by treating them as -inf.
    safe = np.where(np.isnan(stack), -np.inf, stack)
    idx = np.argmax(safe, axis=0)
    max_val = np.take_along_axis(stack, idx[None, ...], axis=0)[0]
    slope_max_diff = np.where(np.isfinite(max_val) & np.isfinite(center),
                              max_val - center, 0.0).astype(np.float32)
    slope_dir = np.take_along_axis(code_stack, idx[None, ...], axis=0)[0].astype(np.int16)

    # Standard gradient slope via central differences for a smooth angle map.
    inv_2res = 1.0 / (2.0 * resolution)
    dzdx = (padded[1:1+h, 2:2+w] - padded[1:1+h, 0:w]) * inv_2res
    dzdy = (padded[2:2+h, 1:1+w] - padded[0:h, 1:1+w]) * inv_2res
    grad_mag = np.sqrt(dzdx*dzdx + dzdy*dzdy)
    slope_deg = np.degrees(np.arctan(grad_mag)).astype(np.float32)

    return {
        "slope_deg": slope_deg,
        "slope_max_diff": slope_max_diff,
        "slope_x": dzdx.astype(np.float32),
        "slope_y": dzdy.astype(np.float32),
        "slope_dir": slope_dir,
    }


def save_geotiff(path, arr, resolution=1.0, origin_xy=(0.0, 0.0)):
    if not _HAS_RASTERIO:
        np.save(os.path.splitext(path)[0] + ".npy", arr)
        return False
    h, w = arr.shape
    transform = from_origin(origin_xy[0], origin_xy[1] + h * resolution,
                            resolution, resolution)
    profile = {
        "driver": "GTiff",
        "height": h,
        "width": w,
        "count": 1,
        "dtype": str(arr.dtype),
        "transform": transform,
        "crs": None,
        "nodata": float("nan") if arr.dtype.kind == "f" else None,
    }
    with rasterio.open(path, "w", **profile) as dst:
        dst.write(arr, 1)
    return True


def save_png_gray(path, arr, vmax):
    if not _HAS_CV2:
        np.save(os.path.splitext(path)[0] + ".npy", arr)
        return False
    norm = np.clip(arr, 0.0, vmax) / max(vmax, 1e-6) * 255.0
    cv2.imwrite(path, norm.astype(np.uint8))
    return True


def save_png_color(path, arr, vmax):
    if not _HAS_CV2:
        return False
    norm = np.clip(arr, 0.0, vmax) / max(vmax, 1e-6) * 255.0
    color = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_JET)
    cv2.imwrite(path, color)
    return True


def save_dir_png(path, dir_code):
    if not _HAS_CV2:
        return False
    # Direction codes 0..8 -> 0..255 spread for visualisation.
    norm = (dir_code.astype(np.float32) / 8.0 * 255.0).astype(np.uint8)
    color = cv2.applyColorMap(norm, cv2.COLORMAP_HSV)
    cv2.imwrite(path, color)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input_height", help="Input height raster (.npy / .tif / .png).")
    ap.add_argument("output_dir", help="Directory for the output rasters.")
    ap.add_argument("--resolution", type=float, default=1.0,
                    help="Cell size in metres. Used both for slope angle and for "
                         "GeoTIFF georeferencing. Default: 1.0.")
    ap.add_argument("--clip_deg", type=float, default=45.0,
                    help="Slope clip [deg] for 8-bit visualisations. Default: 45.")
    ap.add_argument("--height_correction", action="store_true",
                    help="Apply paper Eq. 7 3x3 Gaussian height correction "
                         "before slope computation.")
    ap.add_argument("--iterations", type=int, default=1,
                    help="Height correction iterations. Default: 1.")
    args = ap.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    height = load_height_map(args.input_height)
    if args.height_correction:
        corrected = height_correction(height, iterations=args.iterations)
        save_geotiff(os.path.join(args.output_dir, "ground_corrected.tif"),
                     corrected, resolution=args.resolution)
        source = corrected
    else:
        source = height

    out = compute_slope(source, resolution=args.resolution)

    save_geotiff(os.path.join(args.output_dir, "slope_deg.tif"),
                 out["slope_deg"], resolution=args.resolution)
    save_geotiff(os.path.join(args.output_dir, "slope_maxdiff.tif"),
                 out["slope_max_diff"], resolution=args.resolution)
    save_png_gray(os.path.join(args.output_dir, "slope_deg.png"),
                  out["slope_deg"], args.clip_deg)
    save_png_color(os.path.join(args.output_dir, "slope_color.png"),
                   out["slope_deg"], args.clip_deg)
    save_dir_png(os.path.join(args.output_dir, "slope_dir.png"),
                 out["slope_dir"])

    print("Saved slope rasters to %s" % args.output_dir)
    print("  slope_deg.tif       : %s" % out["slope_deg"].shape)
    print("  slope_maxdiff.tif   : %s" % out["slope_max_diff"].shape)
    print("  slope_dir.png       : %s" % out["slope_dir"].shape)
    if not _HAS_RASTERIO:
        print("  (rasterio missing - GeoTIFFs were saved as .npy fallback)")
    if not _HAS_CV2:
        print("  (cv2 missing - PNGs were saved as .npy fallback)")


if __name__ == "__main__":
    sys.exit(main() or 0)
