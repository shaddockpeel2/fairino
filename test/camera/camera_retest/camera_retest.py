import gc
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
SDK_ROOT = (
    ROOT
    / "sdk_py_1.1.4"
    / "OrbbecSDK_Python_v1.1.4_win_x64_release"
    / "python3.9"
)
PY_LIB = SDK_ROOT / "lib" / "python_lib"
C_LIB = SDK_ROOT / "lib" / "c_lib"
os.add_dll_directory(str(C_LIB))
sys.path.insert(0, str(PY_LIB))

from ObTypes import *  # noqa: E402,F403
import Pipeline  # noqa: E402
from Error import ObException  # noqa: E402


OUTPUT = ROOT / "results"
OUTPUT.mkdir(exist_ok=True)
FRAME_COUNT = 120
WARMUP_FRAMES = 30


def decode_color(frame):
    data = frame.data()
    width, height, fmt = frame.width(), frame.height(), frame.format()
    if fmt == OB_PY_FORMAT_MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    if fmt == OB_PY_FORMAT_RGB888:
        return cv2.cvtColor(np.asarray(data).reshape(height, width, 3), cv2.COLOR_RGB2BGR)
    if fmt == OB_PY_FORMAT_YUYV:
        return cv2.cvtColor(np.asarray(data).reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUYV)
    if fmt == OB_PY_FORMAT_UYVY:
        return cv2.cvtColor(np.asarray(data).reshape(height, width, 2), cv2.COLOR_YUV2BGR_UYVY)
    if fmt == OB_PY_FORMAT_I420:
        image = np.asarray(data).reshape(height * 3 // 2, width)
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
    raise RuntimeError(f"Unsupported color format: {fmt}")


def decode_depth_mm(frame):
    raw = np.asarray(frame.data()).reshape(frame.height(), frame.width(), 2)
    values = raw[:, :, 0].astype(np.uint16) + (raw[:, :, 1].astype(np.uint16) << 8)
    return np.rint(values.astype(np.float32) * frame.getValueScale()).astype(np.uint16)


def find_profile(pipe, sensor_type, width, height, fps, preferred_formats):
    profiles = pipe.getStreamProfileList(sensor_type)
    candidates = []
    for index in range(profiles.count()):
        profile = profiles.getProfile(index).toConcreteStreamProfile(OB_PY_STREAM_VIDEO)
        if profile.width() == width and profile.height() == height and profile.fps() == fps:
            candidates.append(profile)
    for wanted in preferred_formats:
        for profile in candidates:
            if profile.format() == wanted:
                return profile
    if candidates:
        return candidates[0]
    raise RuntimeError(f"No profile {width}x{height}@{fps}")


def camera_info(camera_param, key, frame_id):
    intrinsic = camera_param[f"{key}Intrinsic"]
    distortion = camera_param[f"{key}Distortion"]
    fx, fy, cx, cy = (intrinsic[x] for x in ("fx", "fy", "cx", "cy"))
    return {
        "header": {"frame_id": frame_id, "stamp": "set from each frame systemTimeStamp at publish time"},
        "height": intrinsic["height"],
        "width": intrinsic["width"],
        "distortion_model": "plumb_bob",
        "d": [distortion[x] for x in ("k1", "k2", "p1", "p2", "k3")],
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        "binning_x": 0,
        "binning_y": 0,
        "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
    }


def summarize_timestamps(records, key):
    values = np.asarray([r[key] for r in records], dtype=np.float64)
    deltas = np.diff(values)
    positive = deltas[deltas > 0]
    return {
        "zero_count": int(np.count_nonzero(values == 0)),
        "non_monotonic_steps": int(np.count_nonzero(deltas <= 0)),
        "median_frame_interval_ms": float(np.median(positive)) if positive.size else None,
        "p95_frame_interval_ms": float(np.percentile(positive, 95)) if positive.size else None,
    }


def depth_quality(depth_stack):
    frame_count, height, width = depth_stack.shape
    full = depth_stack.astype(np.float32)
    full[full == 0] = np.nan
    half = 50
    cy, cx = height // 2, width // 2
    roi = depth_stack[:, cy - half : cy + half, cx - half : cx + half].astype(np.float32)
    roi[roi == 0] = np.nan
    per_frame_median = np.nanmedian(roi, axis=(1, 2))
    per_pixel_std = np.nanstd(roi, axis=0)
    temporal_valid = np.sum(np.isfinite(roi), axis=0) >= frame_count * 0.8
    stable_std = per_pixel_std[temporal_valid]
    representative = np.nanmedian(roi, axis=0)
    center_depth = float(np.nanmedian(representative))
    inlier = np.isfinite(representative) & (np.abs(representative - center_depth) < 80.0)
    plane_rmse = None
    if np.count_nonzero(inlier) >= 100:
        yy, xx = np.indices(representative.shape)
        design = np.column_stack((xx[inlier], yy[inlier], np.ones(np.count_nonzero(inlier))))
        coeff, *_ = np.linalg.lstsq(design, representative[inlier], rcond=None)
        residual = representative[inlier] - design @ coeff
        plane_rmse = float(np.sqrt(np.mean(residual**2)))
    full_reference = np.nanmedian(full, axis=0)
    valid_count = np.sum(np.isfinite(full), axis=0)
    full_std = np.nanstd(full, axis=0)
    bands = {}
    for low, high in ((250, 350), (350, 500), (500, 750), (750, 1200)):
        mask = (
            np.isfinite(full_reference)
            & (full_reference >= low)
            & (full_reference < high)
            & (valid_count >= frame_count * 0.8)
        )
        values = full_std[mask]
        bands[f"{low}_{high}_mm"] = {
            "stable_pixel_count": int(values.size),
            "frame_area_ratio": float(values.size / (height * width)),
            "temporal_std_median_mm": float(np.median(values)) if values.size else None,
            "temporal_std_p95_mm": float(np.percentile(values, 95)) if values.size else None,
        }
    return {
        "full_frame_valid_pixel_ratio": float(np.count_nonzero(np.isfinite(full)) / full.size),
        "stable_in_range_bands": bands,
        "roi": {"x": cx - half, "y": cy - half, "width": 100, "height": 100},
        "median_distance_mm": float(np.nanmedian(per_frame_median)),
        "frame_median_jitter_std_mm": float(np.nanstd(per_frame_median)),
        "frame_median_peak_to_peak_mm": float(np.nanmax(per_frame_median) - np.nanmin(per_frame_median)),
        "valid_pixel_ratio": float(np.count_nonzero(np.isfinite(roi)) / roi.size),
        "per_pixel_temporal_std_median_mm": float(np.nanmedian(stable_std)) if stable_std.size else None,
        "per_pixel_temporal_std_p95_mm": float(np.nanpercentile(stable_std, 95)) if stable_std.size else None,
        "spatial_plane_fit_rmse_mm_if_center_is_planar": plane_rmse,
    }


def save_visuals(name, color, depth):
    def write_png(path, image):
        success, encoded = cv2.imencode(".png", image)
        if not success:
            raise RuntimeError(f"Failed to encode {path.name}")
        encoded.tofile(path)

    valid = depth > 0
    if np.any(valid):
        lo, hi = np.percentile(depth[valid], [2, 98])
        scaled = np.clip((depth.astype(np.float32) - lo) * 255.0 / max(hi - lo, 1.0), 0, 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
        depth_color[~valid] = 0
    else:
        depth_color = np.zeros((*depth.shape, 3), dtype=np.uint8)
    write_png(OUTPUT / f"{name}_color.png", color)
    write_png(OUTPUT / f"{name}_depth_mm.png", depth)
    write_png(OUTPUT / f"{name}_depth_color.png", depth_color)
    if color.shape[:2] == depth.shape:
        overlay = cv2.addWeighted(color, 0.58, depth_color, 0.42, 0)
        cv2.rectangle(
            overlay,
            (depth.shape[1] // 2 - 50, depth.shape[0] // 2 - 50),
            (depth.shape[1] // 2 + 50, depth.shape[0] // 2 + 50),
            (255, 255, 255),
            2,
        )
        write_png(OUTPUT / f"{name}_overlay.png", overlay)


def capture(pipe, color_profile, depth_profile, align_mode, name):
    config = Pipeline.Config()
    config.enableStream(color_profile)
    config.enableStream(depth_profile)
    config.setAlignMode(align_mode)
    pipe.start(config, None)
    frame_sync = {"supported": True, "enabled": False, "error": None}
    try:
        try:
            pipe.enableFrameSync()
            frame_sync["enabled"] = True
        except ObException as exc:
            frame_sync.update({"supported": False, "error": str(exc)})

        for _ in range(WARMUP_FRAMES):
            pipe.waitForFrames(500)

        camera_param = pipe.getCameraParam()
        records = []
        depths = []
        saved_color = saved_depth = None
        deadline = time.monotonic() + 20.0
        while len(records) < FRAME_COUNT and time.monotonic() < deadline:
            frames = pipe.waitForFrames(500)
            if frames is None:
                continue
            color = frames.colorFrame()
            depth = frames.depthFrame()
            if color is None or depth is None:
                continue
            color_image = decode_color(color)
            depth_image = decode_depth_mm(depth)
            if color_image is None:
                continue
            records.append(
                {
                    "color_index": color.index(),
                    "depth_index": depth.index(),
                    "color_device_timestamp_ms": color.timeStamp(),
                    "depth_device_timestamp_ms": depth.timeStamp(),
                    "color_system_timestamp_ms": color.systemTimeStamp(),
                    "depth_system_timestamp_ms": depth.systemTimeStamp(),
                    "system_skew_ms": int(color.systemTimeStamp()) - int(depth.systemTimeStamp()),
                }
            )
            depths.append(depth_image)
            if saved_color is None or len(records) == FRAME_COUNT // 2:
                saved_color, saved_depth = color_image.copy(), depth_image.copy()

        if not records:
            raise RuntimeError(f"No complete RGB-D frames captured for {name}")
        stack = np.stack(depths)
        save_visuals(name, saved_color, saved_depth)
        with (OUTPUT / f"{name}_timestamps.csv").open("w", encoding="utf-8", newline="") as handle:
            keys = list(records[0])
            handle.write(",".join(keys) + "\n")
            for row in records:
                handle.write(",".join(str(row[k]) for k in keys) + "\n")

        skews = np.asarray([r["system_skew_ms"] for r in records], dtype=np.float64)
        result = {
            "frames_captured": len(records),
            "frame_sync": frame_sync,
            "color_output": {"width": saved_color.shape[1], "height": saved_color.shape[0], "encoding": "bgr8"},
            "depth_output": {
                "width": saved_depth.shape[1],
                "height": saved_depth.shape[0],
                "encoding": "16UC1",
                "unit": "mm",
                "value_scale_mm_per_raw_lsb": float(frames.depthFrame().getValueScale()),
            },
            "camera_param": camera_param,
            "camera_info": {
                "color": camera_info(camera_param, "rgb", "camera_color_optical_frame"),
                "depth": camera_info(
                    camera_param,
                    "depth",
                    "camera_color_optical_frame"
                    if align_mode != OB_PY_ALIGN_DISABLE
                    else "camera_depth_optical_frame",
                ),
            },
            "timestamps": {
                "color_device": summarize_timestamps(records, "color_device_timestamp_ms"),
                "depth_device": summarize_timestamps(records, "depth_device_timestamp_ms"),
                "color_system": summarize_timestamps(records, "color_system_timestamp_ms"),
                "depth_system": summarize_timestamps(records, "depth_system_timestamp_ms"),
                "color_minus_depth_system_skew_ms": {
                    "median": float(np.median(skews)),
                    "p05": float(np.percentile(skews, 5)),
                    "p95": float(np.percentile(skews, 95)),
                    "max_abs": float(np.max(np.abs(skews))),
                },
            },
            "depth_quality_center_100x100": depth_quality(stack),
        }
        return result
    finally:
        pipe.stop()


def main():
    pipe = Pipeline.Pipeline(None, None)
    info = pipe.getDevice().getDeviceInfo()
    color_profile = find_profile(
        pipe, OB_PY_SENSOR_COLOR, 640, 480, 30, [OB_PY_FORMAT_MJPG, OB_PY_FORMAT_YUYV]
    )
    depth_profile = find_profile(
        pipe, OB_PY_SENSOR_DEPTH, 640, 400, 30, [OB_PY_FORMAT_Y16, OB_PY_FORMAT_YUYV]
    )
    report = {
        "verification_level": "L4: real powered camera, no actuation",
        "sdk_version": "Orbbec SDK 1.5.7 / Python bindings 1.1.4",
        "device": {
            "name": info.name().decode(errors="replace"),
            "vid": hex(info.vid()),
            "pid": hex(info.pid()),
            "serial_number": info.serialNumber().decode(errors="replace"),
            "firmware_version": info.firmwareVersion().decode(errors="replace"),
            "connection": "USB3.0",
        },
        "requested_profiles": {
            "color": {
                "width": color_profile.width(),
                "height": color_profile.height(),
                "fps": color_profile.fps(),
                "format_code": color_profile.format(),
            },
            "depth": {
                "width": depth_profile.width(),
                "height": depth_profile.height(),
                "fps": depth_profile.fps(),
                "format_code": depth_profile.format(),
            },
        },
    }
    try:
        report["raw"] = capture(pipe, color_profile, depth_profile, OB_PY_ALIGN_DISABLE, "raw")
        time.sleep(0.5)
        report["aligned_d2c_software"] = capture(
            pipe, color_profile, depth_profile, OB_PY_ALIGN_D2C_SW_MODE, "aligned"
        )
    finally:
        del pipe
        gc.collect()

    report_path = OUTPUT / "camera_retest_report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    for mode in ("raw", "aligned_d2c_software"):
        info_path = OUTPUT / f"camera_info_{mode}.json"
        info_path.write_text(
            json.dumps(report[mode]["camera_info"], ensure_ascii=False, indent=2), encoding="utf-8"
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"REPORT_PATH={report_path}")


if __name__ == "__main__":
    main()
