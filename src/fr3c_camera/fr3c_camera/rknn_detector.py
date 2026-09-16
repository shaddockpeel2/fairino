"""Pure RKNNLite YOLO detector backend for the RK3588 production path."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
try:
    from rknnlite.api import RKNNLite
except ImportError:  # Pure post-processing tests do not require the NPU wheel.
    RKNNLite = None  # type: ignore[assignment]


CLASS_NAMES = (
    "Apple_core",
    "Book",
    "Cigarette_butts",
    "Disposable_chopsticks",
    "Egg_shell",
    "Expired_cosmetics",
    "Expired_tablets",
    "Fish_bone",
    "Newspaper",
    "Old_school_bag",
    "Peach_pit",
    "Syringe",
    "Toilet_paper",
    "Used_batteries",
    "Watermelon_rind",
    "Zip_top_can",
)

EXPECTED_OUTPUT_SHAPES = (
    (1, 64, 80, 80),
    (1, 16, 80, 80),
    (1, 1, 80, 80),
    (1, 64, 40, 40),
    (1, 16, 40, 40),
    (1, 1, 40, 40),
    (1, 64, 20, 20),
    (1, 16, 20, 20),
    (1, 1, 20, 20),
)

CORE_MASK_NAMES = ("core0", "core012")


@dataclass(frozen=True)
class Detection:
    """Backend-independent detection in original BGR image coordinates."""

    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]

    @property
    def center_uv(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox_xyxy
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


@dataclass(frozen=True)
class LetterboxTransform:
    ratio: float
    pad_x: int
    pad_y: int
    original_width: int
    original_height: int


def _letterbox_bgr(
    image: np.ndarray,
    size: int = 640,
) -> tuple[np.ndarray, LetterboxTransform]:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected HxWx3 BGR image, got {image.shape}")
    height, width = image.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")

    ratio = min(size / width, size / height)
    resized_width = round(width * ratio)
    resized_height = round(height * ratio)
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    pad_x = (size - resized_width) // 2
    pad_y = (size - resized_height) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[
        pad_y:pad_y + resized_height,
        pad_x:pad_x + resized_width,
    ] = resized

    # best.rknn embeds rgb2bgr=False. The public interface is deliberately BGR,
    # therefore this is the only BGR -> RGB conversion in the inference path.
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    transform = LetterboxTransform(
        ratio=ratio,
        pad_x=pad_x,
        pad_y=pad_y,
        original_width=width,
        original_height=height,
    )
    return rgb, transform


def _softmax(values: np.ndarray, axis: int) -> np.ndarray:
    shifted = values - np.max(values, axis=axis, keepdims=True)
    exponentials = np.exp(shifted)
    return exponentials / np.sum(exponentials, axis=axis, keepdims=True)


def _decode_outputs(
    outputs: list[np.ndarray],
    transform: LetterboxTransform,
    confidence_threshold: float,
    iou_threshold: float,
    max_detections: int,
) -> list[Detection]:
    shapes = tuple(tuple(int(value) for value in output.shape) for output in outputs)
    if shapes != EXPECTED_OUTPUT_SHAPES:
        raise ValueError(f"unexpected RKNN output contract: {shapes}")

    boxes: list[list[float]] = []
    scores: list[float] = []
    class_ids: list[int] = []

    for head_index, stride in enumerate((8, 16, 32)):
        regression, classes, objects = outputs[
            head_index * 3:head_index * 3 + 3
        ]
        combined_scores = classes[0] * objects[0, 0][None, :, :]
        candidate_classes, rows, columns = np.where(
            combined_scores >= confidence_threshold
        )
        if not len(columns):
            continue

        candidate_scores = combined_scores[candidate_classes, rows, columns]
        order = np.argsort(candidate_scores)[-30000:]
        candidate_classes = candidate_classes[order]
        rows = rows[order]
        columns = columns[order]
        candidate_scores = candidate_scores[order]

        distributions = regression[0][:, rows, columns].reshape(4, 16, -1)
        distances = (
            _softmax(distributions, axis=1)
            * np.arange(16, dtype=np.float32)[None, :, None]
        ).sum(axis=1)
        centers_x = (columns.astype(np.float32) + 0.5) * stride
        centers_y = (rows.astype(np.float32) + 0.5) * stride
        x1_values = centers_x - distances[0] * stride
        y1_values = centers_y - distances[1] * stride
        x2_values = centers_x + distances[2] * stride
        y2_values = centers_y + distances[3] * stride

        for index in range(len(columns)):
            x1 = float(x1_values[index])
            y1 = float(y1_values[index])
            x2 = float(x2_values[index])
            y2 = float(y2_values[index])
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(float(candidate_scores[index]))
            class_ids.append(int(candidate_classes[index]))

    detections: list[Detection] = []
    for class_id, class_name in enumerate(CLASS_NAMES):
        class_indices = [
            index for index, value in enumerate(class_ids) if value == class_id
        ]
        if not class_indices:
            continue
        kept = cv2.dnn.NMSBoxes(
            [boxes[index] for index in class_indices],
            [scores[index] for index in class_indices],
            confidence_threshold,
            iou_threshold,
        )
        for kept_index in np.asarray(kept).reshape(-1):
            source_index = class_indices[int(kept_index)]
            x, y, width, height = boxes[source_index]
            x1 = float(
                np.clip(
                    (x - transform.pad_x) / transform.ratio,
                    0,
                    transform.original_width,
                )
            )
            y1 = float(
                np.clip(
                    (y - transform.pad_y) / transform.ratio,
                    0,
                    transform.original_height,
                )
            )
            x2 = float(
                np.clip(
                    (x + width - transform.pad_x) / transform.ratio,
                    0,
                    transform.original_width,
                )
            )
            y2 = float(
                np.clip(
                    (y + height - transform.pad_y) / transform.ratio,
                    0,
                    transform.original_height,
                )
            )
            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=scores[source_index],
                    bbox_xyxy=(x1, y1, x2, y2),
                )
            )

    detections.sort(key=lambda item: item.confidence, reverse=True)
    return detections[:max_detections]


class RKNNDetector:
    """RKNNLite-only implementation for best.rknn on RK3588."""

    def __init__(
        self,
        model_path: str | Path,
        confidence_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        core_mode: str = "core0",
        max_detections: int = 100,
    ) -> None:
        if core_mode not in CORE_MASK_NAMES:
            raise ValueError(
                f"core_mode must be one of {CORE_MASK_NAMES}, got {core_mode}"
            )
        if not 0.0 < confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in (0, 1]")
        if not 0.0 < iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be in (0, 1]")
        if max_detections <= 0:
            raise ValueError("max_detections must be positive")

        self.model_path = Path(model_path).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"RKNN model not found: {self.model_path}")
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.max_detections = max_detections
        self.core_mode = core_mode
        self.last_timings_ms: dict[str, float] = {}
        self.output_shapes: tuple[tuple[int, ...], ...] = ()
        self._closed = False
        if RKNNLite is None:
            raise RuntimeError(
                "rknnlite is unavailable; source the RK3588 runtime environment"
            )
        core_masks = {
            "core0": RKNNLite.NPU_CORE_0,
            "core012": RKNNLite.NPU_CORE_0_1_2,
        }
        self._rknn = RKNNLite(verbose=False)

        load_result = self._rknn.load_rknn(str(self.model_path))
        if load_result != 0:
            self._rknn.release()
            self._closed = True
            raise RuntimeError(f"load_rknn failed with code {load_result}")
        init_result = self._rknn.init_runtime(core_mask=core_masks[core_mode])
        if init_result != 0:
            self._rknn.release()
            self._closed = True
            raise RuntimeError(f"init_runtime failed with code {init_result}")
        self.sdk_version = self._rknn.get_sdk_version()

    def infer(self, bgr_image: np.ndarray) -> list[Detection]:
        if self._closed:
            raise RuntimeError("RKNN backend is closed")

        import time

        started = time.perf_counter()
        input_rgb, transform = _letterbox_bgr(bgr_image)
        preprocess_ms = (time.perf_counter() - started) * 1000.0

        started = time.perf_counter()
        outputs = self._rknn.inference(
            inputs=[input_rgb[None]],
            data_format=["nhwc"],
        )
        npu_ms = (time.perf_counter() - started) * 1000.0
        if outputs is None:
            raise RuntimeError("RKNN inference returned None")

        started = time.perf_counter()
        detections = _decode_outputs(
            outputs,
            transform,
            self.confidence_threshold,
            self.iou_threshold,
            self.max_detections,
        )
        postprocess_ms = (time.perf_counter() - started) * 1000.0
        self.output_shapes = tuple(tuple(output.shape) for output in outputs)
        self.last_timings_ms = {
            "preprocess": preprocess_ms,
            "npu": npu_ms,
            "postprocess": postprocess_ms,
        }
        return detections

    def close(self) -> None:
        if not self._closed:
            self._rknn.release()
            self._closed = True

    def metadata(self) -> dict[str, Any]:
        return {
            "backend": "rknnlite",
            "model_path": str(self.model_path),
            "classes": list(CLASS_NAMES),
            "core_mode": self.core_mode,
            "output_shapes": [list(shape) for shape in self.output_shapes],
            "sdk_version": self.sdk_version,
        }


def draw_detections(
    bgr_image: np.ndarray,
    detections: list[Detection],
) -> np.ndarray:
    """Return a BGR copy with all detections drawn."""
    annotated = bgr_image.copy()
    for detection in detections:
        x1, y1, x2, y2 = map(round, detection.bbox_xyxy)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 220, 0), 2)
        label = f"{detection.class_name} {detection.confidence:.2f}"
        cv2.putText(
            annotated,
            label,
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 220, 0),
            2,
        )
    return annotated
