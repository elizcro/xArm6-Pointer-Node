import numpy as np
from sklearn.cluster import DBSCAN

from .segment_green import segment_and_box_green


def detect_weeds(img: np.ndarray) -> np.ndarray:
    """Return (N, 2) array of bounding-box center points in pixel coordinates."""
    _, bboxes = segment_and_box_green(img)
    if bboxes.shape[0] == 0:
        return np.empty((0, 2), dtype=np.float32)
    centers = np.hstack([
        (bboxes[:, 0] + bboxes[:, 2] / 2).reshape(-1, 1),
        (bboxes[:, 1] + bboxes[:, 3] / 2).reshape(-1, 1),
    ])
    return centers


def choose_weeds(samples: list[np.ndarray], min_fraction: float = 0.8, eps: float = 5.0) -> np.ndarray:
    """Cluster detections across multiple frames with DBSCAN.

    Returns (M, 2) array of cluster centroids in pixel coordinates.
    """
    non_empty = [s for s in samples if s.shape[0] > 0]
    if len(non_empty) == 0:
        return np.empty((0, 2), dtype=np.float64)

    combined = np.vstack(non_empty)
    min_samples = max(1, int(len(samples) * min_fraction))

    db = DBSCAN(eps=eps, min_samples=min_samples)
    db.fit(combined)

    labels = db.labels_
    centroids = []
    for label in set(labels):
        if label == -1:
            continue
        centroids.append(combined[labels == label].mean(axis=0))

    if len(centroids) == 0:
        return np.empty((0, 2), dtype=np.float64)
    return np.array(centroids)
