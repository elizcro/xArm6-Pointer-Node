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

    if logger is not None:
        n_noise = int(np.sum(labels == -1))
        cluster_ids = sorted(l for l in set(labels) if l != -1)
        logger.info(f'[choose_weeds] {combined.shape[0]} pts / {len(samples)} frames | '
                    f'eps={eps} min_samples={min_samples} | '
                    f'{len(cluster_ids)} clusters, {n_noise} noise')
        for cid in cluster_ids:
            pts = combined[labels == cid]
            c = pts.mean(axis=0)
            span = pts.max(axis=0) - pts.min(axis=0)
            logger.info(f'    cluster {cid}: n={pts.shape[0]:>3} '
                        f'centroid=({c[0]:.0f},{c[1]:.0f}) span=({span[0]:.0f}x{span[1]:.0f})px')
        if n_noise:
            noise = combined[labels == -1]
            nc = noise.mean(axis=0)
            logger.info(f'    noise: n={n_noise} mean=({nc[0]:.0f},{nc[1]:.0f}) '
                        f'x[{noise[:,0].min():.0f}-{noise[:,0].max():.0f}] '
                        f'y[{noise[:,1].min():.0f}-{noise[:,1].max():.0f}]')

    centroids = []
    for label in set(labels):
        if label == -1:
            continue
        centroids.append(combined[labels == label].mean(axis=0))

    if len(centroids) == 0:
        return np.empty((0, 2), dtype=np.float64)
    return np.array(centroids)
