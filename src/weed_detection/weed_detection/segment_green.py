import cv2
import numpy as np


def segment_and_box_green(bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Segment green vegetation and return (binary_mask, bounding_boxes).

    bounding_boxes is shape (N, 4) with each row as (x, y, width, height).
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV_FULL)

    exg_low = 13
    exg_high = 200
    h_low = 50
    h_high = 150
    min_area = 100

    hsv_mask = cv2.inRange(hsv, (h_low, 0, 0), (h_high, 255, 255))

    exg = 2 * bgr[:, :, 1] - bgr[:, :, 2] - bgr[:, :, 0]
    exg_mask = cv2.inRange(exg, exg_low, exg_high)

    mask = hsv_mask & exg_mask

    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    areas = stats[1:, cv2.CC_STAT_AREA]
    keep = areas >= min_area

    small = np.where(~keep)[0] + 1
    if len(small) != 0:
        mask[np.isin(labels, small)] = 0

    kept_stats = stats[1:][keep]
    bboxes = kept_stats[:, :4]

    mask = np.clip(mask, 0, 1).astype(np.uint8)

    return mask, bboxes
