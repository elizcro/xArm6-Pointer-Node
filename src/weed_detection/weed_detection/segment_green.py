import cv2
import numpy as np

def merge_close_boxes(bboxes: np.ndarray, gap: int = 40) -> np.ndarray:
    """Merge boxes whose rectangles are within `gap` px in both axes.

    Collapses leaf-blobs of one plant (small gap) into a single box while leaving
    distinct plants (large gap) separate. In/out are (N, 4) as (x, y, w, h).
    """
    if bboxes.shape[0] <= 1:
        return bboxes

    boxes = [[int(x), int(y), int(x + w), int(y + h)] for (x, y, w, h) in bboxes]
    changed = True
    while changed:
        changed = False
        out, used = [], [False] * len(boxes)
        for i in range(len(boxes)):
            if used[i]:
                continue
            ax1, ay1, ax2, ay2 = boxes[i]
            for j in range(i + 1, len(boxes)):
                if used[j]:
                    continue
                bx1, by1, bx2, by2 = boxes[j]
                dx = max(0, max(ax1, bx1) - min(ax2, bx2))
                dy = max(0, max(ay1, by1) - min(ay2, by2))
                if dx <= gap and dy <= gap:
                    ax1, ay1 = min(ax1, bx1), min(ay1, by1)
                    ax2, ay2 = max(ax2, bx2), max(ay2, by2)
                    used[j] = changed = True
            used[i] = True
            out.append([ax1, ay1, ax2, ay2])
        boxes = out

    return np.array([[x1, y1, x2 - x1, y2 - y1] for (x1, y1, x2, y2) in boxes],
                    dtype=bboxes.dtype)

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

    hsv_mask = cv2.inRange(hsv, (h_low, 60, 40), (h_high, 255, 255))

    exg = 2 * bgr[:, :, 1] - bgr[:, :, 2] - bgr[:, :, 0]
    exg_mask = cv2.inRange(exg, exg_low, exg_high)

    mask = hsv_mask & exg_mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))

    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    areas = stats[1:, cv2.CC_STAT_AREA]
    keep = areas >= min_area

    small = np.where(~keep)[0] + 1
    if len(small) != 0:
        mask[np.isin(labels, small)] = 0

    kept_stats = stats[1:][keep]
    bboxes = kept_stats[:, :4]
    bboxes = merge_close_boxes(bboxes, gap=40)

    mask = np.clip(mask, 0, 1).astype(np.uint8)
    
    return mask, bboxes
