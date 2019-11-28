import cv2
import numpy as np


def red_mask(hsv):  # type: (np.ndarray) -> np.ndarray
    mask_low = cv2.inRange(hsv, np.asarray([0, 70, 40]), np.asarray([10, 255, 255])) > 0.0
    mask_high = cv2.inRange(hsv, np.asarray([170, 70, 40]), np.asarray([180, 255, 255])) > 0.0
    return mask_low | mask_high


def green_mask(hsv):  # type: (np.ndarray) -> np.ndarray
    return cv2.inRange(hsv, np.asarray([40, 70, 20]), np.asarray([90, 255, 255])) > 0.0


def get_largest_contour(mask):  # type: (np.ndarray) -> np.ndarray
        contours, _ = cv2.findContours(
            mask.astype(np.uint8),
            mode=cv2.RETR_EXTERNAL,
            method=cv2.CHAIN_APPROX_SIMPLE,
        )
        areas = [
            cv2.contourArea(c) for c in contours
        ]
        return contours[np.argmax(areas)]


def crop_to_bb(img, bb):
    x, y, w, h = bb
    return img[y:y+h, x:x+w]


def isolate_shape(img):
    img = np.asarray(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    img = red_mask(hsv)
    img = crop_to_bb(img, cv2.boundingRect(get_largest_contour(img)))
    img = cv2.resize(img.astype('float32'), (28, 28))
    img = np.expand_dims(img, 0)
    return img


def find_boxes_to_track(mask):
    contours, _ = cv2.findContours(
        mask.astype(np.uint8) * 255,
        mode=cv2.RETR_EXTERNAL,
        method=cv2.CHAIN_APPROX_SIMPLE,
    )
    boxes = []
    for contour in contours:
        if cv2.contourArea(contour) < 100.0:
            continue
        boxes.append(tuple(int(i) for i in cv2.boundingRect(contour)))
    return boxes
