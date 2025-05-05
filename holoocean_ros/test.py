import cv2
import numpy as np

rgba = np.zeros((720, 1280, 4), dtype=np.uint8)
rgba[:, :, 0] = 255  # Blue channel

bgr = rgba[:, :, :3].copy()

cv2.imshow('Image', bgr)
cv2.waitKey(0)
cv2.destroyAllWindows()
