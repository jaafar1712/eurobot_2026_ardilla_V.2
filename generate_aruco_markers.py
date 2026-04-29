#!/usr/bin/env python3
"""Regenerate ArUco marker PNGs with white backgrounds for Gazebo crate models."""

import cv2
import cv2.aruco as aruco
import numpy as np
import os

MARKER_SIZE_PX = 400
BORDER_BITS = 1
OUTPUT_SIZE = 512

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

markers = [
    (36, 'crate/materials/textures/aruco_blue_36.png'),
    (47, 'crate/materials/textures/aruco_yellow_47.png'),
    (36, 'crate_yellow/materials/textures/aruco_blue_36.png'),
    (47, 'crate_yellow/materials/textures/aruco_yellow_47.png'),
]

base = os.path.join(os.path.dirname(__file__),
                    'src', 'mam_eurobot_2026', 'models')

for marker_id, rel_path in markers:
    img = aruco.generateImageMarker(aruco_dict, marker_id, OUTPUT_SIZE)
    out_path = os.path.join(base, rel_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, img)
    print(f"Generated ID {marker_id} → {out_path}")

print("Done.")
