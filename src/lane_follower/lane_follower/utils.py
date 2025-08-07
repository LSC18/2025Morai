#!/usr/bin/env python3
import numpy as np

def compute_histogram(img, axis=0):
    """img: 바이너리 이미지 → 합산 히스토그램 반환"""
    return np.sum(img, axis=axis)
