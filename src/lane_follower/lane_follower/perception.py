#!/usr/bin/env python3
import cv2
import numpy as np

class Perception:
    def __init__(self):
        self.img = None
        self.img_hsv = None
        self.h = self.s = self.v = None
        self.y = self.x = None
        self.yellow_range = None
        self.white_range = None
        self.combined_range = None
        self.warped_img = None
        self.speed = 1800  # 기본 속도

    def img_init(self, img):
        # 원본 BGR -> HSV & 크기 저장
        self.img = img
        self.y, self.x = img.shape[:2]
        self.img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.h, self.s, self.v = cv2.split(self.img_hsv)

    def img_transform(self):
        # 노란선 / 흰선 마스크 생성
        lower_yellow = np.array([15, 100, 140])
        upper_yellow = np.array([30, 200, 255])
        self.yellow_range = cv2.inRange(self.img_hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 140])
        upper_white = np.array([50, 70, 255])
        self.white_range = cv2.inRange(self.img_hsv, lower_white, upper_white)

        # 합치기
        self.combined_range = cv2.bitwise_or(self.yellow_range, self.white_range)
        self.img = self.combined_range

    def img_warp(self):
        # 투시 변환 (원본 코드 그대로)
        y, x = self.y, self.x
        img = self.img
        warp_img_size = [x, x]
        warp_img_zoomx = x // 4

        # 원본 ROI
        bottomx, bottomy = -25, y
        topx, topy       = 269, 271
        src_points = np.float32([
            [bottomx, bottomy],
            [ topx,    topy ],
            [ x-topx,  topy ],
            [ x-bottomx, bottomy ]
        ])
        # 투영 대상
        bx, tx = warp_img_zoomx, warp_img_zoomx
        by, ty = x, x//4
        dst_points = np.float32([
            [bx, by],
            [tx, ty],
            [x-tx, ty],
            [x-bx, by]
        ])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        self.warped_img = cv2.warpPerspective(img, matrix, tuple(warp_img_size))
        self.img = self.warped_img
