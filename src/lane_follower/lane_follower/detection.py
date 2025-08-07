#!/usr/bin/env python3
import numpy as np

class Detection:
    def __init__(self, nwindows=12, margin=60, minpix=5, threshold=100):
        self.nwindows = nwindows
        self.margin = margin
        self.minpix = minpix
        self.threshold = threshold
        self.speed = None

    def sliding_window(self, img):
        # img: 바이너리 흑백(0/255), shape = (y, x)
        y, x = img.shape
        histogram = np.sum(img, axis=0)
        midpoint = x // 2
        leftx_current  = np.argmax(histogram[:midpoint])
        rightx_current = np.argmax(histogram[midpoint:]) + midpoint
        window_height = int(y / self.nwindows)
        nz = img.nonzero()

        left_lane_inds = []
        right_lane_inds = []
        l_err = [0, 0]
        r_err = [0, 0]
        foundl = foundr = False
        lx = []; ly = []; rx = []; ry = []

        # 슬라이딩 윈도우 루프
        for window in range(self.nwindows):
            win_y_low  = y - (window+1)*window_height
            win_y_high = y - window*window_height
            win_xll = leftx_current  - self.margin
            win_xlh = leftx_current  + self.margin
            win_xrl = rightx_current - self.margin
            win_xrh = rightx_current + self.margin

            good_left_inds  = ((nz[0]>=win_y_low)&(nz[0]<win_y_high)&(nz[1]>=win_xll)&(nz[1]<win_xlh)).nonzero()[0]
            good_right_inds = ((nz[0]>=win_y_low)&(nz[0]<win_y_high)&(nz[1]>=win_xrl)&(nz[1]<win_xrh)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # 왼쪽 윈도우 이동
            if len(good_left_inds) > self.minpix:
                if self.threshold < leftx_current < histogram.shape[0]-self.threshold:
                    l_err[1] = 0; foundl = True
                else:
                    l_err[1] += 1
                leftx_current = int(np.mean(nz[1][good_left_inds]))
            else:
                l_err[1] += 1
            if not foundl: l_err[0] += 1

            # 오른쪽 윈도우 이동
            if len(good_right_inds) > self.minpix:
                if self.threshold < rightx_current < histogram.shape[0]-self.threshold:
                    r_err[1] = 0; foundr = True
                else:
                    r_err[1] += 1
                rightx_current = int(np.mean(nz[1][good_right_inds]))
            else:
                r_err[1] += 1
            if not foundr: r_err[0] += 1

            # 결과 저장
            lx.append(leftx_current); ly.append((win_y_low+win_y_high)/2)
            rx.append(rightx_current); ry.append((win_y_low+win_y_high)/2)

        # 최종 인덱스 플랫
        left_lane_inds  = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # l_lane, r_lane 포맷: (x_list, y_list, [fail_count, success_count])
        self.l_lane = (lx, ly, l_err)
        self.r_lane = (rx, ry, r_err)
        return self.l_lane, self.r_lane

    def compute_control(self, x, l_lane, r_lane, midrange=300):
        # 기본 go_forward 로직: 중앙선 계산
        posl = int(np.mean(l_lane[0][1:4]))
        posr = int(np.mean(r_lane[0][1:3]))
        fail_thr = 3
        if max(l_lane[2]) >= fail_thr:
            posl = posr - 286
        pos = (posl + posr)//2
        # 0~1로 정규화
        midstart = x//2 - midrange
        midend   = x//2 + midrange
        ctrl = ((pos-midstart)*(1-0)/(midend-midstart)) + 0
        return ctrl
