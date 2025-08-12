#!/usr/bin/env python3
import numpy as np

class Detection:
    def __init__(self, nwindows=12, margin=60, minpix=5, threshold=100):
        self.nwindows = nwindows
        self.margin = margin
        self.minpix = minpix
        self.threshold = threshold
        self.speed = None
        
    def _apply_roi(self, img, mode):
        """좌/우 전용 모드일 때만 간단 폴리곤 ROI로 반대쪽 영역 컷"""
        if mode == "normal":
            return img

        h, w = img.shape
        mask = np.zeros_like(img, dtype=np.uint8)

        if mode == "left_only":
            # 좌회전: 오른쪽/우상향 영역을 줄이기 위한 폴리곤 (값은 맵에 맞게 미세조정)
            pts = np.array([[
                (0, h), (int(0.60*w), h), (int(0.30*w), int(0.5*h)), (0, int(0.0*h))
            ]], dtype=np.int32)

        elif mode == "right_only":
            # 우회전: 왼쪽/좌상향 영역 컷
            pts = np.array([[
                (int(0.40*w), h), (w, h), (w, int(0.3*h)), (int(0.70*w), int(0.65*h))
            ]], dtype=np.int32)

        cv2 = __import__("cv2")  # (외부 import 안 건드리기용)
        cv2.fillPoly(mask, pts, 255)
        return cv2.bitwise_and(img, mask)

    def _slope_ok(self, xs, ys, mode, k_thr=0.05):
        """
        라인의 방향 제약. 좌회전이면 dx/dy< -k_thr, 우회전이면 dx/dy> +k_thr.
        포인트 적으면 True(제외 안 함).
        """
        import numpy as np
        if len(xs) < 5:
            return True
        xs = np.asarray(xs); ys = np.asarray(ys)
        idx = np.argsort(ys)     # y 오름차순 정렬
        k = np.polyfit(ys[idx], xs[idx], 1)[0]  # dx/dy (1차 기울기)
        if mode == "left_only":
            return (k < -k_thr)
        if mode == "right_only":
            return (k >  k_thr)
        return True

    def _repr_x(self, xs):
        import numpy as np
        if len(xs) >= 3:
            return int(np.mean(xs[1:4]))
        return int(np.mean(xs)) if len(xs) > 0 else None

    def sliding_window(self, img, mode="normal"):
        # img: 바이너리 흑백(0/255), shape = (y, x)
        # ROI 마스킹
        img = self._apply_roi(img, mode)
        
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

        # 기울기 제약
        if mode == "left_only" and not self._slope_ok(lx, ly, "left_only"):
            rx, ry = [], []; r_err = [999, 999]
        if mode == "right_only" and not self._slope_ok(rx, ry, "right_only"):
            lx, ly = [], []; l_err = [999, 999]  # 사실상 무효로 처리
            
        # l_lane, r_lane 포맷: (x_list, y_list, [fail_count, success_count])
        self.l_lane = (lx, ly, l_err)
        self.r_lane = (rx, ry, r_err)
        return self.l_lane, self.r_lane, img

    # def compute_control(self, x, l_lane, r_lane, midrange=300, bias=0.0):
    #     # 기본 go_forward 로직: 중앙선 계산
    #     posl = int(np.mean(l_lane[0][1:4]))
    #     posr = int(np.mean(r_lane[0][1:3]))
    #     fail_thr = 3
    #     if max(l_lane[2]) >= fail_thr:
    #         posl = posr - 286
    #     pos = (posl + posr)//2
    #     # bias 적용
    #     pos += int(bias * midrange)
    #     # 0~1로 정규화
    #     midstart = x//2 - midrange
    #     midend   = x//2 + midrange
    #     ctrl = ((pos-midstart)*(1-0)/(midend-midstart)) + 0
    #     return ctrl
    
    def compute_control(self, x, l_lane, r_lane,
                        midrange=300,
                        mode="normal",           # "normal" | "left_only" | "right_only"
                        lane_width_px=286,       # 워프 기준 차선폭(픽셀) - 실측 후 보정 권장
                        delta_px=20):            # 차로 중앙 쪽으로 살짝 더 넣는 보정(픽셀)

        """
        x            : 이미지 폭(픽셀)
        l_lane/r_lane: (x_list, y_list, fail_list)
        midrange     : 조향 민감도 범위(픽셀)
        mode         : "normal"(기존 로직) | "left_only"(좌회전 전용) | "right_only"(우회전 전용)
        lane_width_px: BEV 상 차선 간격(픽셀)
        delta_px     : 차로 중앙쪽으로 더 파고드는 보정치(+는 중앙쪽)
        return       : ctrl ∈ [0,1]
        """

        fail_thr = 3
        # 좌/우 라인 유효 여부
        hasL = (len(l_lane[0]) > 0) and (max(l_lane[2]) < fail_thr)
        hasR = (len(r_lane[0]) > 0) and (max(r_lane[2]) < fail_thr)

        # --- 모드 분기 ---
        if mode == "left_only":
            xL = self._repr_x(l_lane[0])
            pos = xL + lane_width_px // 2 - delta_px

        elif mode == "right_only":
            xR = self._repr_x(r_lane[0])
            pos = xR - lane_width_px // 2 - delta_px

        else:
            # ---- 기존 로직 그대로 ----
            posl = self._repr_x(l_lane[0])
            posr = self._repr_x(r_lane[0])
            # 한쪽 실패 보정(왼쪽 못찾으면 오른쪽에서 차선폭 가정)
            if not hasL:
                posl = posr - lane_width_px

            pos = (posl + posr) // 2

        # --- 0~1 정규화(그대로) ---
        midstart = x // 2 - midrange
        midend   = x // 2 + midrange
        ctrl = (pos - midstart) / float(midend - midstart)
        # 클램프
        if ctrl < 0.0: ctrl = 0.0
        if ctrl > 1.0: ctrl = 1.0
        return ctrl
    
    def detect_stopline(self, img, band_from_bottom=80, band_height=30,
                        row_ratio_thr=0.40, hold_frames=3):
        """
        img: BEV 이진영상 (H x W)
        band_from_bottom: 하단에서부터 밴드 시작 위치(px)
        band_height: 검사할 밴드 높이(px)
        row_ratio_thr: 밴드 내 '한 줄'이라도 흰 픽셀 비율이 이 값 이상이면 정지선으로 판단
                       (ex) 0.55면 해당 줄의 55% 이상이 흰색이면 정지선
        hold_frames: 히스테리시스(디바운스)용. 연속 N프레임 이상 조건 만족 시 True

        반환: bool (정지선)
        """
        h, w = img.shape[:2]
        y1 = max(0, h - band_from_bottom - band_height)
        y2 = max(0, h - band_from_bottom)
        band = img[y1:y2, :]  # 하단 밴드

        if band.size == 0:
            return False

        # 각 행의 흰 픽셀 비율
        row_white_ratio = (band > 0).sum(axis=1) / float(w)

        # 한 행이라도 임계 초과 → 후보
        hit = np.any(row_white_ratio >= row_ratio_thr)

        # 간단한 히스테리시스(연속 프레임 보정)
        if not hasattr(self, "_stop_hit_cnt"):
            self._stop_hit_cnt = 0
        if not hasattr(self, "_stop_state"):
            self._stop_state = False

        if hit:
            self._stop_hit_cnt += 1
        else:
            self._stop_hit_cnt = 0

        if self._stop_hit_cnt >= hold_frames:
            self._stop_state = True
        if not hit and self._stop_state:
            # 조건이 깨진 프레임이 조금 이어지면 해제하고 싶으면 여기에 down-count 로직 추가 가능
            self._stop_state = False

        return self._stop_state


    def shape_steer(self, steer_in, p=1.6, scale=5.0):
        e = steer_in - 0.5
        y = (abs(e) ** p) * np.sign(e)* scale
        steer_out = 0.5 + y
        return max(0.0, min(1.0, steer_out))