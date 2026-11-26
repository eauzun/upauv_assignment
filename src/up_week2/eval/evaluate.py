#!/usr/bin/env python3
"""Tespit basarim olcumu.

Kullanim:
  ros2 run up_week2 evaluate.py --ros-args -p lighting:=normal -p frames:=400
Yer gercegi: /target_mover/ground_truth (dunya koordinati), kamera SDF'ten.
"""
import math
import os

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

W, H, FOV = 640, 480, 1.0
CAM = (0.0, -3.8, -1.0)
YAW, PITCH = 1.5, 0.25
PILLAR = (-0.65, -1.2, 0.24)      # x, y, yaricap
TARGET_HALF = 0.22

F = (W / 2.0) / math.tan(FOV / 2.0)
CY, SY, CP, SP = math.cos(YAW), math.sin(YAW), math.cos(PITCH), math.sin(PITCH)
FWD = (CP * CY, CP * SY, -SP)
LEFT = (-SY, CY, 0.0)
UP = (SP * CY, SP * SY, CP)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def project(p):
    d = (p[0] - CAM[0], p[1] - CAM[1], p[2] - CAM[2])
    xc = dot(d, FWD)
    if xc <= 0.1:
        return None
    return (W / 2.0 - F * dot(d, LEFT) / xc,
            H / 2.0 - F * dot(d, UP) / xc,
            xc)


def pillar_dist(p):
    """Kamera->hedef dogrusuna sutun ekseninin uzakligi (sutun onde degilse sonsuz)."""
    dx, dy = p[0] - CAM[0], p[1] - CAM[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return 1e9
    along = ((PILLAR[0] - CAM[0]) * dx + (PILLAR[1] - CAM[1]) * dy) / length
    if along <= 0.0 or along > length - 0.15:
        return 1e9
    return abs((PILLAR[0] - CAM[0]) * dy - (PILLAR[1] - CAM[1]) * dx) / length


def classify(p):
    pr = project(p)
    if pr is None:
        return 'invisible', None
    u, v, xc = pr
    half = F * TARGET_HALF / xc
    if u < -half or u > W + half or v < -half or v > H + half:
        return 'invisible', pr
    dist = pillar_dist(p)
    if dist < 0.02:
        return 'invisible', pr
    inside = half <= u <= W - half and half <= v <= H - half
    if inside and dist > PILLAR[2] + TARGET_HALF:
        return 'visible', pr
    return 'ambiguous', pr


class Evaluator(Node):
    def __init__(self):
        super().__init__('detection_evaluator')
        self.label = self.declare_parameter('lighting', 'normal').value
        self.limit = self.declare_parameter('frames', 400).value
        self.out_dir = self.declare_parameter('out_dir', '.').value
        self.gt = None
        self.frames = 0
        self.vis = self.vis_hit = 0
        self.inv = self.inv_fp = 0
        self.amb = 0
        self.settle_n = self.declare_parameter('settle_frames', 4).value
        self.settle = 0
        self.last_def = None
        self.err_sum = 0.0
        self.done = False
        self.create_subscription(PointStamped, '/target_mover/ground_truth',
                                 self.on_gt, 10)
        self.create_subscription(Detection2DArray, '/vision/detections',
                                 self.on_det, 10)
        self.get_logger().info(
            f'evaluating "{self.label}" for {self.limit} frames '
            '(waiting for ground truth)')

    def on_gt(self, msg):
        self.gt = (msg.point.x, msg.point.y, msg.point.z)

    def on_det(self, msg):
        if self.gt is None or self.done:
            return
        kind, pr = classify(self.gt)
        self.frames += 1
        if kind != 'ambiguous':
            if kind != self.last_def:
                self.settle = self.settle_n
                self.last_def = kind
            if self.settle > 0:
                self.settle -= 1
                kind = 'ambiguous'
        if kind == 'visible':
            self.vis += 1
            if msg.detections:
                self.vis_hit += 1
                c = msg.detections[0].bbox.center.position
                self.err_sum += math.hypot(c.x - pr[0], c.y - pr[1])
        elif kind == 'invisible':
            self.inv += 1
            if msg.detections:
                self.inv_fp += 1
        else:
            self.amb += 1
        if self.frames >= self.limit:
            self.finish()

    def finish(self):
        self.done = True
        rate = 100.0 * self.vis_hit / self.vis if self.vis else float('nan')
        fp = 100.0 * self.inv_fp / self.inv if self.inv else float('nan')
        err = self.err_sum / self.vis_hit if self.vis_hit else float('nan')
        ok = lambda c: 'PASS' if c else 'FAIL'
        lines = [
            f'lighting            : {self.label}',
            f'frames              : {self.frames} (visible={self.vis}, '
            f'invisible={self.inv}, ambiguous/skipped={self.amb})',
            f'detection rate      : {rate:.1f} %  (>= 90)  {ok(rate >= 90)}',
            f'false detection rate: {fp:.1f} %  (<= 5)   {ok(fp <= 5)}',
            f'mean center error   : {err:.2f} px (<= 10)  {ok(err <= 10)}',
        ]
        text = '\n'.join(lines)
        print('\n' + text + '\n')
        path = os.path.join(self.out_dir, f'results_{self.label}.txt')
        try:
            with open(path, 'w') as fh:
                fh.write(text + '\n')
            print(f'written: {path}')
        except OSError as exc:
            print(f'could not write {path}: {exc}')


def main():
    rclpy.init()
    node = Evaluator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
