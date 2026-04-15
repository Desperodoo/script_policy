#!/usr/bin/env python3
"""
Teleop shadow validation node.

Reads teleop_target_v2 / events_v2 from lower machine backend, builds a
learning-level human chunk from processed absolute teleop target, and records
JSONL shadow logs without taking control ownership.
"""

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np
import requests
import rospy

carm_deploy_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, carm_deploy_root)

from core.env_ros import RealEnvironment
from data.teleop_shadow_utils import (
    compute_learning_level_chunk,
    extract_processed_target_abs,
    percentile,
)
from utils.timeline_logger import TimelineLogger


class SseMonitor:
    """Background SSE monitor for raw/processed dual-channel events."""

    def __init__(self, url: str):
        self.url = url
        self.session = requests.Session()
        self.session.trust_env = False
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.last_event = None
        self.event_count = 0
        self.heartbeat_count = 0
        self.arrival_gaps_ms = []
        self.last_arrival_time = None
        self.errors = 0

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def snapshot(self):
        with self.lock:
            return {
                "last_event": self.last_event,
                "event_count": self.event_count,
                "heartbeat_count": self.heartbeat_count,
                "arrival_gaps_ms": list(self.arrival_gaps_ms),
                "errors": self.errors,
            }

    def _record_event(self, payload: dict):
        now = time.time()
        with self.lock:
            arrival_gap_ms = None
            if self.last_arrival_time is not None:
                arrival_gap_ms = max(0.0, (now - self.last_arrival_time) * 1000.0)
            self.last_arrival_time = now
            self.last_event = payload
            self.event_count += 1
            if payload.get("heartbeat"):
                self.heartbeat_count += 1
            if arrival_gap_ms is not None:
                self.arrival_gaps_ms.append(arrival_gap_ms)
                if len(self.arrival_gaps_ms) > 2000:
                    self.arrival_gaps_ms = self.arrival_gaps_ms[-2000:]

    def _run(self):
        while self.running and not rospy.is_shutdown():
            try:
                resp = self.session.get(self.url, stream=True, timeout=(1.0, 10.0))
                for raw in resp.iter_lines(decode_unicode=True):
                    if not self.running or rospy.is_shutdown():
                        return
                    if not raw or not raw.startswith("data: "):
                        continue
                    payload = json.loads(raw[6:])
                    self._record_event(payload)
            except Exception as exc:
                rospy.logwarn_throttle(5.0, f"SSE monitor error: {exc}")
                with self.lock:
                    self.errors += 1
                time.sleep(0.5)


class TeleopShadowNode:
    def __init__(self, config):
        self.config = config
        self.robot_ip = config["robot_ip"]
        self.backend_url = config["backend_url"]
        self.pred_horizon = int(config["pred_horizon"])
        self.act_horizon = int(config["act_horizon"])
        self.shadow_freq = float(config["shadow_freq"])
        self.enable_sse = bool(config["enable_sse"])
        self.summary_interval = float(config["summary_interval"])

        env_config = {
            "robot_ip": self.robot_ip,
            "robot_mode": config.get("robot_mode", 2),
            "robot_tau": config.get("robot_tau", 10.0),
            "passive_mode": True,
            "skip_init_confirm": True,
            "return_to_zero": False,
        }
        self.env = RealEnvironment(env_config)

        output_dir = os.path.expanduser(os.path.expandvars(config["output_dir"]))
        os.makedirs(output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = config.get("log_path") or os.path.join(output_dir, f"teleop_shadow_{ts}.jsonl")
        self.timeline_logger = TimelineLogger(self.log_path)

        self.http = requests.Session()
        self.http.trust_env = False
        self.sse_monitor = SseMonitor(config["events_v2_url"]) if self.enable_sse else None
        if self.sse_monitor is not None:
            self.sse_monitor.start()
        self._last_logged_sse_signature = None

        self.stats = {
            "samples": 0,
            "active_samples": 0,
            "inactive_segments": 0,
            "prev_active": False,
            "http_latencies_ms": [],
            "pos_errors": [],
            "rot_errors": [],
            "exceptions": 0,
        }
        self.last_summary_t = time.time()

    def shutdown(self):
        if self.sse_monitor is not None:
            self.sse_monitor.stop()
        self.timeline_logger.close()
        self.env.shutdown()

    def run(self):
        rospy.loginfo(f"Teleop shadow log path: {self.log_path}")
        rate = rospy.Rate(self.shadow_freq)
        while not rospy.is_shutdown():
            try:
                self.step()
            except Exception as exc:
                self.stats["exceptions"] += 1
                rospy.logerr(f"Teleop shadow step failed: {exc}")
            rate.sleep()

    def step(self):
        obs = self.env.get_state_observation()
        if obs is None:
            return

        t_recv_sys = time.time()
        t0 = time.time()
        teleop_state_v2 = self.env.get_teleop_action_v2(self.backend_url)
        latency_ms = (time.time() - t0) * 1000.0

        self.stats["samples"] += 1
        self.stats["http_latencies_ms"].append(latency_ms)
        if len(self.stats["http_latencies_ms"]) > 4000:
            self.stats["http_latencies_ms"] = self.stats["http_latencies_ms"][-4000:]

        processed = (teleop_state_v2 or {}).get("processed") or {}
        raw = (teleop_state_v2 or {}).get("raw") or {}
        backend_server_timestamp = (teleop_state_v2 or {}).get("server_timestamp")
        teleop_active = bool(processed.get("active"))
        if teleop_active:
            self.stats["active_samples"] += 1
        elif self.stats["prev_active"]:
            self.stats["inactive_segments"] += 1
        self.stats["prev_active"] = teleop_active

        payload = {
            "event_kind": "snapshot",
            "transport_mode": "http",
            "obs_stamp_ros": obs["stamp"],
            "t_recv_sys": t_recv_sys,
            "backend_server_timestamp": backend_server_timestamp,
            "processed_sequence": processed.get("sequence"),
            "raw_sequence": raw.get("sequence"),
            "teleop_active": teleop_active,
            "processed_target_abs": processed.get("target_pose_abs"),
            "processed_gripper": processed.get("gripper_pose"),
            "ref_pose": obs["qpos_end"][:7],
            "http_or_sse_latency_ms": latency_ms,
        }

        target_pose_abs = extract_processed_target_abs(teleop_state_v2)
        if target_pose_abs is not None:
            chunk = compute_learning_level_chunk(obs["qpos_end"][:7], target_pose_abs, self.pred_horizon)
            payload.update({
                "human_chunk_abs_first": chunk["human_chunk_abs"][0],
                "human_chunk_rel_first": chunk["human_chunk_rel"][0],
                "reconstructed_target_abs_first": chunk["reconstructed_target_abs"][0],
                "abs_reconstruction_pos_error": chunk["abs_reconstruction_pos_error"],
                "abs_reconstruction_rot_error": chunk["abs_reconstruction_rot_error"],
            })
            self.stats["pos_errors"].append(chunk["abs_reconstruction_pos_error"])
            self.stats["rot_errors"].append(chunk["abs_reconstruction_rot_error"])
            if len(self.stats["pos_errors"]) > 4000:
                self.stats["pos_errors"] = self.stats["pos_errors"][-4000:]
                self.stats["rot_errors"] = self.stats["rot_errors"][-4000:]
        else:
            payload.update({
                "human_chunk_abs_first": None,
                "human_chunk_rel_first": None,
                "reconstructed_target_abs_first": None,
                "abs_reconstruction_pos_error": None,
                "abs_reconstruction_rot_error": None,
            })

        self.timeline_logger.log("teleop_shadow", **payload)
        self._maybe_log_sse_event()
        self._maybe_print_summary()

    def _maybe_log_sse_event(self):
        if self.sse_monitor is None:
            return
        snap = self.sse_monitor.snapshot()
        last_event = snap.get("last_event")
        if not last_event:
            return
        processed = last_event.get("processed") or {}
        raw = last_event.get("raw") or {}
        signature = (
            processed.get("sequence"),
            raw.get("sequence"),
            bool(last_event.get("heartbeat")),
        )
        if signature == self._last_logged_sse_signature:
            return
        self._last_logged_sse_signature = signature
        self.timeline_logger.log(
            "teleop_shadow_sse",
            event_kind="heartbeat" if last_event.get("heartbeat") else "sse",
            transport_mode="sse",
            backend_server_timestamp=last_event.get("server_timestamp"),
            processed_sequence=processed.get("sequence"),
            raw_sequence=raw.get("sequence"),
            teleop_active=processed.get("active", False),
            processed_target_abs=processed.get("target_pose_abs"),
            processed_gripper=processed.get("gripper_pose"),
            http_or_sse_latency_ms=None,
        )

    def _maybe_print_summary(self):
        now = time.time()
        if now - self.last_summary_t < self.summary_interval:
            return
        self.last_summary_t = now
        samples = max(self.stats["samples"], 1)
        active_ratio = self.stats["active_samples"] / samples
        http_lat = self.stats["http_latencies_ms"]
        pos_err = self.stats["pos_errors"]
        rot_err = self.stats["rot_errors"]
        sse_snap = self.sse_monitor.snapshot() if self.sse_monitor is not None else None
        sse_events = sse_snap["event_count"] if sse_snap is not None else 0
        sse_arrival_gaps = sse_snap["arrival_gaps_ms"] if sse_snap is not None else []
        heartbeat_count = sse_snap["heartbeat_count"] if sse_snap is not None else 0
        sse_errors = sse_snap["errors"] if sse_snap is not None else 0

        rospy.loginfo(
            "teleop_shadow "
            f"active_ratio={active_ratio:.1%} "
            f"http_mean={np.mean(http_lat):.2f}ms http_p95={percentile(http_lat, 95) or 0.0:.2f}ms "
            f"sse_events={sse_events} heartbeat_count={heartbeat_count} "
            f"sse_gap_p95={(percentile(sse_arrival_gaps, 95) or 0.0):.2f}ms sse_errors={sse_errors} "
            f"pos_err_mean={(np.mean(pos_err) if pos_err else 0.0):.6f} "
            f"pos_err_max={(np.max(pos_err) if pos_err else 0.0):.6f} "
            f"rot_err_mean={(np.mean(rot_err) if rot_err else 0.0):.6f} "
            f"inactive_segments={self.stats['inactive_segments']} "
            f"exceptions={self.stats['exceptions']}"
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Teleop shadow validation node")
    parser.add_argument("--robot_ip", type=str, default="10.42.0.101")
    parser.add_argument("--robot_mode", type=int, default=2)
    parser.add_argument("--robot_tau", type=float, default=10.0)
    parser.add_argument("--backend_url", type=str, default="")
    parser.add_argument("--events_v2_url", type=str, default="")
    parser.add_argument("--output_dir", type=str, default="./teleop_shadow_logs")
    parser.add_argument("--log_path", type=str, default="")
    parser.add_argument("--pred_horizon", type=int, default=16)
    parser.add_argument("--act_horizon", type=int, default=8)
    parser.add_argument("--control_freq", type=int, default=50)
    parser.add_argument("--chunk_time_base", type=str, default="sys_time")
    parser.add_argument("--shadow_freq", type=float, default=20.0)
    parser.add_argument("--summary_interval", type=float, default=5.0)
    parser.add_argument("--enable_sse", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("teleop_shadow", anonymous=False)

    backend_url = args.backend_url or f"http://{args.robot_ip}:1999/api/joystick/teleop_target_v2"
    events_v2_url = args.events_v2_url or f"http://{args.robot_ip}:1999/api/joystick/events_v2"
    config = {
        "robot_ip": args.robot_ip,
        "robot_mode": args.robot_mode,
        "robot_tau": args.robot_tau,
        "backend_url": backend_url,
        "events_v2_url": events_v2_url,
        "output_dir": args.output_dir,
        "log_path": args.log_path,
        "pred_horizon": args.pred_horizon,
        "act_horizon": args.act_horizon,
        "control_freq": args.control_freq,
        "chunk_time_base": args.chunk_time_base,
        "shadow_freq": args.shadow_freq,
        "summary_interval": args.summary_interval,
        "enable_sse": args.enable_sse,
    }

    node = TeleopShadowNode(config)
    try:
        node.run()
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
