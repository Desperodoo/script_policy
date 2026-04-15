#!/usr/bin/env python3
"""
Shared teleop uplift utilities for record_data_ros and future HITL flows.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional

import numpy as np
import requests
import rospy

from data.teleop_shadow_utils import (
    compute_learning_level_chunk,
    extract_processed_target_abs,
    quaternion_angle_distance,
)


def _replace_api_suffix(url: str, old_suffix: str, new_suffix: str) -> str:
    if not url:
        return url
    if url.endswith(old_suffix):
        return url[: -len(old_suffix)] + new_suffix
    return url


def _safe_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _safe_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


class SseMonitor:
    """Background SSE monitor for dual-channel teleop events."""

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
        if self.running or not self.url:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def snapshot(self) -> Dict[str, Any]:
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
                rospy.logwarn_throttle(5.0, f"Teleop SSE monitor error: {exc}")
                with self.lock:
                    self.errors += 1
                time.sleep(0.5)


class TeleopSignalClient:
    """HTTP/SSE client for lower-machine teleop state and control ownership."""

    def __init__(
        self,
        robot_ip: str,
        backend_url_v2: str = "",
        events_v2_url: str = "",
        control_state_url: str = "",
        enable_sse: bool = False,
        timeout_s: float = 0.05,
    ):
        self.robot_ip = robot_ip
        self.backend_url_v2 = backend_url_v2 or f"http://{robot_ip}:1999/api/joystick/teleop_target_v2"
        self.events_v2_url = events_v2_url or _replace_api_suffix(
            self.backend_url_v2, "/teleop_target_v2", "/events_v2"
        )
        self.control_state_url = control_state_url or _replace_api_suffix(
            self.backend_url_v2, "/teleop_target_v2", "/control_state"
        )
        self.timeout_s = timeout_s
        self.session = requests.Session()
        self.session.trust_env = False
        self.sse_monitor = SseMonitor(self.events_v2_url) if enable_sse else None
        if self.sse_monitor is not None:
            self.sse_monitor.start()
        self._last_snapshot_recv_sys = None
        self._last_processed_sequence = None

    def close(self):
        if self.sse_monitor is not None:
            self.sse_monitor.stop()

    def fetch_state_v2(self) -> Optional[dict]:
        try:
            resp = self.session.get(self.backend_url_v2, timeout=self.timeout_s)
            if resp.status_code == 200:
                body = resp.json()
                return body.get("data", {})
        except requests.RequestException:
            pass
        return None

    def get_control_state(self) -> dict:
        fallback = {
            "local_control_enabled": None,
            "control_owner": None,
            "updated_at": None,
        }
        try:
            resp = self.session.get(self.control_state_url, timeout=self.timeout_s)
            if resp.status_code == 200:
                body = resp.json()
                data = body.get("data", {})
                fallback.update(data)
        except requests.RequestException:
            pass
        return fallback

    def set_control_state(
        self,
        local_control_enabled: bool,
        control_owner: str,
        timeout_s: Optional[float] = None,
    ) -> dict:
        payload = {
            "local_control_enabled": bool(local_control_enabled),
            "control_owner": control_owner,
        }
        resp = self.session.post(
            self.control_state_url,
            json=payload,
            timeout=self.timeout_s if timeout_s is None else float(timeout_s),
        )
        body = resp.json()
        if resp.status_code != 200 or body.get("code") != 1:
            raise RuntimeError(body.get("msg", "set control_state failed"))
        return body.get("data", {})

    def fetch_snapshot(self) -> dict:
        t_recv_sys = time.time()
        t0 = time.time()
        teleop_state_v2 = self.fetch_state_v2()
        http_latency_ms = (time.time() - t0) * 1000.0

        processed = (teleop_state_v2 or {}).get("processed") or {}
        raw = (teleop_state_v2 or {}).get("raw") or {}
        control_state = (teleop_state_v2 or {}).get("control_state") or {}
        processed_sequence = _safe_int(processed.get("sequence"))
        signal_age_ms = _safe_float(processed.get("source_age_ms"))
        if signal_age_ms is None:
            signal_age_ms = _safe_float(raw.get("source_age_ms"))
        if signal_age_ms is None and self._last_snapshot_recv_sys is not None:
            sequence_changed = (
                processed_sequence is not None
                and self._last_processed_sequence is not None
                and processed_sequence != self._last_processed_sequence
            )
            arrival_gap_ms = max(0.0, (t_recv_sys - self._last_snapshot_recv_sys) * 1000.0)
            signal_age_ms = 0.0 if sequence_changed else arrival_gap_ms
        self._last_snapshot_recv_sys = t_recv_sys
        self._last_processed_sequence = processed_sequence

        return {
            "teleop_state_v2": teleop_state_v2,
            "processed": processed,
            "raw": raw,
            "control_state": control_state,
            "backend_server_timestamp": (teleop_state_v2 or {}).get("server_timestamp"),
            "processed_sequence": processed_sequence,
            "raw_sequence": _safe_int(raw.get("sequence")),
            "teleop_active": bool(processed.get("active")),
            "signal_age_ms": signal_age_ms,
            "http_latency_ms": http_latency_ms,
            "t_recv_sys": t_recv_sys,
            "sse_snapshot": self.sse_monitor.snapshot() if self.sse_monitor is not None else None,
        }


class TeleopShadowTransformer:
    """Convert processed absolute teleop target to learning-level chunk fields."""

    def __init__(self, pred_horizon: int):
        self.pred_horizon = int(pred_horizon)
        if self.pred_horizon <= 0:
            raise ValueError("pred_horizon must be positive")

    def build(self, ref_pose: np.ndarray, teleop_state_v2: Optional[dict]) -> Dict[str, Any]:
        processed_target_abs = extract_processed_target_abs(teleop_state_v2)
        zero_chunk_abs = np.zeros((self.pred_horizon, 8), dtype=np.float64)
        zero_chunk_rel = np.zeros((self.pred_horizon, 7), dtype=np.float64)

        if processed_target_abs is None:
            return {
                "processed_target_abs": np.zeros(8, dtype=np.float64),
                "human_chunk_abs": zero_chunk_abs,
                "human_chunk_rel": zero_chunk_rel,
                "reconstructed_target_abs": zero_chunk_abs.copy(),
                "teleop_valid": False,
                "abs_reconstruction_pos_error": 0.0,
                "abs_reconstruction_rot_error": 0.0,
            }

        result = compute_learning_level_chunk(ref_pose, processed_target_abs, self.pred_horizon)
        return {
            "processed_target_abs": processed_target_abs.astype(np.float64),
            "human_chunk_abs": result["human_chunk_abs"].astype(np.float64),
            "human_chunk_rel": result["human_chunk_rel"].astype(np.float64),
            "reconstructed_target_abs": result["reconstructed_target_abs"].astype(np.float64),
            "teleop_valid": True,
            "abs_reconstruction_pos_error": float(result["abs_reconstruction_pos_error"]),
            "abs_reconstruction_rot_error": float(result["abs_reconstruction_rot_error"]),
        }


class TeleopUpperControlBridge:
    """Candidate/live upper control bridge managed by record_data_ros."""

    def __init__(
        self,
        env,
        signal_client: TeleopSignalClient,
        control_freq: float = 50.0,
        signal_timeout_ms: float = 150.0,
        live_enabled: bool = False,
    ):
        self.env = env
        self.signal_client = signal_client
        self.control_freq = float(control_freq)
        self.signal_timeout_ms = float(signal_timeout_ms)
        self.live_enabled = bool(live_enabled)

        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.recording_active = False
        self.owner_active = False
        self.current_processed_target_abs = None
        self.current_signal_age_ms = None
        self.current_sequence = None
        self.current_active = False
        self.last_valid_target_abs = None
        self.last_valid_sequence = None
        self.latest_candidate_target_abs = np.zeros(8, dtype=np.float64)
        self.latest_executed_target_abs = np.zeros(8, dtype=np.float64)
        self.latest_pos_error = 0.0
        self.latest_rot_error = 0.0
        self.latest_loop_dt_ms = 0.0
        self.latest_stale = False
        self.latest_applied = False
        self.stale_events = 0
        self.last_error = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.deactivate_owner()

    def update_signal(self, processed_target_abs, signal_age_ms, teleop_active, processed_sequence):
        with self.lock:
            self.current_processed_target_abs = None if processed_target_abs is None else np.asarray(
                processed_target_abs, dtype=np.float64
            )
            self.current_signal_age_ms = _safe_float(signal_age_ms)
            self.current_sequence = _safe_int(processed_sequence)
            self.current_active = bool(teleop_active)
            if self.current_processed_target_abs is not None and self.current_active:
                self.last_valid_target_abs = self.current_processed_target_abs.copy()
                self.last_valid_sequence = self.current_sequence

    def activate_for_recording(self):
        with self.lock:
            self.recording_active = True
        if self.live_enabled:
            try:
                self.signal_client.set_control_state(False, "upper_machine")
                with self.lock:
                    self.owner_active = True
                    self.last_error = None
            except Exception as exc:
                with self.lock:
                    self.owner_active = False
                    self.last_error = str(exc)
                rospy.logerr(f"Failed to acquire upper control owner: {exc}")

    def deactivate_owner(self):
        with self.lock:
            self.recording_active = False
            had_owner = self.owner_active
            self.owner_active = False
        if had_owner or self.live_enabled:
            last_exc = None
            for attempt in range(3):
                try:
                    self.signal_client.set_control_state(
                        True,
                        "lower_machine",
                        timeout_s=max(getattr(self.signal_client, "timeout_s", 0.05), 0.2),
                    )
                    with self.lock:
                        self.last_error = None
                    return
                except Exception as exc:
                    last_exc = exc
                    if attempt < 2:
                        time.sleep(0.05)
            with self.lock:
                self.last_error = str(last_exc)
            rospy.logerr(f"Failed to restore lower control owner: {last_exc}")

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "upper_candidate_target_abs": self.latest_candidate_target_abs.copy(),
                "upper_executed_target_abs": self.latest_executed_target_abs.copy(),
                "upper_candidate_pos_error": float(self.latest_pos_error),
                "upper_candidate_rot_error": float(self.latest_rot_error),
                "teleop_candidate_loop_dt_ms": float(self.latest_loop_dt_ms),
                "teleop_candidate_stale": bool(self.latest_stale),
                "teleop_candidate_applied": bool(self.latest_applied),
                "recording_active": bool(self.recording_active),
                "owner_active": bool(self.owner_active),
                "last_error": self.last_error,
                "stale_events": int(self.stale_events),
            }

    def _loop(self):
        period_s = 1.0 / self.control_freq if self.control_freq > 0 else 0.02
        last_loop_ts = time.time()

        while self.running and not rospy.is_shutdown():
            now = time.time()
            loop_dt_ms = max(0.0, (now - last_loop_ts) * 1000.0)
            last_loop_ts = now

            with self.lock:
                current_target = None if self.current_processed_target_abs is None else self.current_processed_target_abs.copy()
                signal_age_ms = self.current_signal_age_ms
                current_active = self.current_active
                last_valid_target = None if self.last_valid_target_abs is None else self.last_valid_target_abs.copy()
                recording_active = self.recording_active
                owner_active = self.owner_active

            stale = (
                (not current_active)
                or current_target is None
                or signal_age_ms is None
                or signal_age_ms > self.signal_timeout_ms
            )

            candidate_target = current_target if current_target is not None else last_valid_target
            if candidate_target is None:
                candidate_target = np.zeros(8, dtype=np.float64)

            pos_error = 0.0
            rot_error = 0.0
            obs = self.env.get_state_observation()
            if obs is not None:
                qpos_end = np.asarray(obs["qpos_end"], dtype=np.float64)
                pos_error = float(np.linalg.norm(qpos_end[:3] - candidate_target[:3]))
                if self._has_valid_quaternion(qpos_end[3:7]) and self._has_valid_quaternion(candidate_target[3:7]):
                    rot_error = float(quaternion_angle_distance(qpos_end[3:7], candidate_target[3:7]))

            applied = False
            if self.live_enabled and recording_active and owner_active and not stale and current_target is not None:
                self.env.end_control_nostep(current_target)
                applied = True

            with self.lock:
                if stale and not self.latest_stale:
                    self.stale_events += 1
                self.latest_candidate_target_abs = np.asarray(candidate_target, dtype=np.float64)
                if applied:
                    self.latest_executed_target_abs = np.asarray(current_target, dtype=np.float64)
                self.latest_pos_error = pos_error
                self.latest_rot_error = rot_error
                self.latest_loop_dt_ms = loop_dt_ms
                self.latest_applied = applied
                self.latest_stale = stale

            time.sleep(period_s)

    @staticmethod
    def _has_valid_quaternion(quat: np.ndarray, eps: float = 1e-8) -> bool:
        quat = np.asarray(quat, dtype=np.float64)
        return quat.shape == (4,) and np.all(np.isfinite(quat)) and np.linalg.norm(quat) > eps
