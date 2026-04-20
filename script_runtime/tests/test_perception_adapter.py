from script_runtime.adapters.perception_adapter import PerceptionObservation, RoboTwinDepthPoseProvider


class _StubOracleBackend:
    def get_grasp_candidates(self):
        return [
            {
                "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                "arm": "left",
                "score": 1.0,
            }
        ]


class _PlannerSDK:
    def evaluate_pose_candidates(self, poses, kind="pregrasp"):
        rows = []
        for index, pose in enumerate(poses):
            rows.append(
                {
                    "status": "Success" if index == 1 else "Failure",
                    "waypoint_count": 24 if index == 1 else 0,
                    "pose": pose,
                    "kind": kind,
                }
            )
        return rows


class _Context:
    def __init__(self, arm="left"):
        self.adapters = {"sdk": _PlannerSDKWithArm(arm=arm)}
        self.world_state = type("World", (), {"robot": type("Robot", (), {"eef_pose": [-0.2, -0.2, 0.9]})()})()
        self.blackboard = None


class _PlannerSDKWithArm(_PlannerSDK):
    def __init__(self, arm="left"):
        self.arm = arm

    def _active_arm(self):
        return self.arm


class _EmptyOracleBackend:
    def get_grasp_candidates(self):
        return []


def test_robotwin_depth_provider_expands_single_oracle_candidate():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_StubOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(observation)

    assert candidates is not None
    assert len(candidates) >= 4
    assert candidates[0]["variant_label"] == "base"
    assert any(candidate["variant_label"] == "pregrasp_high" for candidate in candidates)
    assert any(candidate["variant_label"] == "lateral_pos" for candidate in candidates)
    assert any("yaw_pos_20" in candidate["variant_label"] for candidate in candidates)
    assert provider.last_grasp_source == "oracle_augmented"


def test_robotwin_depth_provider_deduplicates_candidate_variants():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_StubOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(observation)

    keys = {
        (
            tuple(round(float(v), 4) for v in candidate["pose"][:7]),
            tuple(round(float(v), 4) for v in candidate["pregrasp_pose"][:7]),
        )
        for candidate in candidates
    }
    assert len(keys) == len(candidates)


def test_robotwin_depth_provider_uses_planner_to_rerank_candidates():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_StubOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(observation, context=_Context())

    assert candidates is not None
    assert candidates[0]["planner_status"] == "Success"
    assert provider.last_grasp_diagnostics
    assert provider.last_grasp_diagnostics[0]["planner_status"] == "Success"


def test_robotwin_depth_provider_synthesizes_candidates_without_oracle_grasps():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_EmptyOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(
        observation,
        context=_Context(),
    )

    # No real depth is provided in this unit test, so synthesis still needs a pose hint.
    synthesized = provider._synthesize_grasp_candidates_from_pose([0.12, -0.01, 0.74, 0.0, 0.0, 0.0, 1.0], context=_Context())
    assert len(synthesized) >= 8
    assert synthesized[0]["variant_label"].startswith("synth_")
    assert any("ori_pitch_pos_18" in candidate["variant_label"] for candidate in synthesized)


def test_robotwin_depth_provider_adds_left_arm_specific_variants():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_StubOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(observation, context=_Context(arm="left"))

    assert candidates is not None
    assert any("left_short_backoff" in candidate["variant_label"] for candidate in candidates)
    assert any(candidate.get("arm") == "left" for candidate in candidates)


class _FeasibleOracleBackend:
    def get_grasp_candidates(self):
        return [
            {
                "pose": [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                "pregrasp_pose": [0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                "arm": "left",
                "score": 1.5,
                "variant_label": "contact_0",
                "planner_status": "Success",
                "planner_waypoint_count": 12,
            },
            {
                "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                "arm": "left",
                "score": 0.2,
                "variant_label": "contact_1",
                "planner_status": "Failure",
            },
        ]


def test_robotwin_depth_provider_keeps_backend_feasible_candidate_first():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_FeasibleOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(observation, context=_Context(arm="left"))

    assert candidates is not None
    assert candidates[0]["variant_label"] == "contact_0"
    assert candidates[0]["planner_status"] == "Success"
    assert provider.last_grasp_source == "oracle_feasibility_first"


class _FlakyOracleBackend:
    def __init__(self):
        self.calls = 0

    def get_grasp_candidates(self):
        self.calls += 1
        if self.calls == 1:
            return [
                {
                    "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                    "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                    "arm": "left",
                    "score": 0.1,
                    "variant_label": "contact_1",
                    "planner_status": "Failure",
                }
            ]
        return [
            {
                "pose": [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                "pregrasp_pose": [0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                "arm": "left",
                "score": 1.3,
                "variant_label": "contact_0",
                "planner_status": "Success",
                "planner_waypoint_count": 10,
            }
        ]


def test_robotwin_depth_provider_retries_backend_before_augmenting():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_FlakyOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(observation, context=_Context(arm="left"))

    assert candidates is not None
    assert candidates[0]["variant_label"] == "contact_0"
    assert candidates[0]["planner_status"] == "Success"
    assert provider.last_grasp_source == "oracle_feasibility_first"


class _MultiReadOracleBackend:
    def __init__(self):
        self.calls = 0

    def get_grasp_candidates(self):
        self.calls += 1
        if self.calls == 1:
            return [
                {
                    "pose": [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                    "pregrasp_pose": [0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                    "arm": "left",
                    "score": 1.4,
                    "variant_label": "contact_0",
                    "contact_point_id": 0,
                    "planner_status": "Success",
                    "planner_waypoint_count": 14,
                }
            ]
        return [
            {
                "pose": [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                "pregrasp_pose": [0.28, 0.0, 0.33, 0.0, 0.0, 0.0, 1.0],
                "arm": "left",
                "score": 1.4,
                "variant_label": "contact_0",
                "contact_point_id": 0,
                "planner_status": "Success",
                "planner_waypoint_count": 14,
            },
            {
                "pose": [0.31, -0.02, 0.21, 0.0, 0.0, 0.0, 1.0],
                "pregrasp_pose": [0.22, -0.02, 0.31, 0.0, 0.0, 0.0, 1.0],
                "arm": "left",
                "score": 1.1,
                "variant_label": "contact_1",
                "contact_point_id": 1,
                "planner_status": "Success",
                "planner_waypoint_count": 19,
            },
        ]


def test_robotwin_depth_provider_merges_candidates_across_backend_reads():
    backend = _MultiReadOracleBackend()
    provider = RoboTwinDepthPoseProvider(oracle_backend=backend, use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(observation, context=_Context(arm="left"))

    assert candidates is not None
    assert backend.calls == 2
    labels = [candidate["variant_label"] for candidate in candidates]
    assert labels[:2] == ["contact_0", "contact_1"]
    assert labels.count("contact_0") == 1
    assert labels.count("contact_1") == 1
    assert all(candidate["planner_status"] == "Success" for candidate in candidates[:2])
    assert provider.last_grasp_source == "oracle_feasibility_first"
