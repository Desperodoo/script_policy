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
    def __init__(self):
        self.adapters = {"sdk": _PlannerSDK()}
        self.world_state = type("World", (), {"robot": type("Robot", (), {"eef_pose": [-0.2, -0.2, 0.9]})()})()


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
    assert provider.last_grasp_source == "oracle_augmented"


def test_robotwin_depth_provider_deduplicates_candidate_variants():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_StubOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(observation)

    keys = {
        (
            tuple(round(float(v), 4) for v in candidate["pose"][:3]),
            tuple(round(float(v), 4) for v in candidate["pregrasp_pose"][:3]),
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


def test_robotwin_depth_provider_synthesizes_candidates_without_oracle_grasps():
    provider = RoboTwinDepthPoseProvider(oracle_backend=_EmptyOracleBackend(), use_oracle_fallback=False)
    observation = PerceptionObservation(task_goal={"target_object": "cup"})

    candidates = provider.get_grasp_candidates(
        observation,
        context=_Context(),
    )

    # No real depth is provided in this unit test, so synthesis still needs a pose hint.
    synthesized = provider._synthesize_grasp_candidates_from_pose([0.12, -0.01, 0.74, 0.0, 0.0, 0.0, 1.0], context=_Context())
    assert len(synthesized) >= 4
    assert synthesized[0]["variant_label"].startswith("synth_")
