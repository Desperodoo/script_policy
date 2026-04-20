from script_runtime.adapters.robotwin_bridge import RoboTwinBridge


class _FakeActor:
    def iter_contact_points(self, mode):
        assert mode == "list"
        yield 0, [0.0, 0.0, 0.0]
        yield 1, [0.0, 0.0, 0.0]


class _FakeEnv:
    def choose_grasp_pose(self, actor, arm_tag, pre_dis=0.1, contact_point_id=None):
        assert actor is not None
        if contact_point_id == 0:
            return [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0], [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
        return [0.2, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0], [0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]


def test_feasibility_first_selector_prefers_reachable_contact():
    bridge = RoboTwinBridge()
    bridge.env = _FakeEnv()

    statuses = {
        tuple([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]): {"status": "Success", "waypoint_count": 15},
        tuple([0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]): {"status": "Success", "waypoint_count": 12},
        tuple([0.2, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]): {"status": "Failure"},
        tuple([0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]): {"status": "Failure"},
    }

    def _fake_plan_single_pose(pose, arm=None):
        return dict(statuses[tuple(pose)])

    bridge._plan_single_pose = _fake_plan_single_pose  # type: ignore[attr-defined]

    candidates = bridge._build_feasibility_first_grasp_candidates(actor=_FakeActor(), arm="left")

    assert len(candidates) == 2
    assert candidates[0]["contact_point_id"] == 0
    assert candidates[0]["planner_status"] == "Success"
    assert candidates[1]["contact_point_id"] == 1
    assert candidates[1]["planner_status"] == "Failure"


def test_feasibility_first_selector_marks_grasp_fail_as_failure():
    bridge = RoboTwinBridge()
    bridge.env = _FakeEnv()

    statuses = {
        tuple([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]): {"status": "Success", "waypoint_count": 15},
        tuple([0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]): {"status": "Failure"},
    }

    def _fake_plan_single_pose(pose, arm=None):
        return dict(statuses.get(tuple(pose), {"status": "Failure"}))

    bridge._plan_single_pose = _fake_plan_single_pose  # type: ignore[attr-defined]

    candidate = bridge._contact_grasp_candidate(actor=_FakeActor(), arm="left", contact_id=0)

    assert candidate is not None
    assert candidate["planner_status"] == "Failure"
    assert candidate["planner_debug"]["pregrasp_status"] == "Success"
    assert candidate["planner_debug"]["grasp_status"] == "Failure"
