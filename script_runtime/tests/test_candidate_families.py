from script_runtime.planning import build_grasp_candidate_variants


def test_build_grasp_candidate_variants_adds_object_centric_current_lane_variants():
    candidate = {
        "pose": [-0.29, -0.02, 0.826, 0.0, 0.0, 0.0, 1.0],
        "pregrasp_pose": [-0.389, -0.018, 0.846, 0.0, 0.0, 0.0, 1.0],
        "arm": "left",
    }
    object_pose = [-0.205, -0.023, 0.741, 0.5, 0.5, 0.5, 0.5]
    current_eef = [-0.298, -0.314, 0.942, 0.7, 0.0, 0.0, 0.714]

    variants = build_grasp_candidate_variants(
        candidate=candidate,
        object_pose=object_pose,
        current_eef=current_eef,
    )

    labels = {variant["variant_label"] for variant in variants}
    assert "object_current_lane" in labels
    assert "object_inside_sweep" in labels
    assert "object_arc_entry" in labels


def test_build_grasp_candidate_variants_adds_left_arm_coarse_orientation_bank():
    candidate = {
        "pose": [-0.29, -0.02, 0.826, 0.0, 0.0, 0.0, 1.0],
        "pregrasp_pose": [-0.389, -0.018, 0.846, 0.0, 0.0, 0.0, 1.0],
        "arm": "left",
    }
    object_pose = [-0.205, -0.023, 0.741, 0.5, 0.5, 0.5, 0.5]
    current_eef = [-0.298, -0.314, 0.942, 0.7, 0.0, 0.0, 0.714]

    variants = build_grasp_candidate_variants(
        candidate=candidate,
        object_pose=object_pose,
        current_eef=current_eef,
    )

    labels = {variant["variant_label"] for variant in variants}
    assert "base_left_yaw_pos_45" in labels
    assert "base_left_yaw_neg_45" in labels
    assert "base_left_roll_in_35" in labels
    assert "object_current_lane_left_yaw_pos_45" in labels
