from script_runtime.executors.pytrees_executor import RecoveryNode, RetryNode, SequenceNode, TimeoutNode
from script_runtime.tasks.probes.articulated_probe import ArticulatedProbeTask
from script_runtime.tasks.probes.common import (
    PROBE_EXECUTE_GRASP_TIMEOUT_S,
    PROBE_GO_PREGRASP_TIMEOUT_S,
    PROBE_PREPARE_GRIPPER_TIMEOUT_S,
)
from script_runtime.tasks.probes.handover_probe import HandoverProbeTask
from script_runtime.tasks.probes.staged_place_probe import StagedPlaceProbeTask


def _grasp_timeout_nodes(task):
    root = task.build(registry=None)
    assert isinstance(root, SequenceNode)
    grasp_retry = root.children[2]
    assert isinstance(grasp_retry, RetryNode)
    grasp_recovery = grasp_retry.child
    assert isinstance(grasp_recovery, RecoveryNode)
    grasp_attempt = grasp_recovery.child
    assert isinstance(grasp_attempt, SequenceNode)
    nodes = grasp_attempt.children[:4]
    assert isinstance(nodes[0], TimeoutNode)
    assert isinstance(nodes[1], TimeoutNode)
    assert isinstance(nodes[3], TimeoutNode)
    return nodes[0], nodes[1], nodes[3]


def test_staged_place_probe_uses_relaxed_probe_timeouts():
    prepare, pregrasp, grasp = _grasp_timeout_nodes(StagedPlaceProbeTask())
    assert prepare.timeout_s == PROBE_PREPARE_GRIPPER_TIMEOUT_S
    assert pregrasp.timeout_s == PROBE_GO_PREGRASP_TIMEOUT_S
    assert grasp.timeout_s == PROBE_EXECUTE_GRASP_TIMEOUT_S


def test_articulated_probe_uses_relaxed_probe_timeouts():
    prepare, pregrasp, grasp = _grasp_timeout_nodes(ArticulatedProbeTask())
    assert prepare.timeout_s == PROBE_PREPARE_GRIPPER_TIMEOUT_S
    assert pregrasp.timeout_s == PROBE_GO_PREGRASP_TIMEOUT_S
    assert grasp.timeout_s == PROBE_EXECUTE_GRASP_TIMEOUT_S


def test_handover_probe_uses_relaxed_probe_timeouts_for_both_arms():
    root = HandoverProbeTask().build(registry=None)
    assert isinstance(root, SequenceNode)

    source_retry = root.children[2]
    assert isinstance(source_retry, RetryNode)
    source_recovery = source_retry.child
    assert isinstance(source_recovery, RecoveryNode)
    source_attempt = source_recovery.child
    assert isinstance(source_attempt, SequenceNode)

    receiver_retry = root.children[5]
    assert isinstance(receiver_retry, RetryNode)
    receiver_recovery = receiver_retry.child
    assert isinstance(receiver_recovery, RecoveryNode)
    receiver_attempt = receiver_recovery.child
    assert isinstance(receiver_attempt, SequenceNode)

    for attempt in [source_attempt, receiver_attempt]:
        prepare, pregrasp, grasp = attempt.children[0], attempt.children[1], attempt.children[3]
        assert isinstance(prepare, TimeoutNode)
        assert isinstance(pregrasp, TimeoutNode)
        assert isinstance(grasp, TimeoutNode)
        assert prepare.timeout_s == PROBE_PREPARE_GRIPPER_TIMEOUT_S
        assert pregrasp.timeout_s == PROBE_GO_PREGRASP_TIMEOUT_S
        assert grasp.timeout_s == PROBE_EXECUTE_GRASP_TIMEOUT_S
