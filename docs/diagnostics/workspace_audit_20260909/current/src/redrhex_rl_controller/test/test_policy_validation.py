from __future__ import annotations

from redrhex_rl_controller.policy_validation import (
    deployment_metadata_rejection_reasons,
)


def test_deployment_metadata_accepts_approved_or_unrelated_values() -> None:
    assert deployment_metadata_rejection_reasons({}) == []
    assert deployment_metadata_rejection_reasons(
        {
            "redrhex_artifact_status": "deployment_approved",
            "redrhex_quality_status": "quality_approved",
            "training.note": "exported for the hardware validation queue",
        }
    ) == []


def test_deployment_metadata_rejects_unknown_status_values() -> None:
    assert deployment_metadata_rejection_reasons(
        {
            "redrhex_artifact_status": "pending_review",
            "redrhex_quality_status": "production_candidate",
        }
    ) == [
        "redrhex_artifact_status=pending_review",
        "redrhex_quality_status=production_candidate",
    ]


def test_deployment_metadata_rejects_current_diagnostic_labels() -> None:
    reasons = deployment_metadata_rejection_reasons(
        {
            "redrhex_artifact_status": "diagnostic_only_not_deployable",
            "redrhex_quality_status": "quality_rejected",
        }
    )
    assert reasons == [
        "redrhex_artifact_status=diagnostic_only_not_deployable",
        "redrhex_quality_status=quality_rejected",
    ]


def test_deployment_metadata_rejects_false_deployable_flags() -> None:
    assert deployment_metadata_rejection_reasons(
        {
            "redrhex.deployable": "false",
            "redrhex.deployment_approved": "no",
        }
    ) == [
        "redrhex.deployable=false",
        "redrhex.deployment_approved=no",
    ]
