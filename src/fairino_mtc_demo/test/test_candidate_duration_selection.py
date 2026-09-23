"""Regression checks for bounded first-feasible candidate selection."""
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / "src" / "first_grasp_node.cpp"
).read_text()


def test_candidate_metrics_are_still_recorded_for_route_auditing():
    assert "total.duration_s += part.duration_s" in SOURCE
    assert "best_duration_s = metrics.duration_s" in SOURCE
    assert "best_path_length_rad = metrics.path_length_rad" in SOURCE


def test_first_complete_candidate_is_selected_without_exhaustive_comparison():
    preview = SOURCE[
        SOURCE.index("std::vector<PlannedMotion> previewAll()"):
        SOURCE.index("void publishPreview(")]
    assert '"grasp_first_feasible"' in preview
    assert preview.index("best_motions = std::move(motions);") < preview.index(
        '"grasp_first_feasible"')
    assert "break;" in preview[
        preview.index('"grasp_first_feasible"'):
        preview.index("} catch (const std::exception & error)", preview.index('"grasp_first_feasible"'))]
    assert "metrics.duration_s < best_duration_s" not in preview
    assert "equal_duration_shorter_path" not in preview


def test_candidate_and_execution_logs_include_planned_duration():
    assert SOURCE.count("planned_duration_s=") >= 3
