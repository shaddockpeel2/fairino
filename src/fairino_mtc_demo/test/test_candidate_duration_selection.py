"""Regression checks for fastest complete-trajectory candidate selection."""
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / "src" / "first_grasp_node.cpp"
).read_text()


def test_candidate_primary_score_is_total_planned_duration():
    assert "total.duration_s += part.duration_s" in SOURCE
    assert "metrics.duration_s < best_duration_s" in SOURCE
    assert "best_duration_s = metrics.duration_s" in SOURCE


def test_path_length_is_only_an_equal_duration_tie_breaker():
    assert "equal_duration_shorter_path" in SOURCE
    assert "metrics.path_length_rad < best_path_length_rad" in SOURCE


def test_candidate_and_execution_logs_include_planned_duration():
    assert SOURCE.count("planned_duration_s=") >= 3
