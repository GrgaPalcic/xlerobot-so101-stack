import numpy as np

from grasp_server.pipeline import DetectionBox, ViewCandidate, select_cross_view_candidates


def _candidate(view, center_x, score, centroid):
    box = DetectionBox(
        index=int(center_x),
        box_xyxy=(center_x - 2.0, 0.0, center_x + 2.0, 4.0),
        score=score,
    )
    return ViewCandidate(
        view_name=view,
        detection=box,
        rank=-1,
        points_base=np.zeros((3, 3), dtype=np.float32),
        colors=np.zeros((3, 3), dtype=np.float32),
        mask=np.zeros((4, 4), dtype=bool),
        depth_m=np.ones((4, 4), dtype=np.float32),
        centroid_base=np.asarray(centroid, dtype=np.float32),
    )


def test_metric_association_prefers_nearest_centroids():
    selected, strategy = select_cross_view_candidates(
        {
            "overhead": [
                _candidate("overhead", 10.0, 0.90, [0.1, 0.0, 0.2]),
                _candidate("overhead", 50.0, 0.20, [0.5, 0.0, 0.2]),
            ],
            "wrist": [
                _candidate("wrist", 20.0, 0.10, [0.51, 0.0, 0.2]),
                _candidate("wrist", 60.0, 0.85, [0.12, 0.0, 0.2]),
            ],
        },
        primary_view="overhead",
        axis="x",
        max_metric_distance_m=0.05,
    )

    assert strategy.startswith("metric_centroid")
    assert selected["overhead"].detection.center_x == 10.0
    assert selected["wrist"].detection.center_x == 60.0


def test_untrusted_metric_association_keeps_only_primary_view():
    selected, strategy = select_cross_view_candidates(
        {
            "overhead": [
                _candidate("overhead", 10.0, 0.95, [0.1, 0.0, 0.2]),
                _candidate("overhead", 50.0, 0.30, [0.5, 0.0, 0.2]),
            ],
            "wrist": [
                _candidate("wrist", 20.0, 0.25, [1.1, 0.0, 0.2]),
                _candidate("wrist", 60.0, 0.90, [1.5, 0.0, 0.2]),
            ],
        },
        primary_view="overhead",
        axis="x",
        max_metric_distance_m=0.05,
    )

    assert strategy == "single_overhead:metric_untrusted:0.600m"
    assert set(selected) == {"overhead"}
    assert selected["overhead"].detection.center_x == 10.0
