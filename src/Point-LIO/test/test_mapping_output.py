import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from mapping_control_node import MappingConfig, MappingSession  # noqa: E402


def test_saved_pgm_matches_live_grid_after_manual_clear(tmp_path: Path) -> None:
    input_pcd = tmp_path / "scans.pcd"
    executable = tmp_path / "pcd2pgm_headless"
    config_file = tmp_path / "map.cfg"
    input_pcd.write_bytes(b"pcd")
    executable.write_bytes(b"executable")
    config_file.write_text("output_stem=maps\n", encoding="utf-8")

    def fake_converter(_cmd: list[str], cwd: Path) -> None:
        (cwd / "map.pcd").write_bytes(b"pcd")

    session = MappingSession(MappingConfig(
        output_dir=tmp_path / "output",
        resolution=1.0,
        size_x=3.0,
        size_y=2.0,
        origin_x=0.0,
        origin_y=0.0,
        occupied_min_observations=1,
        free_min_observations=1,
        occupied_closing_iterations=0,
        occupied_despeckle_neighbors=0,
        pcd_input_path=input_pcd,
        pcd2pgm_executable=executable,
        pcd2pgm_config=config_file,
    ), command_runner=fake_converter)
    session.start("test", "session")
    session.ingest_xyz(np.array([[0.2, 0.2, 0.0], [1.2, 0.2, -1.0]]))
    session.clear_radius(0.5, 0.5, 0.6)
    cleared = session.occupancy_grid_snapshot(preview=False)["cells"]
    assert cleared[0, 0] == 0
    # 手动清除只清当前证据；后续真实扫描可以重新补回障碍。
    session.ingest_xyz(np.array([[0.2, 0.2, 0.0]]))
    expected = session.occupancy_grid_snapshot(preview=False)["cells"]
    assert expected[0, 0] == 100

    result = session.save_with_cloud_snapshot(lambda: None)
    pgm = Path(result["map_pgm_path"]).read_bytes()
    header, pixels = pgm.split(b"\n255\n", 1)

    assert header == b"P5\n3 2"
    saved = np.flipud(np.frombuffer(pixels, dtype=np.uint8).reshape(2, 3))
    actual = np.full(saved.shape, -1, dtype=np.int8)
    actual[saved == 254] = 0
    actual[saved == 0] = 100
    assert np.array_equal(actual, expected)
    yaml = Path(result["map_yaml_path"]).read_text()
    assert 'image: "maps.pgm"' in yaml
    assert "free_thresh: 0.196" in yaml
