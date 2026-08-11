from pathlib import Path


source = (Path(__file__).parents[1] / "src" / "pid_path_follower.cpp").read_text(
    encoding="utf-8"
)
target = source.split("PidPathFollower::targetPoseFromLocalPath(", 1)[1]
target = target.split("\n}\n\n", 1)[0]

assert "selection.anchor_pose" in target
assert "robot_pose" not in target
assert "selection.local_path.size() < 2" not in source
assert "selection.local_path.size() == 1" in source
