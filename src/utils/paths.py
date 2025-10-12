import os

from pathlib import Path
from typing import Union


def get_project_root() -> Path:
    return Path.cwd()


def resolve_scene_path(scene_path: str, scene_name: str) -> Path:
    project_root = get_project_root()

    scenes_dir = project_root / scene_path
    scene_file = scenes_dir / f"{scene_name}.ttt"

    scene_file = scene_file.resolve()

    if not scene_file.exists():
        raise FileNotFoundError(f"Scene file not found: {scene_file}")

    return scene_file


def resolve_output_path(output_dir: str) -> Path:
    project_root = get_project_root()
    output_path = project_root / output_dir
    return output_path.resolve()


def ensure_directory_exists(directory_path: Union[str, Path]) -> Path:
    path_obj = Path(directory_path)
    path_obj.mkdir(parents=True, exist_ok=True)
    return path_obj


def validate_scene_file(scene_path: str, scene_name: str) -> bool:
    try:
        scene_file = resolve_scene_path(scene_path, scene_name)
        return scene_file.is_file() and os.access(scene_file, os.R_OK)
    except FileNotFoundError:
        return False


def get_relative_path_for_display(absolute_path: Path) -> str:
    project_root = get_project_root()

    try:
        return str(absolute_path.relative_to(project_root))
    except ValueError:
        return str(absolute_path)
