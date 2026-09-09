"""Named motion settings only; never store calibration or hardware authority."""
import json
from pathlib import Path

from .plans import atomic_json, validate_plan


def validate_name(name):
    if (not isinstance(name, str) or not 1 <= len(name) <= 30
            or name != name.strip() or not name.isprintable()):
        raise ValueError('名稱請填 1～30 個可顯示的字，例如「L2 慢速正轉」。')
    return name


def validate_library(value):
    if not isinstance(value, dict) or len(value) > 30:
        raise ValueError('動作收藏格式不正確，最多可存 30 組。')
    for name, plan in value.items():
        validate_name(name)
        validate_plan(plan)
    return value


def load_library(path):
    path = Path(path)
    if not path.exists():
        return {}
    if path.stat().st_size > 262144:
        raise ValueError('動作收藏檔過大，請檢查 '+str(path))
    return validate_library(json.loads(path.read_text()))


def save_library(path, value):
    atomic_json(path, validate_library(value))
