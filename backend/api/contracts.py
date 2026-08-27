"""公开实时 API 的闭合 JSON 结构合同。"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


API_VERSION = "v1"
API_VERSION_HEADER = "X-Cable-Tension-API-Version"
CONTRACT_ROOT = Path(__file__).resolve().parent / "contracts"

SCHEMA_FILES = {
    "realtime-session-create": "realtime_session_create_v1.schema.json",
    "realtime-sensor-packet": "realtime_sensor_packet_v1.schema.json",
    "realtime-result": "realtime_result_v1.schema.json",
}


def load_schema(name: str) -> dict[str, Any]:
    """加载内置 Schema；它们是测试和适配层的单一字段来源。"""

    filename = SCHEMA_FILES.get(name)
    if filename is None:
        raise KeyError(name)
    return json.loads((CONTRACT_ROOT / filename).read_text(encoding="utf-8"))


def json_structure_errors(name: str, value: Any) -> dict[str, str]:
    """在进入物理量解析前拒绝类型错误、缺失字段和额外字段。"""

    root = load_schema(name)
    errors: dict[str, str] = {}
    _collect_errors(root, value, root=root, root_name=name, path="", errors=errors)
    return errors


def _collect_errors(
    schema: dict[str, Any],
    value: Any,
    *,
    root: dict[str, Any],
    root_name: str,
    path: str,
    errors: dict[str, str],
) -> None:
    reference = schema.get("$ref")
    if reference is not None:
        target, target_root, target_name = _resolve_reference(
            reference,
            root=root,
            root_name=root_name,
        )
        _collect_errors(
            target,
            value,
            root=target_root,
            root_name=target_name,
            path=path,
            errors=errors,
        )
        return

    for subschema in schema.get("allOf", []):
        _collect_errors(
            subschema,
            value,
            root=root,
            root_name=root_name,
            path=path,
            errors=errors,
        )

    expected = schema.get("type")
    if expected is not None and not _matches_type(value, expected):
        errors[path or "$"] = f"must be a JSON {expected}"
        return
    if "const" in schema and value != schema["const"]:
        errors[path or "$"] = f"must equal {schema['const']!r}"
        return

    if expected == "object":
        properties = schema.get("properties", {})
        for required in schema.get("required", []):
            if required not in value:
                errors[_join(path, required)] = "is required"
        if schema.get("additionalProperties") is False:
            for key in value.keys() - properties.keys():
                errors[_join(path, key)] = "is not allowed by the schema"
        for key, child in properties.items():
            if key in value:
                _collect_errors(
                    child,
                    value[key],
                    root=root,
                    root_name=root_name,
                    path=_join(path, key),
                    errors=errors,
                )
    elif expected == "array" and "items" in schema:
        for index, item in enumerate(value):
            _collect_errors(
                schema["items"],
                item,
                root=root,
                root_name=root_name,
                path=f"{path}[{index}]",
                errors=errors,
            )


def _resolve_reference(
    reference: str,
    *,
    root: dict[str, Any],
    root_name: str,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    if reference.startswith("#"):
        target_root = root
        target_name = root_name
        fragment = reference[1:]
    else:
        location, _, fragment_value = reference.partition("#")
        if not location.startswith("./"):
            raise ValueError(f"Unsupported JSON Schema reference: {reference}")
        target_name = location.removeprefix("./")
        target_root = load_schema(target_name)
        fragment = f"/{fragment_value.removeprefix('/')}" if fragment_value else ""
    target: Any = target_root
    if fragment:
        for token in fragment.removeprefix("/").split("/"):
            target = target[token.replace("~1", "/").replace("~0", "~")]
    return target, target_root, target_name


def _matches_type(value: Any, expected: str | list[str]) -> bool:
    if isinstance(expected, list):
        return any(_matches_type(value, item) for item in expected)
    if expected == "object":
        return isinstance(value, dict)
    if expected == "array":
        return isinstance(value, list)
    if expected == "string":
        return isinstance(value, str)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    if expected == "integer":
        return (
            isinstance(value, int) and not isinstance(value, bool)
        ) or (
            isinstance(value, float) and math.isfinite(value) and value.is_integer()
        )
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "null":
        return value is None
    raise ValueError(f"Unsupported JSON Schema type: {expected}")


def _join(parent: str, child: str) -> str:
    return f"{parent}.{child}" if parent else child
