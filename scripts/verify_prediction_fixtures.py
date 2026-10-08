#!/usr/bin/env python3
"""Independent stdlib-only schema subset + semantic validation of Java prediction vectors.

The interpreter implements precisely the JSON Schema keywords used by the checked-in
v1 schemas. It is not a general replacement for a Draft 2020-12 validator. Runtime
consumers may use a full validator plus these cross-document semantic checks.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import re
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent


def require(ok: bool, detail: str) -> None:
    if not ok:
        raise ValueError(detail)


def read_json(path: Path) -> Any:
    def reject_constant(value: str) -> None:
        raise ValueError(f"JSON nonfinite constant {value}")

    def unique_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result = {}
        for key, value in pairs:
            require(key not in result, f"duplicate JSON key: {key}")
            result[key] = value
        return result

    return json.loads(path.read_text(), parse_constant=reject_constant, object_pairs_hook=unique_pairs)


def validate_schema(value: Any, schema: dict, root: dict | None = None, path: str = "$") -> None:
    root = schema if root is None else root
    if "$ref" in schema:
        require(schema["$ref"].startswith("#/$defs/"), "only local schema references supported")
        validate_schema(value, root["$defs"][schema["$ref"].split("/")[-1]], root, path)
    for keyword in ("oneOf", "anyOf"):
        if keyword in schema:
            count = 0
            for option in schema[keyword]:
                try:
                    validate_schema(value, option, root, path)
                    count += 1
                except ValueError:
                    pass
            require(count == 1 if keyword == "oneOf" else count >= 1, f"{path}: {keyword} mismatch")
    for option in schema.get("allOf", []):
        validate_schema(value, option, root, path)
    if "if" in schema:
        try:
            validate_schema(value, schema["if"], root, path)
            matched = True
        except ValueError:
            matched = False
        if matched and "then" in schema:
            validate_schema(value, schema["then"], root, path)
    if "const" in schema:
        # Python bool is an int subclass; JSON const 0 must not accept false.
        require(value == schema["const"] and (not isinstance(value, bool) or isinstance(schema["const"], bool)), f"{path}: constant mismatch")
    if "enum" in schema:
        require(value in schema["enum"], f"{path}: unsupported enum")
    kind = schema.get("type")
    types = {"object": isinstance(value, dict), "array": isinstance(value, list),
             "string": isinstance(value, str), "boolean": isinstance(value, bool),
             "integer": isinstance(value, int) and not isinstance(value, bool),
             "number": isinstance(value, (int, float)) and not isinstance(value, bool),
             "null": value is None}
    if kind:
        require(types.get(kind, False), f"{path}: expected {kind}")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        require(math.isfinite(value), f"{path}: nonfinite number")
        if "minimum" in schema:
            require(value >= schema["minimum"], f"{path}: below minimum")
        if "maximum" in schema:
            require(value <= schema["maximum"], f"{path}: above maximum")
    if isinstance(value, str):
        require(len(value) >= schema.get("minLength", 0), f"{path}: short string")
        require(len(value) <= schema.get("maxLength", math.inf), f"{path}: long string")
        if "pattern" in schema:
            require(re.search(schema["pattern"], value) is not None, f"{path}: string pattern mismatch")
    if isinstance(value, list):
        require(len(value) >= schema.get("minItems", 0), f"{path}: too few items")
        require(len(value) <= schema.get("maxItems", math.inf), f"{path}: too many items")
        if "items" in schema:
            for i, element in enumerate(value):
                validate_schema(element, schema["items"], root, f"{path}[{i}]")
    if isinstance(value, dict):
        require(set(schema.get("required", [])) <= set(value), f"{path}: missing keys")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            require(set(value) <= set(properties), f"{path}: unknown keys")
        for key, element in value.items():
            if key in properties:
                validate_schema(element, properties[key], root, f"{path}.{key}")


def covariance(xx: float, xy: float, yy: float, positive: bool = False) -> None:
    require(all(math.isfinite(v) for v in (xx, xy, yy)), "nonfinite covariance")
    require(xx >= 0 and yy >= 0 and xy * xy <= xx * yy + 1e-12, "covariance not PSD")
    if positive:
        require(xx > 0 and yy > 0, "valid forecast needs positional uncertainty")


def validate_request(r: dict) -> None:
    validate_schema(r, read_json(ROOT / "schemas/prediction-request-v1.schema.json"))
    require(r["history_cutoff_us"] <= r["issued_us"] < r["valid_until_us"], "invalid request time window")
    require(r["sync"]["valid"], "unsynchronized input")
    require(r["horizon_us"] % r["step_us"] == 0, "nondivisible grid")
    n = r["horizon_us"] // r["step_us"] + 1
    require(n <= 256, "unbounded grid")
    previous_time = previous_id = -1
    for h in r["history"]:
        s = h["snapshot"]
        require(s["epoch"] == r["epoch"] and s["field"] == r["field"], "history reset mismatch")
        time_us = s["ego"]["time_us"]
        require(previous_time < time_us <= r["history_cutoff_us"] and previous_id < s["id"], "history time/id order")
        previous_time, previous_id = time_us, s["id"]
        if h["valid_mask"]:
            require(s["ego"]["localization_valid"], "valid history invalid localization")
        u = s["ego"]["uncertainty"]
        covariance(u["xx_m2"], u["xy_m2"], u["yy_m2"])
        track_ids = set()
        for t in s["tracks"]:
            require(t["id"] not in track_ids, "duplicate history track")
            track_ids.add(t["id"])
            require(t["epoch"] == r["epoch"] and t["last_measurement_us"] <= t["estimate_us"] <= time_us, "track epoch/time")
            u = t["uncertainty"]
            covariance(u["xx_m2"], u["xy_m2"], u["yy_m2"])
            for stamp in t["provenance"]:
                require(stamp["capture_us"] <= stamp["publication_us"], "publication before capture")
        c = h["accepted_command"]
        if c:
            require(c["issued_us"] <= c["accepted_us"] <= time_us, "command not accepted at history sample")
            mechanisms(c["mechanisms"])
    require(previous_id == r["snapshot_id"] and r["history"][-1]["valid_mask"], "latest snapshot mismatch")
    require(len({t["target_id"] for t in r["targets"]}) == len(r["targets"]), "duplicate target")
    require(sum(t["kind"] == "EGO" for t in r["targets"]) <= 1, "duplicate ego")
    track_targets = [t["track_id"] for t in r["targets"] if t["kind"] == "OBJECT_TRACK"]
    require(len(set(track_targets)) == len(track_targets) and set(track_targets) <= {t["id"] for t in r["history"][-1]["snapshot"]["tracks"]}, "target track absent/duplicated")
    require(len({c["candidate_id"] for c in r["candidates"]}) == len(r["candidates"]), "duplicate candidate")
    for c in r["candidates"]:
        require(len(c["samples"]) == n, "candidate dimensions")
        for i, s in enumerate(c["samples"]):
            require(s["offset_us"] == i * r["step_us"], "candidate time grid")
            mechanisms(s["mechanisms"])


def mechanisms(values: list[dict]) -> None:
    require(len({m["channel"] for m in values}) == len(values), "duplicate mechanism")
    for value in values:
        if value["unit"] == "bool":
            require(value["value"] in (0, 1), "bool mechanism outside {0,1}")


def validate_pair(r: dict, p: dict) -> None:
    validate_request(r)
    validate_schema(p, read_json(ROOT / "schemas/prediction-reply-v1.schema.json"))
    for key in ("schema_version", "model_id", "request_id", "snapshot_id", "epoch", "field", "source", "frame", "timestamp_domain", "history_cutoff_us", "horizon_us", "step_us"):
        require(r[key] == p[key], f"reply mismatch: {key}")
    require(r["issued_us"] <= p["generated_us"] <= p["valid_until_us"] <= r["valid_until_us"], "reply time window")
    expected = {(c["candidate_id"], t["target_id"]) for c in r["candidates"] for t in r["targets"]}
    actual = {(f["candidate_id"], f["target_id"]) for f in p["forecasts"]}
    require(actual == expected and len(p["forecasts"]) == len(expected), "forecast pair dimensions")
    n = r["horizon_us"] // r["step_us"] + 1
    valid_samples = 0
    for f in p["forecasts"]:
        require(len(f["samples"]) == n, "forecast sample dimensions")
        for i, s in enumerate(f["samples"]):
            require(s["offset_us"] == i * r["step_us"], "forecast time grid")
            if s["valid_mask"]:
                valid_samples += 1
                covariance(s["covariance_xx_m2"], s["covariance_xy_m2"], s["covariance_yy_m2"], positive=True)
            else:
                require(all(s[key] == 0 for key in ("x_m", "y_m", "vx_mps", "vy_mps", "covariance_xx_m2", "covariance_xy_m2", "covariance_yy_m2")) and s["confidence"] is None, "invalid mask placeholders")
    require(valid_samples > 0, "no valid forecast")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-dir", type=Path, default=ROOT / "fixtures/prediction")
    args = parser.parse_args()
    r = read_json(args.fixture_dir / "request-v1.json")
    p = read_json(args.fixture_dir / "reply-v1.json")
    validate_pair(r, p)
    # Target plane is signed relative to the robot origin, not necessarily floor z=0.
    signed = copy.deepcopy(r)
    signed["history"][0]["snapshot"]["tracks"][0]["provenance"][0]["target_height_m"] = -0.1
    validate_pair(signed, p)
    for document in (r, p):
        require(json.loads(json.dumps(document, allow_nan=False)) == document, "Python JSON roundtrip")
    cases = []
    def bad_request(key, value):
        d = copy.deepcopy(r); d[key] = value; cases.append((d, p))
    def bad_reply(key, value):
        d = copy.deepcopy(p); d[key] = value; cases.append((r, d))
    bad_request("timestamp_domain", "ROBOT_MONOTONIC_NS")
    bad_request("frame", "RED_FIELD")
    bad_request("step_us", 75_000)
    bad_request("epoch", 8)
    bad_request("unknown_field", "reject")
    bad_reply("request_id", "old-request")
    bad_reply("request_id", " ")
    bad_reply("history_cutoff_us", 1_000_000_000)
    bad_reply("uncertainty_calibration_id", "")
    d = copy.deepcopy(p); d["forecasts"][0]["samples"][0]["x_m"] = float("nan"); cases.append((r, d))
    d = copy.deepcopy(p); d["forecasts"][0]["samples"][0]["covariance_xy_m2"] = 1; cases.append((r, d))
    d = copy.deepcopy(p); d["forecasts"][0]["samples"][0]["valid_mask"] = False; cases.append((r, d))
    d = copy.deepcopy(r); d["history"][0]["accepted_command"]["authority"] = "CANDIDATE"; cases.append((d, p))
    d = copy.deepcopy(r); d["candidates"][0]["authority"] = "ACCEPTED"; cases.append((d, p))
    d = copy.deepcopy(r); d["history"].reverse(); cases.append((d, p))
    for request, reply in cases:
        try:
            validate_pair(request, reply)
        except ValueError:
            continue
        raise AssertionError("invalid prediction vector accepted")
    print(f"Prediction fixtures verified: Java-emitted request/reply, Python roundtrip, {len(cases)} rejection vectors")


if __name__ == "__main__":
    main()
