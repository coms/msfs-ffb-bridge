"""Tests for configuration, profiles and their JSON round trip."""

from __future__ import annotations

import json

from ffbbridge.core.config import (
    BridgeConfig,
    ModuleSettings,
    ProfileSet,
    RoutingConfig,
    SafetyConfig,
)


def test_profile_set_round_trips_through_json(tmp_path):
    profiles = ProfileSet(
        default=BridgeConfig(name="Default GA"),
        profiles=[
            BridgeConfig(
                name="Spitfire",
                match=["*spitfire*"],
                safety=SafetyConfig(master_gain=0.9),
                modules={"ground_roll": ModuleSettings(gain=1.4, params={"max_hz": 40.0})},
            )
        ],
    )
    path = tmp_path / "profiles.json"
    profiles.save(path)
    loaded = ProfileSet.load(path)

    assert loaded.default.name == "Default GA"
    assert loaded.profiles[0].safety.master_gain == 0.9
    assert loaded.profiles[0].modules["ground_roll"].params["max_hz"] == 40.0


def test_profile_matching_is_case_insensitive_and_checks_both_names():
    profile = BridgeConfig(name="Cessna", match=["*C172*"])
    assert profile.matches("Cessna Skyhawk c172 Asobo", "")
    assert profile.matches("", "C172")
    assert not profile.matches("Boeing 747", "B748")


def test_profile_without_match_patterns_never_matches():
    """The default profile is chosen by fallback, not by matching everything."""
    assert not BridgeConfig(name="Default").matches("anything", "at all")


def test_select_prefers_first_matching_profile():
    profiles = ProfileSet(
        default=BridgeConfig(name="Default"),
        profiles=[
            BridgeConfig(name="Specific", match=["*c172*"]),
            BridgeConfig(name="Generic", match=["*cessna*"]),
        ],
    )
    assert profiles.select("Cessna C172", "").name == "Specific"
    assert profiles.select("Cessna 208", "").name == "Generic"
    assert profiles.select("Airbus A320", "").name == "Default"


def test_module_defaults_are_filled_in_without_losing_overrides():
    defaults = {
        "ground_roll": ModuleSettings(enabled=True, gain=1.0, params={"max_hz": 32.0}),
        "touchdown": ModuleSettings(enabled=True, gain=1.0),
    }
    config = BridgeConfig(modules={"ground_roll": ModuleSettings(gain=1.5, params={})})
    merged = config.with_module_defaults(defaults)

    assert merged.modules["ground_roll"].gain == 1.5
    assert merged.modules["ground_roll"].params["max_hz"] == 32.0  # default preserved
    assert "touchdown" in merged.modules


def test_unknown_modules_survive_a_round_trip():
    """A profile written by a newer build must not lose settings in an older one."""
    config = BridgeConfig(modules={"from_the_future": ModuleSettings(gain=0.5)})
    merged = config.with_module_defaults({"ground_roll": ModuleSettings()})
    assert "from_the_future" in merged.modules


def test_load_or_default_survives_a_corrupt_file(tmp_path):
    path = tmp_path / "broken.json"
    path.write_text("{not json at all", encoding="utf-8")
    assert ProfileSet.load_or_default(path).default.name == "Default GA"


def test_load_or_default_survives_a_missing_file(tmp_path):
    assert ProfileSet.load_or_default(tmp_path / "nope.json").default.name == "Default GA"


def test_safety_values_are_clamped_on_load():
    safety = SafetyConfig.from_dict({"master_gain": 5.0, "max_force": -1.0, "decay_ms": 0.0})
    assert safety.master_gain == 1.0
    assert safety.max_force == 0.0
    assert safety.decay_ms >= 1.0


def test_routing_rejects_an_unknown_mode():
    assert RoutingConfig.from_dict({"mode": "nonsense"}).mode == "auto"
    assert RoutingConfig.from_dict({"mode": "rudder_only"}).mode == "rudder_only"


def test_config_dict_is_json_serialisable():
    payload = ProfileSet().to_dict()
    assert json.loads(json.dumps(payload)) == payload


def test_module_accessor_creates_settings_on_demand():
    config = BridgeConfig()
    settings = config.module("brand_new")
    settings.gain = 0.25
    assert config.modules["brand_new"].gain == 0.25


def test_soft_lock_survives_the_round_trip_and_reads_as_a_fraction():
    from ffbbridge.core.config import WheelConfig

    wheel = WheelConfig(rotation_deg=540.0, soft_lock_deg=180.0)
    assert wheel.soft_lock_fraction == 1 / 3
    assert WheelConfig.from_dict(wheel.to_dict()) == wheel
    assert WheelConfig().soft_lock_fraction == 0.0
