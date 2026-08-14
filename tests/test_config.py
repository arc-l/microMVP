"""Tests for the deployment config layer.

The promise this layer makes is that a wrong or missing field fails loudly
and says what to fix, so these tests care as much about the error messages
as about the happy path.
"""
import copy

import pytest
import yaml

from micromvp.config import Config, ConfigError, load_config

SHIPPED_CONFIG = "config/car_v4.yaml"


@pytest.fixture
def data():
    with open(SHIPPED_CONFIG, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@pytest.fixture
def cfg(data):
    return Config(data, SHIPPED_CONFIG)


class TestLookup:
    def test_reads_a_nested_field(self, cfg):
        assert cfg.require("car.marker_size_mm", float) == 36.0

    def test_reads_a_deeply_nested_field(self, cfg):
        assert cfg.require("workspace.tolerance.origin_m", float) == 0.01

    def test_require_pair_returns_floats(self, cfg):
        assert cfg.require_pair("car.marker_to_axle_offset_cm") == (0.0, 1.8)

    def test_has_reports_presence(self, cfg):
        assert cfg.has("car.wheel_base_cm")
        assert not cfg.has("car.nonexistent")

    def test_optional_falls_back(self, cfg):
        assert cfg.optional("car.nonexistent", 7.0) == 7.0

    def test_null_stays_null(self, cfg):
        # lookahead_cm is deliberately nullable: null means "derive it"
        assert cfg.require("control.lookahead_cm") is None


class TestMissingField:
    def test_raises(self, data):
        del data["car"]["marker_size_mm"]
        with pytest.raises(ConfigError) as exc:
            Config(data, SHIPPED_CONFIG).require("car.marker_size_mm", float)
        assert "car.marker_size_mm" in str(exc.value)

    def test_message_names_the_caller_and_the_file(self, data):
        del data["car"]["marker_size_mm"]
        with pytest.raises(ConfigError) as exc:
            Config(data, SHIPPED_CONFIG).require(
                "car.marker_size_mm", float, who="ArucoObserver"
            )
        message = str(exc.value)
        assert "ArucoObserver" in message
        assert SHIPPED_CONFIG in message

    def test_message_lists_siblings_to_expose_typos(self, data):
        del data["car"]["wheel_base_cm"]
        with pytest.raises(ConfigError) as exc:
            Config(data, SHIPPED_CONFIG).require("car.wheel_base_cm", float)
        assert "body_width_cm" in str(exc.value)

    def test_missing_whole_section_is_reported(self, data):
        del data["workspace"]
        with pytest.raises(ConfigError) as exc:
            Config(data, SHIPPED_CONFIG).require("workspace.margin_cm", float)
        assert "workspace.margin_cm" in str(exc.value)


class TestTypeChecking:
    def test_rejects_a_string_where_a_number_belongs(self, data):
        data["camera"]["device"] = "zero"
        with pytest.raises(ConfigError) as exc:
            Config(data, SHIPPED_CONFIG).require("camera.device", int)
        assert "must be an integer" in str(exc.value)

    def test_bool_is_not_an_int(self, data):
        data["camera"]["device"] = True
        with pytest.raises(ConfigError):
            Config(data, SHIPPED_CONFIG).require("camera.device", int)

    def test_int_is_accepted_as_float(self, data):
        data["car"]["marker_size_mm"] = 36
        assert Config(data, SHIPPED_CONFIG).require("car.marker_size_mm", float) == 36.0

    def test_number_is_not_a_bool(self, data):
        data["camera"]["undistort"] = 1
        with pytest.raises(ConfigError) as exc:
            Config(data, SHIPPED_CONFIG).require("camera.undistort", bool)
        assert "must be a boolean" in str(exc.value)

    def test_pair_rejects_wrong_length(self, data):
        data["car"]["axle_offset_cm"] = [1.0, 2.0, 3.0]
        with pytest.raises(ConfigError):
            Config(data, SHIPPED_CONFIG).require_pair("car.axle_offset_cm")


class TestUnusedFields:
    def test_reports_a_field_nobody_read(self, cfg):
        cfg.require("car.marker_size_mm", float)
        assert "car.wheel_base_cm" in cfg.unused_fields()

    def test_reading_a_block_counts_for_everything_inside_it(self, cfg):
        cfg.require("obstacle.shapes", dict)
        unused = cfg.unused_fields()
        assert not [f for f in unused if f.startswith("obstacle.shapes")]

    def test_catches_a_typo(self, data):
        data["camera"]["framerate"] = 60  # the real field is `fps`
        cfg = Config(data, SHIPPED_CONFIG)
        cfg.require("camera.fps", int)
        assert "camera.framerate" in cfg.unused_fields()


class TestLoadConfig:
    def test_missing_file_explains_where_it_looked(self):
        with pytest.raises(ConfigError) as exc:
            load_config("config/does_not_exist.yaml")
        message = str(exc.value)
        assert "does_not_exist.yaml" in message
        assert "config/car_v4.yaml" in message  # points at a working example

    def test_shipped_config_loads(self):
        cfg = load_config(SHIPPED_CONFIG)
        assert cfg.source == SHIPPED_CONFIG


class TestShippedConfigIsComplete:
    """The shipped config must satisfy every module that reads it."""

    def test_builds_every_config_object(self, data):
        from micromvp.env.real_env.observer import ObserverConfig
        from micromvp.env.real_env.serial_action import SerialActionConfig

        observer = ObserverConfig.from_config(Config(data, SHIPPED_CONFIG))
        assert observer.car_marker_size_mm > 0
        assert observer.obstacle_shapes  # at least one obstacle registered

        # Pin the port so this test does not depend on what happens to be
        # plugged into the machine running it.
        raw = copy.deepcopy(data)
        raw["actuation"]["serial_port"] = "/dev/null"
        sender = SerialActionConfig.from_config(Config(raw, SHIPPED_CONFIG))
        assert sender.port == "/dev/null"

    def test_car_and_obstacle_use_different_dictionaries(self):
        cfg = load_config(SHIPPED_CONFIG)
        car = cfg.require("car.aruco_dict", str)
        obstacle = cfg.require("obstacle.aruco_dict", str)
        assert car != obstacle, (
            "cars and obstacles must use different ArUco dictionaries, "
            "otherwise ids collide between them"
        )
