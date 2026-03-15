"""
Tests for the new_real_push_env public API boundary.
"""


class TestPackageLevelExports:
    """Only the contracted public names should appear in __all__."""

    ALLOWED = {"NewRealPushEnv", "NewRealPushConfig", "v3_config", "v4_config"}

    def test_all_contains_only_allowed(self):
        import micromvp.env.new_real_push_env as pkg

        assert hasattr(pkg, "__all__")
        exported = set(pkg.__all__)
        assert exported == self.ALLOWED, (
            f"Unexpected exports: {exported - self.ALLOWED}, "
            f"Missing exports: {self.ALLOWED - exported}"
        )

    def test_internal_classes_not_in_all(self):
        import micromvp.env.new_real_push_env as pkg

        forbidden = [
            "ArucoObserver",
            "ObserverConfig",
            "CarObservation",
            "WorkspaceEstimate",
            "SerialActionSender",
            "SerialActionConfig",
            "APStatusInfo",
        ]
        exported = set(pkg.__all__)
        for name in forbidden:
            assert name not in exported, f"{name} should not be in __all__"

    def test_ap_status_removed(self):
        """APStatusInfo should not exist anywhere in the package."""
        import importlib
        mod = importlib.import_module("micromvp.env.new_real_push_env.serial_action")
        assert not hasattr(mod, "APStatusInfo")

    def test_env_level_init_does_not_export_new_real_push_env(self):
        """NewRealPushEnv is experimental, should not be in env/__init__."""
        import micromvp.env as env_pkg

        if hasattr(env_pkg, "__all__"):
            assert "NewRealPushEnv" not in env_pkg.__all__
