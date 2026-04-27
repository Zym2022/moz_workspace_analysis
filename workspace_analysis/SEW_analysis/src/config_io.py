#!/usr/bin/env python3
"""Configuration loading with a stdlib fallback."""

import json


def load_config(config_path):
    """Load YAML config, falling back to JSON-compatible YAML without PyYAML."""
    text = open(config_path, "r", encoding="utf-8").read()
    try:
        import yaml
    except ModuleNotFoundError:
        return json.loads(text)
    return yaml.safe_load(text)
