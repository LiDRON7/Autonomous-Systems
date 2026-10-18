"""Draft 2020-12 scenario schema plus finite-number and semantic checks."""
import json
import math
from pathlib import Path


def load_scenario(path):
    from jsonschema import Draft202012Validator
    config = json.loads(Path(path).read_text())
    schema = json.loads(Path(__file__).with_name('scenario.schema.json').read_text())
    error = next(Draft202012Validator(schema).iter_errors(config), None)
    if error:
        raise ValueError(error.message)
    def finite(value):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError('numeric values must be finite')
        if isinstance(value, dict):
            for item in value.values():
                finite(item)
        elif isinstance(value, list):
            for item in value:
                finite(item)
    finite(config)
    expected = config['expected']
    if set(expected['states']) & set(expected['forbidden_states']):
        raise ValueError('required and forbidden states overlap')
    if config['version'] != 1 or type(config['version']) is not int:
        raise ValueError('only integer scenario version 1 is supported')
    return config
