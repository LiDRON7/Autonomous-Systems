from navigation.geo import gps_to_ned


def test_reference_maps_to_origin():
    assert gps_to_ned(18.0, -67.0, 10.0, (18.0, -67.0, 10.0)) == (0.0, 0.0, 0.0)


def test_altitude_converts_to_down():
    _, _, down = gps_to_ned(18.0, -67.0, 15.0, (18.0, -67.0, 10.0))
    assert down == -5.0


def test_north_and_east_offsets_have_expected_signs():
    north, east, _ = gps_to_ned(18.001, -66.999, 10.0, (18.0, -67.0, 10.0))
    assert north > 0.0
    assert east > 0.0
