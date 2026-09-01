from lidron_navigation.geo import gps_to_ned


def test_reference_maps_to_origin():
    assert gps_to_ned(18.0, -67.0, 10.0, (18.0, -67.0, 10.0)) == (0.0, 0.0, 0.0)


def test_altitude_converts_to_down():
    _, _, down = gps_to_ned(18.0, -67.0, 15.0, (18.0, -67.0, 10.0))
    assert down == -5.0
