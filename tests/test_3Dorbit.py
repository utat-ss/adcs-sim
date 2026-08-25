import numpy as np
import pytest 

from visualization import (
    sample_true_anomaly,
    keplerian_2_cartesian_orbit,
    build_earth_mesh,
)
from orbit import KeplerianElements


def test_sample_true_anomaly_returns_default_length():
    nus = sample_true_anomaly()
    assert len(nus) == 800


def test_sample_true_anomaly_returns_custom_length():
    nus = sample_true_anomaly(n_points=100)
    assert len(nus) == 100


def test_sample_true_anomaly_covers_full_orbit():
    nus = sample_true_anomaly(n_points=50)
    assert nus[0] == pytest.approx(0.0)
    assert nus[-1] == pytest.approx(2 * np.pi)


def test_sample_true_anomaly_is_monotonically_increasing():
    nus = sample_true_anomaly(n_points=200)
    assert np.all(np.diff(nus) > 0)


def test_keplerian_2_cartesian_orbit_returns_correct_shape():
    kep = KeplerianElements(
        a_km=6771,
        e=0.001,
        i_rad=np.radians(51.6),
        Om_rad=0.0,
        om_rad=0.0,
    )
    points = keplerian_2_cartesian_orbit(kep, n_points=100)
    assert points.shape == (100, 3)


def test_keplerian_2_cartesian_orbit_uses_default_n_points():
    kep = KeplerianElements(
        a_km=6771,
        e=0.001,
        i_rad=np.radians(51.6),
        Om_rad=0.0,
        om_rad=0.0,
    )
    points = keplerian_2_cartesian_orbit(kep)
    assert points.shape == (800, 3)


def test_keplerian_2_cartesian_orbit_radius_matches_semi_major_axis_for_near_circular_orbit():
    kep = KeplerianElements(
        a_km=6771,
        e=0.001,
        i_rad=np.radians(51.6),
        Om_rad=0.0,
        om_rad=0.0,
    )
    points = keplerian_2_cartesian_orbit(kep, n_points=200)
    radii = np.linalg.norm(points, axis=1)

    for radius in radii:
        assert radius == pytest.approx(kep.a_km, rel=0.01)


def test_keplerian_2_cartesian_orbit_start_and_end_points_coincide():
    kep = KeplerianElements(
        a_km=6771,
        e=0.001,
        i_rad=np.radians(51.6),
        Om_rad=0.0,
        om_rad=0.0,
    )
    points = keplerian_2_cartesian_orbit(kep, n_points=500)
    assert points[0] == pytest.approx(points[-1], abs=1.0)


def test_build_earth_mesh_returns_correct_shapes():
    rows, cols = 10, 10
    vertices, faces, texcoords = build_earth_mesh(rows=rows, cols=cols)
    n_verts = (rows + 1) * (cols + 1)

    assert vertices.shape == (n_verts, 3)
    assert texcoords.shape == (n_verts, 2)
    assert faces.shape[1] == 3


def test_build_earth_mesh_vertices_lie_on_sphere_surface():
    radius = 6371
    vertices, _, _ = build_earth_mesh(radius=radius, rows=20, cols=20)
    radii = np.linalg.norm(vertices, axis=1)

    for r in radii:
        assert r == pytest.approx(radius, abs=1e-6)


def test_build_earth_mesh_texcoords_are_within_unit_range():
    _, _, texcoords = build_earth_mesh(rows=15, cols=15)

    assert np.all(texcoords >= 0.0)
    assert np.all(texcoords <= 1.0)


def test_build_earth_mesh_gmst_rotation_changes_vertex_positions_but_not_radius():
    v0, _, _ = build_earth_mesh(rows=10, cols=10, gmst_rad=0.0)
    v1, _, _ = build_earth_mesh(rows=10, cols=10, gmst_rad=np.pi / 4)

    assert not np.allclose(v0, v1)
    assert np.allclose(np.linalg.norm(v0, axis=1), np.linalg.norm(v1, axis=1))