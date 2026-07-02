from utils import rot_x, rot_z
import numpy as np
from vispy import scene, visuals
from orbit import KeplerianElements


#Support Function
def sample_true_anomaly(n_points: int = 800) -> np.ndarray:
    """Generate true anomaly samples for one full orbit."""
    return np.linspace(0.0, 2 * np.pi, n_points)

#Support Function
def compute_radius(a_km: float, e: float, v: np.ndarray) -> np.ndarray:
    """
    Keplerian orbit radius equation.

    TODO: Replace with orbit.true_anom2radius once available.
    """
    return a_km * (1 - e**2) / (1 + e * np.cos(v))

#Support Function
def perifocal_to_eci(points_pf: np.ndarray, kep: KeplerianElements) -> np.ndarray:
    """
    Convert perifocal coordinates to ECI frame.

    TODO: Replace with orbit.keplerian2cartesian once implemented.
    """
    R = rot_z(kep.Om_rad) @ rot_x(kep.i_rad) @ rot_z(kep.om_rad)
    return points_pf @ R.T

#The Main Function 
def keplerian_to_cartesian_orbit(
    kep: KeplerianElements,
    n_points: int = 800
) -> np.ndarray:
    """
    Generate full Keplerian orbit as Cartesian ECI points.

    NOTE:
    - Uses true anomaly sampling (no time propagation)
    - Pure Keplerian model (no perturbations)
    """
    v = sample_true_anomaly(n_points)

    r = compute_radius(kep.a_km, kep.e, v)

    # Perifocal frame
    x_pf = r * np.cos(v)
    y_pf = r * np.sin(v)
    z_pf = np.zeros_like(v)

    points_pf = np.column_stack((x_pf, y_pf, z_pf))

    return perifocal_to_eci(points_pf, kep)



# Vispy Rendering
def create_earth(view):
    """Create Earth sphere in scene.
    TODO: Replace with Earth texture mapping once available.
    """

    earth = scene.visuals.Sphere(
        radius=6371,
        rows=60,
        cols=60,
        method='latitude',
        parent=view.scene
    )

    earth.mesh.color = (0.2, 0.5, 1, 0.6)

    wireframe = visuals.filters.WireframeFilter(
        width=1,
        color=(0.3, 0.6, 1, 0.2)
    )
    earth.mesh.attach(wireframe)

    return earth


def create_camera(view):
    """Setup 3D interactive camera."""
    view.camera = 'turntable'
    view.camera.distance = 20000
    view.camera.fov = 60


def plot_keplerian_orbit(kep: KeplerianElements):
    """
    Render a full 3D Keplerian orbit using VisPy.

    Parameters
    ----------
    kep : KeplerianElements
        Orbital elements defining the orbit.
    """
    points = keplerian_to_cartesian_orbit(kep)

    canvas = scene.SceneCanvas(
        keys='interactive',
        show=True,
        bgcolor='black'
    )

    view = canvas.central_widget.add_view()

    create_camera(view)
    create_earth(view)

    # Orbit line
    scene.visuals.Line(
        pos=points,
        color='red',
        width=2,
        parent=view.scene
    )

    # Reference axes
    axis = scene.visuals.XYZAxis(parent=view.scene)
    axis.transform = scene.transforms.STTransform(scale=(8000, 8000, 8000))

    canvas.app.run()


# ============================================================
# TEST RUN
# ============================================================

if __name__ == "__main__":
    kep = KeplerianElements(
        a_km=6771,
        e=0.001,
        i_rad=np.radians(51.6),
        Om_rad=0.0,
        om_rad=0.0
    )

    plot_keplerian_orbit(kep)