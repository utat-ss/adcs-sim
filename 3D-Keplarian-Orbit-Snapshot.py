import numpy as np
from vispy import scene, visuals
from orbit import KeplerianElements, keplerian2cartesian

#Support Function
def sample_true_anomaly(n_points: int = 800) -> np.ndarray:
    """Generate true anomaly samples for one full orbit.
    TODO: Replace with time anomaly sampling once available."""
    return np.linspace(0.0, 2 * np.pi, n_points)

#The Main Function 
def keplerian_to_cartesian_orbit(
    kep: KeplerianElements,
    n_points: int = 800,
    mu_km3_s2: float = 398600.4418
) -> np.ndarray:

    nus = sample_true_anomaly(n_points)

    orbit_points = []

    for nu in nus:
        state = keplerian2cartesian(
            kep,
            nu,
            mu_km3_s2
        )

        position = state[:3]      #as assumed by keplerian2cartesian

        orbit_points.append(position)

    return np.array(orbit_points, d_type=float)



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


# TEST RUN

if __name__ == "__main__":
    kep = KeplerianElements(
        a_km=6771,
        e=0.001,
        i_rad=np.radians(51.6),
        Om_rad=0.0,
        om_rad=0.0
    )

    plot_keplerian_orbit(kep)