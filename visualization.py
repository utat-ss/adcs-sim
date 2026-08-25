import numpy as np
from vispy import scene, visuals
from vispy.visuals.filters import TextureFilter
import imageio.v3 as iio
from orbit import KeplerianElements, keplerian2cartesian
from astropy.time import Time

#Temporary Function (will be replaced with time anomaly sampling once available)
def sample_true_anomaly(n_points: int = 800) -> np.ndarray:
    """Generate true anomaly samples for one full orbit.
    TODO: Replace with time anomaly once orbit.mean_anom2ecc_anom implements"""
    return np.linspace(0.0, 2 * np.pi, n_points)

#Support Function for coordinates conversion
def keplerian_2_cartesian_orbit(
    kep: KeplerianElements,
    n_points: int = 800,
    mu_km3_s2: float = 398600.4418
) -> np.ndarray:

    nus = sample_true_anomaly(n_points)
    orbit_points = []

    for nu in nus:
        state = keplerian2cartesian(kep, nu, mu_km3_s2)
        position = state[:3]      
        orbit_points.append(position)

    return np.array(orbit_points, dtype=float)

def build_earth_mesh(radius=6371, rows=60, cols=60, gmst_rad=0.0):
    """Build a lat/lon sphere mesh with texture coordinates aligned to the ECI frame.

    gmst_rad: Greenwich Mean Sidereal Time (radians) at the epoch being plotted.
        The texture image is fixed to Earth (ECEF), where longitude 0 is the
        Greenwich meridian. The ECI X axis instead points at the vernal equinox,
        a fixed direction in inertial space. gmst_rad rotates the Earth-fixed
        texture into the correct position under the ECI axes at this epoch.
        Pass 0.0 to align the prime meridian directly with the X axis.
    """
    lats = np.linspace(-np.pi / 2, np.pi / 2, rows + 1)
    lons = np.linspace(-np.pi, np.pi, cols + 1)

    vertices = []
    texcoords = []
    for lat in lats:
        for lon in lons:
            eci_lon = lon + gmst_rad  # rotate Earth-fixed longitude into ECI
            x = radius * np.cos(lat) * np.cos(eci_lon)
            y = radius * np.cos(lat) * np.sin(eci_lon)
            z = radius * np.sin(lat)
            vertices.append([x, y, z])

            u = (lon + np.pi) / (2 * np.pi)
            v = 1.0 - (lat + np.pi / 2) / np.pi
            texcoords.append([u, v])

    vertices = np.array(vertices, dtype=np.float32)
    texcoords = np.array(texcoords, dtype=np.float32)

    faces = []
    n_cols = cols + 1
    for i in range(rows):
        for j in range(cols):
            a = i * n_cols + j
            b = a + 1
            c = a + n_cols
            d = c + 1
            faces.append([a, b, c])
            faces.append([b, d, c])
    faces = np.array(faces, dtype=np.uint32)

    return vertices, faces, texcoords

#Create earth sphere and wireframe
def create_earth(view, texture_path='assets/earth.jpg', gmst_rad=0.0):
    """Create Earth sphere in scene, textured with an accurate surface map
    oriented in the ECI frame at the given GMST.
    """
    vertices, faces, texcoords = build_earth_mesh(
        radius=6371, rows=60, cols=60, gmst_rad=gmst_rad
    )

    texture_data = iio.imread(texture_path)

    earth = scene.visuals.Mesh(
        vertices=vertices,
        faces=faces,
        parent=view.scene
    )

    texture_filter = TextureFilter(texture_data, texcoords)
    earth.attach(texture_filter)

    return earth

#Create camera and set view parameters
def create_camera(view):
    """Setup 3D interactive camera."""
    view.camera = 'turntable'
    view.camera.distance = 20000
    view.camera.fov = 60

#Create axes legend
def create_axis_legend(canvas):
    """Create legend for ECI reference axes."""

    scene.visuals.Text(
        'X axis',
        color='red',
        font_size=12,
        pos=(20, 20),
        parent=canvas.scene,
        method='cpu',
        anchor_x='left',
        anchor_y='center'
    )

    scene.visuals.Text(
        'Y axis',
        color='green',
        font_size=12,
        pos=(20, 40),
        parent=canvas.scene,
        method='cpu',
        anchor_x='left',
        anchor_y='center'
    )

    scene.visuals.Text(
        'Z axis',
        color='blue',
        font_size=12,
        pos=(20, 60),
        parent=canvas.scene,
        method='cpu',
        anchor_x='left',
        anchor_y='center'
    )

#The Main Function
def plot_keplerian_orbit(kep: KeplerianElements):
    """Render a full 3D Keplerian orbit using VisPy."""

    points = keplerian_2_cartesian_orbit(kep)
    epoch = Time('2026-08-08T00:00:00', scale='utc')
    gmst_rad = epoch.sidereal_time('mean', 'greenwich').radian

    canvas = scene.SceneCanvas(
        keys='interactive',
        show=True,
        bgcolor='black'
    )

    view = canvas.central_widget.add_view()

    create_camera(view)
    earth = create_earth(view, texture_path='assets/earth.jpg', gmst_rad=gmst_rad)

    # Orbit
    scene.visuals.Line(
        pos=points,
        color='cyan',
        width=3,
        parent=view.scene
    )

    # ECI reference axes
    axis = scene.visuals.XYZAxis(parent=view.scene)
    axis.transform = scene.transforms.STTransform(
        scale=(12000, 12000, 12000)
    )

    # Axis legend
    create_axis_legend(canvas)

    canvas.app.run()