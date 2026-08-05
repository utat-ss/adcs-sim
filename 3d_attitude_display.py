import vispy
vispy.use('PySide6')

from vispy import app, scene
from vispy.visuals.transforms import MatrixTransform
import numpy as np

def plot_vector(
        vector, parent, label_text, 
        vector_colour=[0.0, 0.0, 0.0], 
        line_thickeness=3.0,
        line_length=1.2, 
        label_offset=1.1,
        use_unit_vector=True
    ):

    if use_unit_vector:
        norm = np.linalg.norm(vector)
        if norm > 0:
            vector = vector / norm

    pos = np.array([[0.0, 0.0, 0.0], vector * line_length])

    scene.visuals.Line(
        pos=pos,
        color=vector_colour,
        width=line_thickeness,
        parent=parent
    )

    label_pos = vector * line_length * label_offset

    scene.visuals.Text(
        text=label_text,
        pos=label_pos,
        color=vector_colour,
        font_size=14,
        bold=True,
        parent=parent,
    )
     
def plot_triad(
        rotation_matrix, parent, label_texts, 
        colour_alpha=1.0, 
        line_thickeness=3.0, 
        line_length=1.0, 
        label_offset=1.1
    ):

    # initialize variables
    x_point = rotation_matrix[:, 0]
    y_point = rotation_matrix[:, 1]
    z_point = rotation_matrix[:, 2]

    x_colour = [1.0, 0.0, 0.0, colour_alpha] # red for x-axis
    y_colour = [0.0, 1.0, 0.0, colour_alpha] # green for y-axis
    z_colour = [0.0, 0.0, 1.0, colour_alpha] # blue for z-axis

    # draw axis
    axis_point_pairs = np.array([
        [0, 0, 0], x_point * line_length,
        [0, 0, 0], y_point * line_length,
        [0, 0, 0], z_point * line_length
    ]) # set points into pairs to draw the three axis

    colours = np.array([
        x_colour, x_colour,
        y_colour, y_colour,
        z_colour, z_colour
    ])

    # render 3 axis
    scene.visuals.Line(
        pos=axis_point_pairs, 
        color=colours, 
        connect='segments', 
        width=line_thickeness, 
        parent=parent
    )


    # draw labels
    label_positions = np.array([
        x_point * line_length * label_offset,
        y_point * line_length * label_offset,
        z_point * line_length * label_offset
    ])

    label_colors = [x_colour, y_colour, z_colour]

    # render labels
    scene.visuals.Text(
        text=label_texts, 
        pos=label_positions, 
        color=label_colors, 
        font_size=12, 
        bold=True, 
        parent=parent
    )

def plot_cubeset(
        parent, rotation_matrix,
        cubeset_dimensions=(1.0, 1.0, 1.0), # (width, height, depth)
        cubeset_body_colour = [0.5, 0.5, 0.5, 0.2],
        cubeset_edge_colour = [0.4, 0.4, 0.4, 0.5]
    ):

    cube = scene.visuals.Box(
        width=cubeset_dimensions[0], height=cubeset_dimensions[1], depth=cubeset_dimensions[2], 
        color=cubeset_body_colour, edge_color=cubeset_edge_colour, 
        parent=parent
    )

    transformation_matrix = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    transformation_matrix[:3, :3] = rotation_matrix.T
    cube.transform = MatrixTransform(transformation_matrix)

def plot_attitude(
    # Physical Satellite Orientation (3x3 Matrix)
    body,  
    
    # Reference Frame Orientations (3x3 Matrices)
    eci = None,
    ecef = None,
    lvlh = None,

    # Sun and Target Vectors
    sun_vector = None, 
    target_vector = None,

    # Display Toggles
    show_eci  = True,  # Default
    show_ecef = False,
    show_lvlh = False,
    show_sun_vector = False,
    show_target_vector = False,

    # Grid Size
    grid_size = (800, 600) # Default
):

    '''
    Render a static 3D snapshot of the spacecraft's attitude.
    Displays the spacecraft body frame as a triad (with a grey box representing the satellite bus).

    x-axis: red, y-axis: green, z-axis: blue
    '''

    if not (show_eci or show_ecef or show_lvlh):
        raise ValueError("At least one reference frame (ECI, ECEF, or LVLH) must be True.")
    
    canvas = scene.SceneCanvas(keys='interactive', bgcolor='white', size=grid_size, show=True)
    view = canvas.central_widget.add_view()
    view.camera = scene.cameras.ArcballCamera(fov=45)

    plot_triad(body, view.scene, ["x_body", "y_body", "z_body"])

    if show_eci:
        if eci is None:
            eci = np.array([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
            ])

        plot_triad(
            eci, view.scene, ["x_eci", "y_eci", "z_eci"], 
            colour_alpha=0.5, line_thickeness=1.5, line_length=1.5
        )

    if show_ecef and ecef is not None:
        plot_triad(
            ecef, view.scene, ["x_ecef", "y_ecef", "z_ecef"], 
            colour_alpha=0.5, line_thickeness=1.5, line_length=1.5
        )

    if show_lvlh and lvlh is not None:
        plot_triad(
            lvlh, view.scene, ["x_lvlh", "y_lvlh", "z_lvlh"], 
            colour_alpha=0.5, line_thickeness=1.5, line_length=1.5
        )

    if show_sun_vector and sun_vector is not None:
        plot_vector(sun_vector, view.scene, "sun_vector")

    if show_target_vector and target_vector is not None:
        plot_vector(target_vector, view.scene, "target_vector")

    plot_cubeset(view.scene, body)

    app.run()

plot_attitude(
np.array([
    [ 0.61237244, -0.70710678,  0.35355339],
    [ 0.61237244,  0.70710678,  0.35355339],
    [-0.50000000,  0.00000000,  0.86602540]
]))