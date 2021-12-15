import lib.build.libry as ry


def set_frame_properties(obj: ry.Frame, shape=None, size=None, color=None, position=None,
                         quaternion=None, mass=None):
    if shape is not None and size is not None:
        obj.setShape(shape, size)
    if color is not None:
        obj.setColor(color / 255.)  # Rai need normalized float colors
    if position is not None:
        obj.setPosition(position)
    if quaternion is not None:
        obj.setQuaternion(quaternion)
    if mass is not None:
        obj.setMass(mass)
    return obj


def create_k_markers(rai_config: ry.Config, k: int, name: str = "tracker", parent: str = "", color=[1, 0, 0], size=0.2):
    markers = []
    for i in range(k):
        markers.append(rai_config.addFrame("{}_{}".format(name, i), parent))
        markers[i].setShape(ry.ST.marker, [size])
        markers[i].setColor(color)
    return markers
