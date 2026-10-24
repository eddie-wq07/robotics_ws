"""The surface a marker is drawn on: a flat tile, a can or a ball.

A marker lives in its own flat coordinates (s, t): s to the reader's right and
t up, in metres, origin at the marker centre. Each surface maps (s, t) and a
height ``d`` above the surface to a point in the build frame (origin at the
top centre of the build plate, z up), and gives the outward normal there. The
pen is held along the inward normal, so it stands perpendicular to the surface.

* plane: a tile lying on the plate; the map is a rigid placement.
* cylinder: a can standing on the plate; s wraps around it, t runs up it. The
  map is isometric (unrolling a cylinder does not stretch it).
* sphere: a ball resting on the plate; stereographic from the marker centre,
  scaled so the centre is stretched as much as the edge is shrunk. It is
  conformal (no direction is stretched more than another) and its length error
  grows with the marker's angular size.
"""

from pathlib import Path

import numpy as np
import trimesh
import yaml


# Largest angle (radians) a marker may span from its centre on a ball: at most
# 4.5 percent length error with the balanced scaling.
SPHERE_MAX_ANGLE = 0.60
# A globe's picture is already a map: up to the equator, 90 degrees from the pole.
GLOBE_MAX_ANGLE = np.pi / 2 + 0.01
# Largest angle around a can from the marker centre: past this the pen would
# approach the can nearly edge-on from the arm's side.
CYLINDER_MAX_ANGLE = np.pi / 3


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


class Surface:
    kind = "surface"

    def __init__(self, spec):
        self.spec = spec
        # ``raise_m`` lifts the object off the plate: a riser, or a measured correction.
        self.position = np.array([*spec.get("position_m", [0.0, 0.0]),
                                  spec.get("raise_m", 0.0)], float)

    # Every map is vectorised over arrays of s and t.
    def point(self, s, t, d=0.0):
        raise NotImplementedError

    def normal(self, s, t):
        raise NotImplementedError

    def frame(self, s, t):
        """Unit tangent along s, unit tangent along t and the normal, each (n, 3)."""
        raise NotImplementedError

    def offset_scale(self, d):
        """How much longer a length in (s, t) is at height ``d`` than on the surface."""
        return 1.0

    def scale(self, s, t):
        """Length on the surface per length in (s, t) at this point (1 = exact)."""
        return np.ones_like(np.asarray(s, float))

    def fit(self, radius, extent=None):
        """Check that a marker reaching ``radius`` from its centre fits; return notes.

        ``extent`` is (largest |s|, lowest t, highest t) where a surface needs it.
        """
        return {}

    def height_and_normal(self, points):
        """Height above the surface and the outward normal under each build-frame point."""
        raise NotImplementedError

    def mesh(self):
        raise NotImplementedError

    def describe(self):
        return {"type": self.kind, **self.spec}

    def curvature_radius(self):
        return None


class Plane(Surface):
    """A tile lying on the plate; the marker is on its top face."""

    kind = "plane"

    def __init__(self, spec):
        super().__init__(spec)
        self.size = np.asarray(spec["size_m"], float)  # width, depth, thickness
        marker = spec.get("marker", {})
        rotation = np.radians(marker.get("rotation_deg", 0.0))
        # Read by someone standing in front of the robot (looking toward -x):
        # their right is +y and "up the page" is away from them, -x.
        right, up = np.array([0.0, 1.0, 0.0]), np.array([-1.0, 0.0, 0.0])
        c, s = np.cos(rotation), np.sin(rotation)
        self.e_s, self.e_t = c * right + s * up, -s * right + c * up
        self.origin = self.position + [*marker.get("center_m", [0.0, 0.0]), self.size[2]]

    def point(self, s, t, d=0.0):
        s, t, d = np.broadcast_arrays(np.asarray(s, float), np.asarray(t, float), d)
        return (self.origin + s[..., None] * self.e_s + t[..., None] * self.e_t
                + d[..., None] * np.array([0.0, 0.0, 1.0]))

    def normal(self, s, t):
        s = np.asarray(s, float)
        return np.broadcast_to([0.0, 0.0, 1.0], (*s.shape, 3)).copy()

    def frame(self, s, t):
        n = self.normal(s, t)
        return np.broadcast_to(self.e_s, n.shape).copy(), np.broadcast_to(self.e_t, n.shape).copy(), n

    def fit(self, radius, extent=None):
        half = min(self.size[:2]) / 2
        if radius > half:
            raise ValueError(f"the marker reaches {radius * 1000:.0f} mm from its centre; "
                             f"the tile allows {half * 1000:.0f} mm")
        return {"max_length_error": 0.0}

    def height_and_normal(self, points):
        points = np.atleast_2d(points)
        normal = np.tile([0.0, 0.0, 1.0], (len(points), 1))
        return points[:, 2] - self.origin[2], normal

    def mesh(self):
        box = trimesh.creation.box(self.size)
        box.apply_translation(self.position + [0, 0, self.size[2] / 2])
        return box


class Cylinder(Surface):
    """A can, standing up or lying on its side; the marker wraps around it.

    ``lying: true`` lays the can on the plate with its axis horizontal along
    ``axis_azimuth_deg`` (0 = +x, away from the robot), so the marker on top is
    drawn from above. Standing, the marker is on the side.
    """

    kind = "cylinder"

    def __init__(self, spec):
        super().__init__(spec)
        self.R, self.height = float(spec["radius_m"]), float(spec["height_m"])
        self.lying = bool(spec.get("lying", False))
        marker = spec.get("marker", {})
        if self.lying:
            heading = np.radians(spec.get("axis_azimuth_deg", 0.0))
            # t runs along the axis toward the robot (the marker's "up", as on
            # the tile); azimuth 0 is the top of the can.
            self.axis = -np.array([np.cos(heading), np.sin(heading), 0.0])
            self.ref = np.array([0.0, 0.0, 1.0])
            self.centre = self.position + [0.0, 0.0, self.R]
            self.t0 = float(marker.get("along_m", 0.0))
            self.phi0 = np.radians(marker.get("azimuth_deg", 0.0))
        else:
            self.axis = np.array([0.0, 0.0, 1.0])
            self.ref = np.array([1.0, 0.0, 0.0])
            self.centre = self.position + [0.0, 0.0, self.height / 2]
            self.t0 = float(marker.get("height_m", self.height / 2)) - self.height / 2
            self.phi0 = np.radians(marker.get("azimuth_deg", -90.0))
        self.side = np.cross(self.axis, self.ref)  # (axis, ref, side) is right-handed.

    def _theta(self, s):
        # Increasing s turns to the right of someone facing the marker.
        return self.phi0 + np.asarray(s, float) / self.R

    def normal(self, s, t):
        theta = self._theta(s)[..., None]
        return np.cos(theta) * self.ref + np.sin(theta) * self.side

    def point(self, s, t, d=0.0):
        s, t, d = np.broadcast_arrays(np.asarray(s, float), np.asarray(t, float), d)
        along = self.centre + (self.t0 + t)[..., None] * self.axis
        return along + (self.R + d)[..., None] * self.normal(s, t)

    def frame(self, s, t):
        theta = self._theta(s)[..., None]
        e_s = -np.sin(theta) * self.ref + np.cos(theta) * self.side
        e_t = np.broadcast_to(self.axis, e_s.shape).copy()
        return e_s, e_t, self.normal(s, t)

    def offset_scale(self, d):
        return (self.R + d) / self.R

    def curvature_radius(self):
        return self.R

    def fit(self, radius, extent=None):
        s_reach = radius if extent is None else extent[0]
        t_low, t_high = (-radius, radius) if extent is None else extent[1:]
        if s_reach / self.R > CYLINDER_MAX_ANGLE:
            raise ValueError(
                f"the marker wraps {np.degrees(s_reach / self.R):.0f} deg either side of its "
                f"centre on a {self.R * 2000:.0f} mm can; at most "
                f"{np.degrees(CYLINDER_MAX_ANGLE):.0f} deg (a marker at most "
                f"{2 * CYLINDER_MAX_ANGLE * self.R * 1000:.0f} mm wide)")
        half = self.height / 2 - 0.005
        if self.t0 + t_low < -half or self.t0 + t_high > half:
            raise ValueError("the marker runs off the end of the can")
        return {"max_length_error": 0.0, "wrap_deg": float(np.degrees(2 * s_reach / self.R))}

    def height_and_normal(self, points):
        radial = np.atleast_2d(points) - self.centre
        radial -= (radial @ self.axis)[:, None] * self.axis
        distance = np.linalg.norm(radial, axis=1)
        return distance - self.R, radial / np.maximum(distance, 1e-12)[:, None]

    def mesh(self):
        can = trimesh.creation.cylinder(radius=self.R, height=self.height, sections=96)
        placement = np.eye(4)
        placement[:3, :3] = np.column_stack([self.ref, self.side, self.axis])
        placement[:3, 3] = self.centre
        can.apply_transform(placement)
        return can


class Sphere(Surface):
    """A ball resting on the plate; the marker sits on it like a sticker.

    ``projection: stereographic`` (the default) bends an ordinary picture onto
    the ball with little stretch, for markers up to about 34 degrees across.
    ``projection: azimuthal_equidistant`` is for a picture that already is a
    map of the ball drawn that way (a polar globe map): a point's distance
    from the marker centre is its distance over the ball, so a picture whose
    radius is a quarter of the ball's circumference covers a whole hemisphere.
    """

    kind = "sphere"

    def __init__(self, spec):
        super().__init__(spec)
        self.R = float(spec["radius_m"])
        marker = spec.get("marker", {})
        azimuth = np.radians(marker.get("azimuth_deg", -90.0))
        elevation = np.radians(marker.get("elevation_deg", 30.0))
        self.centre = self.position + [0, 0, self.R]
        self.d0 = np.array([np.cos(elevation) * np.cos(azimuth),
                            np.cos(elevation) * np.sin(azimuth), np.sin(elevation)])
        up = np.array([0.0, 0.0, 1.0]) - self.d0[2] * self.d0
        if np.linalg.norm(up) < 1e-6:  # At the top: "up" points toward the robot, as on the tile.
            up = np.array([-np.cos(azimuth), -np.sin(azimuth), 0.0])
        self.e2 = _unit(up)               # marker "up", toward the top of the ball
        self.e1 = np.cross(self.e2, self.d0)  # marker "right", seen from outside
        self.k = 1.0                      # stereographic scale, set by fit()
        self.projection = spec.get("projection", "stereographic")
        if self.projection not in ("stereographic", "azimuthal_equidistant"):
            raise ValueError("sphere projection must be stereographic or azimuthal_equidistant")

    def _angles(self, s, t):
        s, t = np.asarray(s, float), np.asarray(t, float)
        rho = np.hypot(s, t)
        if self.projection == "azimuthal_equidistant":
            return rho / self.R, np.arctan2(t, s)
        alpha = 2 * np.arctan(rho / (2 * self.R * self.k))
        beta = np.arctan2(t, s)
        return alpha, beta

    def normal(self, s, t):
        alpha, beta = self._angles(s, t)
        radial = np.cos(beta)[..., None] * self.e1 + np.sin(beta)[..., None] * self.e2
        return np.cos(alpha)[..., None] * self.d0 + np.sin(alpha)[..., None] * radial

    def point(self, s, t, d=0.0):
        n = self.normal(s, t)
        d = np.broadcast_to(np.asarray(d, float), n.shape[:-1])
        return self.centre + (self.R + d)[..., None] * n

    def frame(self, s, t):
        """Tangents follow the images of the s and t directions (orthogonal: conformal)."""
        n = self.normal(s, t)
        eps = 1e-6
        ds = self.point(np.asarray(s) + eps, t) - self.point(np.asarray(s) - eps, t)
        e_s = _unit(ds - np.sum(ds * n, -1, keepdims=True) * n)
        return e_s, np.cross(n, e_s), n

    def offset_scale(self, d):
        return (self.R + d) / self.R

    def curvature_radius(self):
        return self.R

    def scale(self, s, t):
        """Length on the ball per length in (s, t): the same in every direction for the
        stereographic map; across the radius (the smaller) for the equidistant one."""
        alpha, _ = self._angles(s, t)
        if self.projection == "azimuthal_equidistant":
            return np.sinc(alpha / np.pi)  # sin(alpha) / alpha
        return np.cos(alpha / 2) ** 2 / self.k

    def fit(self, radius, extent=None):
        """Scale so the stretch at the centre equals the shrink at ``radius``."""
        if self.projection == "azimuthal_equidistant":
            alpha = radius / self.R
            if alpha > GLOBE_MAX_ANGLE:
                raise ValueError(
                    f"the map reaches {np.degrees(alpha):.0f} deg from the pole on a "
                    f"{self.R * 2000:.0f} mm ball; a hemisphere map is at most "
                    f"{np.pi * self.R * 1000:.0f} mm across (--size-mm)")
            return {"max_length_error": float(1 - np.sinc(alpha / np.pi)),
                    "angle_deg": float(np.degrees(alpha)),
                    "picture_is_a_map": True}
        self.k = 1.0
        for _ in range(30):  # The scale depends on the angle it produces.
            alpha = 2 * np.arctan(radius / (2 * self.R * self.k))
            self.k = (1 + np.cos(alpha / 2) ** 2) / 2
        alpha = 2 * np.arctan(radius / (2 * self.R * self.k))
        if alpha > SPHERE_MAX_ANGLE:
            edge = np.cos(SPHERE_MAX_ANGLE / 2) ** 2
            largest = 2 * self.R * np.tan(SPHERE_MAX_ANGLE / 2) * (1 + edge) / 2
            raise ValueError(
                f"the marker reaches {np.degrees(alpha):.0f} deg from its centre on a "
                f"{self.R * 2000:.0f} mm ball; at most {np.degrees(SPHERE_MAX_ANGLE):.0f} deg "
                f"(a marker reaching at most {largest * 1000:.0f} mm from its centre)")
        low, high = self.scale(radius, 0.0), self.scale(0.0, 0.0)
        return {"max_length_error": float(max(abs(low - 1), abs(high - 1))),
                "angle_deg": float(np.degrees(alpha))}

    def height_and_normal(self, points):
        radial = np.atleast_2d(points) - self.centre
        distance = np.linalg.norm(radial, axis=1)
        return distance - self.R, radial / distance[:, None]

    def mesh(self):
        ball = trimesh.creation.icosphere(subdivisions=4, radius=self.R)
        ball.apply_translation(self.centre)
        return ball


KINDS = {"plane": Plane, "cylinder": Cylinder, "sphere": Sphere}


def load_surface(spec_or_path):
    spec = spec_or_path
    if isinstance(spec_or_path, (str, Path)):
        spec = yaml.safe_load(Path(spec_or_path).read_text())
    kind = spec.get("type")
    if kind not in KINDS:
        raise ValueError(f"surface type must be one of {', '.join(KINDS)}, not {kind!r}")
    body = {key: value for key, value in spec.items() if key != "type"}
    return KINDS[kind](body)
