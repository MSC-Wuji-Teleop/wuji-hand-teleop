"""Full-mesh self-collision against the URDF's own collision geometry.

What the geometry backend can and cannot answer. These are triangle meshes,
and coal will not sign-distance a mesh: `enable_signed_distance` is a
convex-shape feature, `computeDistance` returns exactly 0.0 the moment two
meshes touch however deep they then go, and the contact's own
`penetration_depth` saturates at the security margin. So there is no depth of
overlap to be had, and the tool does not pretend otherwise. What is available
is exactly what the decision needs, from three queries with very different
costs (measured on this model, on a real clip frame):

    sweep, margin = clearance      170-220 ms   all 3576 pairs at once, gives
                                                the 37-92 pairs worth looking
                                                at on a frame
    computeDistance, per candidate  16-26 ms    the true gap, 0 at touch
    computeCollision, margin = 0     4-9 ms     touching or not, per candidate

So a contact is either a NEAR MISS with a measured gap, or TOUCHING with no
number attached -- and touching is the defect ("wrists phase through each
other"), so a number for how far through is not needed to act.

Pair set. Every unordered geometry pair is a candidate, then two exclusions:

    tree-adjacent   the two links are the same joint, or within two joints of
                    each other in the kinematic tree. Their meshes touch by
                    construction at every configuration -- a proximal segment
                    and its own distal segment always overlap at the knuckle
                    -- so checking them reports geometry, not motion. 247
                    pairs on this model.
    both static     neither link can move: the clip drives the arms and the
                    hands, and legs and waist are locked. Such a pair gives
                    the same answer at every frame, so it is evaluated once at
                    build time and reported there. 182 pairs.

3576 pairs are left, and the model is collision-free at its neutral pose,
which is the check that validates the exclusion set.

Classification. Whether a contact can be fixed at all depends on which chains
it is between, so every pair carries a class:

    cross_side   left arm or hand against right arm or hand. The spec's first
                 defect. Fixable: either arm can move.
    arm_body     an arm or hand against the torso, head, pelvis or a leg.
                 Fixable: the arm can move.
    arm_self     one side's hand against its own forearm or upper arm.
                 Fixable: that arm's posture decides it.
    intra_hand   two segments of the same hand. NOT fixable here: hand joint
                 angles pass through this tool untouched, so nothing it can
                 change moves either geometry. Reported, and only reported.

That last class is not a corner case. A real bundle clip has intra-hand
finger contact on every single frame -- middle-finger and ring-finger distal
segments, mostly, from the retargeter closing the fingers -- so folding it in
with the rest would bury the defects the tool exists to find.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from clip_audit import SIDES

from .model import SanitizeModel

# Constants. Each one says where its value comes from.

CLASS_CROSS_SIDE = "cross_side"
CLASS_ARM_BODY = "arm_body"
CLASS_ARM_SELF = "arm_self"
CLASS_INTRA_HAND = "intra_hand"

# What an arm-joint change can affect; never intra_hand, whose joints are fixed.
FIXABLE_CLASSES = (CLASS_CROSS_SIDE, CLASS_ARM_BODY, CLASS_ARM_SELF)
ALL_CLASSES = FIXABLE_CLASSES + (CLASS_INTRA_HAND,)

# Not zero: the arm tracks with error, and 5 mm is what these STLs resolve.
DEFAULT_CLEARANCE_M = 0.005

# Aim past the clearance, so a frame does not land exactly on the threshold.
REPAIR_OVERSHOOT_M = 0.001

# Tree distance below which a pair is dropped; depth 2 is collision-free at rest.
ADJACENCY_DEPTH = 2

# Bound on a caller's own solve-and-recheck loop; cli.py's is much smaller.
DEFAULT_REPAIR_ITERS = 8

# Geometry names carry the link name plus this suffix, per link.
GEOM_NAME_SUFFIX = "_0"

# A hand geometry is identified by link name, not tree: fixed joints hide it.
HAND_LINK_PREFIX = "{side}_wuji_"

# When to trust coal's normal sense: it flipped +0.99 to -0.02 between frames.
NORMAL_SENSE_MIN_COSINE = 0.1


def link_name(geom_name: str) -> str:
    """'left_elbow_link_0' -> 'left_elbow_link'."""
    return geom_name[:-len(GEOM_NAME_SUFFIX)] if geom_name.endswith(GEOM_NAME_SUFFIX) else geom_name


@dataclass(frozen=True)
class Pair:
    """One checked geometry pair."""
    index: int              # index into geom_model.collisionPairs
    first: int              # geometry index
    second: int
    name_a: str
    name_b: str
    klass: str
    baseline: bool          # already inside the clearance at the neutral pose

    @property
    def fixable(self) -> bool:
        return self.klass in FIXABLE_CLASSES

    def as_dict(self) -> dict:
        return {"a": self.name_a, "b": self.name_b, "class": self.klass,
                "baseline": self.baseline}


@dataclass
class Contact:
    """One pair, too close at one frame.

    touching means the meshes intersect -- the "phasing through" defect, with
    no depth to report. Otherwise gap_m is the measured distance.
    """
    pair: Pair
    gap_m: float
    touching: bool
    normal: np.ndarray      # unit, points from the first geometry to the second
    point_a: np.ndarray     # where on the first geometry, world frame
    point_b: np.ndarray
    joint_a: int
    joint_b: int

    def describe(self) -> str:
        return "touching" if self.touching else f"gap {1e3 * self.gap_m:+.2f} mm"

    def as_dict(self) -> dict:
        return {"a": self.pair.name_a, "b": self.pair.name_b,
                "class": self.pair.klass, "touching": self.touching,
                "gap_mm": None if self.touching else round(1e3 * self.gap_m, 3)}


@dataclass
class SeparationRow:
    """A linearised "get this pair further apart" constraint.

    The gradient of the pair's gap over the 14 arm joints: the witness points'
    relative velocity along the separating normal. Locked links give zeros.
    """
    contact: Contact
    required_gain_m: float

    def jacobian_row(self, solver) -> np.ndarray:
        Ja = solver.point_jacobian(self.contact.joint_a, self.contact.point_a)
        Jb = solver.point_jacobian(self.contact.joint_b, self.contact.point_b)
        return self.contact.normal @ (Jb - Ja)


class CollisionChecker:
    """The pair set and the three queries, built once per run."""

    def __init__(self, sm: SanitizeModel,
                 clearance_m: float = DEFAULT_CLEARANCE_M,
                 adjacency_depth: int = ADJACENCY_DEPTH) -> None:
        if sm.geom_model is None:
            raise RuntimeError("SanitizeModel was built without geometry")
        import coal

        self.sm = sm
        self.pin = sm.pin
        self.clearance_m = float(clearance_m)
        self.adjacency_depth = int(adjacency_depth)

        self.excluded_adjacent = 0
        self.excluded_static = 0
        self.pairs: List[Pair] = []
        self._build_pairs()

        self.geom_data = sm.geom_model.createData()
        for request in self.geom_data.collisionRequests:
            # security_margin makes isCollision() mean "closer than this".
            request.security_margin = self.clearance_m
            request.enable_contact = True
            request.num_max_contacts = 1

        # The authoritative "do these intersect", kept apart from the sweep's margin.
        self.touch_request = coal.CollisionRequest()
        self.touch_request.security_margin = 0.0
        self.touch_request.enable_contact = True
        self.touch_request.num_max_contacts = 1

        self._mark_baseline()

    # -- pair set ----------------------------------------------------------

    def _chain_of(self, geom) -> str:
        ancestors = set(self.sm.joint_ancestors(geom.parentJoint))
        for side in SIDES:
            side_joints = set(self.sm.arm_joint_ids[side]) | set(self.sm.hand_joint_ids[side])
            if ancestors & side_joints:
                return side
        return "body"

    def _is_hand(self, geom) -> bool:
        return any(geom.name.startswith(HAND_LINK_PREFIX.format(side=s)) for s in SIDES)

    def _classify(self, geom_a, geom_b) -> str:
        chain_a, chain_b = self._chain_of(geom_a), self._chain_of(geom_b)
        if chain_a == "body" or chain_b == "body":
            return CLASS_ARM_BODY
        if chain_a != chain_b:
            return CLASS_CROSS_SIDE
        if self._is_hand(geom_a) and self._is_hand(geom_b):
            return CLASS_INTRA_HAND
        return CLASS_ARM_SELF

    def _build_pairs(self) -> None:
        gm = self.sm.geom_model
        objects = gm.geometryObjects
        # The pair list lives on the shared geometry model, so start from empty.
        gm.removeAllCollisionPairs()
        for a, b in self.sm.geometry_pairs():
            ga, gb = objects[a], objects[b]
            if ga.parentJoint == gb.parentJoint or self.sm.joints_within(
                    ga.parentJoint, gb.parentJoint, self.adjacency_depth):
                self.excluded_adjacent += 1
                continue
            if not (self.sm.geometry_is_movable(ga) or self.sm.geometry_is_movable(gb)):
                self.excluded_static += 1
                continue
            index = len(gm.collisionPairs)
            gm.addCollisionPair(self.pin.CollisionPair(a, b))
            self.pairs.append(Pair(index=index, first=a, second=b,
                                   name_a=link_name(ga.name), name_b=link_name(gb.name),
                                   klass=self._classify(ga, gb), baseline=False))

    def _mark_baseline(self) -> None:
        """Pairs already inside the clearance with the robot at rest.

        A pair too close at the neutral pose is telling us about the STL
        geometry, not the clip, so it is reported only once it touches.
        """
        flagged = {c.pair.index for c in self.check(self.sm.neutral(), classes=ALL_CLASSES)}
        self.pairs = [Pair(index=p.index, first=p.first, second=p.second,
                           name_a=p.name_a, name_b=p.name_b, klass=p.klass,
                           baseline=p.index in flagged)
                      for p in self.pairs]

    def verify_model_at_rest(self) -> List[Contact]:
        """Pairs that actually touch at the neutral pose.

        Should be empty. A non-empty result means the exclusion set no longer
        matches the model -- a regenerated URDF, or a changed adjacency depth
        -- and the caller refuses the run rather than reporting every frame.
        """
        return [c for c in self.check(self.sm.neutral(), classes=ALL_CLASSES) if c.touching]

    # -- queries -----------------------------------------------------------

    @property
    def pair_count(self) -> int:
        return len(self.pairs)

    def counts_by_class(self) -> Dict[str, int]:
        return {k: sum(1 for p in self.pairs if p.klass == k) for k in ALL_CLASSES}

    def _separating_normal(self, pair: Pair, p_a: np.ndarray, p_b: np.ndarray,
                           touching: bool) -> Optional[np.ndarray]:
        """A unit vector that, followed by the second geometry, separates them.

        Away from contact the witness points span the gap. At contact they
        coincide, so coal's normal is used with its sense taken from the
        placements -- see NORMAL_SENSE_MIN_COSINE for why.
        """
        if not touching:
            span = p_b - p_a
            length = float(np.linalg.norm(span))
            if length > 1e-9:
                return span / length

        placements = self.geom_data.oMg
        centre = (placements[pair.second].translation - placements[pair.first].translation)
        centre_norm = float(np.linalg.norm(centre))
        if centre_norm <= 1e-9:
            return None
        centre = centre / centre_norm

        result = self.geom_data.collisionResults[pair.index]
        if result.numContacts():
            normal = np.asarray(result.getContact(0).normal, dtype=float)
            norm = float(np.linalg.norm(normal))
            if norm > 1e-9:
                normal = normal / norm
                projection = float(normal @ centre)
                if abs(projection) >= NORMAL_SENSE_MIN_COSINE:
                    return normal if projection > 0.0 else -normal
        return centre

    def _contact_of(self, pair: Pair) -> Optional[Contact]:
        """The gap, the witness points and the separating normal for one pair.

        Assumes the geometry placements are current and the pair has already
        come back positive from the sweep (or from computeCollision).
        """
        touching = bool(self.pin.computeCollision(
            self.sm.geom_model, self.geom_data, pair.index, self.touch_request))
        self.pin.computeDistance(self.sm.geom_model, self.geom_data, pair.index)
        distance = self.geom_data.distanceResults[pair.index]
        gap = float(distance.min_distance)
        p_a = np.asarray(distance.getNearestPoint1(), dtype=float)
        p_b = np.asarray(distance.getNearestPoint2(), dtype=float)

        if touching:
            # Inside an overlap the contact point is the only known meeting place.
            result = self.geom_data.collisionResults[pair.index]
            if result.numContacts():
                p_a = p_b = np.asarray(result.getContact(0).pos, dtype=float)
            gap = 0.0

        normal = self._separating_normal(pair, p_a, p_b, touching)
        if normal is None:
            return None

        objects = self.sm.geom_model.geometryObjects
        return Contact(pair=pair, gap_m=gap, touching=touching, normal=normal,
                       point_a=p_a, point_b=p_b,
                       joint_a=objects[pair.first].parentJoint,
                       joint_b=objects[pair.second].parentJoint)

    def _counts(self, pair: Pair, contact: Contact) -> bool:
        """Whether this contact is reported at all.

        A pair that is already inside the clearance with the robot at rest
        only counts once it actually touches -- that is the STL geometry
        talking, not the clip.
        """
        return contact.touching or not pair.baseline

    def check(self, q: np.ndarray,
              classes: Sequence[str] = FIXABLE_CLASSES) -> List[Contact]:
        """Every pair of the given classes inside the clearance."""
        wanted = set(classes)
        self.pin.computeCollisions(self.sm.model, self.sm.data,
                                   self.sm.geom_model, self.geom_data, q, False)
        self.pin.updateGeometryPlacements(self.sm.model, self.sm.data,
                                          self.sm.geom_model, self.geom_data, q)
        out: List[Contact] = []
        for pair in self.pairs:
            if pair.klass not in wanted:
                continue
            if not self.geom_data.collisionResults[pair.index].isCollision():
                continue
            contact = self._contact_of(pair)
            if contact is not None and self._counts(pair, contact):
                out.append(contact)
        return out

    def recheck(self, q: np.ndarray, pairs: Sequence[Pair]) -> List[Contact]:
        """Re-measure a known subset, without sweeping the whole pair set.

        The constrained solve calls this every iteration: only the arms moved,
        so a per-pair query beats another 200 ms sweep.
        """
        self.pin.updateGeometryPlacements(self.sm.model, self.sm.data,
                                          self.sm.geom_model, self.geom_data, q)
        out: List[Contact] = []
        for pair in pairs:
            if not self.pin.computeCollision(self.sm.geom_model, self.geom_data, pair.index):
                continue
            contact = self._contact_of(pair)
            if contact is not None and self._counts(pair, contact):
                out.append(contact)
        return out

    def separation_rows(self, contacts: Sequence[Contact]) -> List[SeparationRow]:
        """Turn contacts into constraints, aiming just past the clearance.

        A touching pair has no measured gap, so it is asked for the whole
        clearance; a near miss is asked only for what it is short by.
        """
        rows: List[SeparationRow] = []
        for contact in contacts:
            if not contact.pair.fixable:
                continue
            target = (0.0 if contact.pair.baseline else self.clearance_m) + REPAIR_OVERSHOOT_M
            gain = target - contact.gap_m
            if gain <= 0.0:
                continue
            rows.append(SeparationRow(contact=contact, required_gain_m=gain))
        return rows

    def as_dict(self) -> dict:
        return {"pairs_checked": self.pair_count,
                "pairs_by_class": self.counts_by_class(),
                "excluded": {"tree_adjacent": self.excluded_adjacent,
                             "both_static": self.excluded_static},
                "adjacency_depth": self.adjacency_depth,
                "clearance_mm": round(1e3 * self.clearance_m, 3),
                "baseline_pairs": [p.as_dict() for p in self.pairs if p.baseline]}
