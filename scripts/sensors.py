import numpy as np
import seahowl


class Sensor:
    def __init__(self):
        self.sensor_function = None

    def __call__(self) -> np.ndarray:
        return self.sensor_function()


class SensorFEA(Sensor):
    def __init__(self, component: seahowl.elasto.ComponentElastoFEA):
        super().__init__()
        if not isinstance(component, seahowl.elasto.ComponentElastoFEA):
            raise TypeError(
                f"Trying to add an FEA sensor to a non-FEA component: {type(component)}."
            )


class AccelerometerFEA(SensorFEA):
    def __init__(
        self,
        component: seahowl.elasto.ComponentElastoFEA,
        fraction: float,
        is_local: bool = True,
    ) -> None:
        super().__init__(component)
        node1, weight1, node2, weight2 = get_nodes_and_weights(component, fraction)

        def get_acceleration_at_fraction():
            node1_acc = node1.get_acceleration()
            node2_acc = node2.get_acceleration()
            if is_local:  # transform to local frame
                node1_acc = (node1.get_rotation_matrix().T).dot(node1_acc)
                node2_acc = (node2.get_rotation_matrix().T).dot(node2_acc)
            return weight1 * node1_acc + weight2 * node2_acc

        self.sensor_function = get_acceleration_at_fraction


class AccelerometerRotationalFEA(SensorFEA):
    def __init__(
        self,
        component: seahowl.elasto.ComponentElastoFEA,
        fraction: float,
        is_local: bool = True,
    ) -> None:
        super().__init__(component)
        node1, weight1, node2, weight2 = get_nodes_and_weights(component, fraction)
        self.sensor_function = lambda: weight1 * node1.get_rotational_acceleration(
            is_local
        ) + weight2 * node2.get_rotational_acceleration(is_local)


class MomentGaugeFEA(SensorFEA):
    def __init__(
        self, component: seahowl.elasto.ComponentElastoFEA, fraction: float
    ) -> None:
        super().__init__(component)
        element, eta_element = get_element_and_eta(component, fraction)
        self.sensor_function = lambda: element.get_torque(eta_element)


class ForceGaugeFEA(SensorFEA):
    def __init__(
        self, component: seahowl.elasto.ComponentElastoFEA, fraction: float
    ) -> None:
        super().__init__(component)
        element, eta_element = get_element_and_eta(component, fraction)
        self.sensor_function = lambda: element.get_force(eta_element)


class PositionGaugeFEA(SensorFEA):
    def __init__(
        self, component: seahowl.elasto.ComponentElastoFEA, fraction: float
    ) -> None:
        super().__init__(component)
        element, eta_element = get_element_and_eta(component, fraction)
        self.sensor_function = lambda: element.get_position(eta_element)


class SensorLink(Sensor):
    """Base sensor for ``seahowl.elasto.Link`` components.

    Unlike ``*GaugeFEA`` sensors, links are not ``ComponentElastoFEA``: they
    do not have ``discretized_points``/``elements`` so no fraction is needed.
    """

    def __init__(self, component: seahowl.elasto.Link) -> None:
        super().__init__()
        if not isinstance(component, seahowl.elasto.Link):
            raise TypeError(
                f"Trying to add a Link sensor to a non-Link component: {type(component)}."
            )


class ReactionForceGaugeFEA(SensorLink):
    """Sensor measuring the reaction force of a ``seahowl.elasto.Link``."""

    def __init__(self, component: seahowl.elasto.Link) -> None:
        super().__init__(component)
        self.sensor_function = component.get_reaction_force


class ReactionTorqueGaugeFEA(SensorLink):
    """Sensor measuring the reaction torque of a ``seahowl.elasto.Link``."""

    def __init__(self, component: seahowl.elasto.Link) -> None:
        super().__init__(component)
        self.sensor_function = component.get_reaction_torque


def get_element_and_eta(
    component: seahowl.elasto.ComponentElastoFEA, fraction: float
) -> tuple[seahowl.elasto.ElementElasto, float]:
    assert 0 <= fraction <= 1, "fraction must be between 0 and 1"
    for idx, point in enumerate(component.discretized_points):
        if point.fraction >= fraction:
            if idx == 0:  # take the next node if index 0
                idx = 1
            element_eta = (
                (fraction - component.discretized_points[idx - 1].fraction)
                / (
                    component.discretized_points[idx].fraction
                    - component.discretized_points[idx - 1].fraction
                )
            ) * 2 - 1
            return component.elements[idx - 1], element_eta


def get_nodes_and_weights(
    component: seahowl.elasto.ComponentElastoFEA, fraction: float
) -> tuple[seahowl.elasto.NodeElasto, float, seahowl.elasto.NodeElasto, float]:
    assert 0 <= fraction <= 1, "fraction must be between 0 and 1"
    for idx, point in enumerate(component.discretized_points):
        if point.fraction >= fraction:
            if idx == 0:  # take the next node if index 0
                idx = 1
            weight = (fraction - component.discretized_points[idx - 1].fraction) / (
                component.discretized_points[idx].fraction
                - component.discretized_points[idx - 1].fraction
            )
            return (component.nodes[idx - 1], 1 - weight, component.nodes[idx], weight)
