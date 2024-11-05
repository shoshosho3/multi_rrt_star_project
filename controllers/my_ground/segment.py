import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Result:
    """Result structure for distance computation."""
    distance: float = 0.0
    sqr_distance: float = 0.0
    parameter: List[float] = None
    closest: List[np.ndarray] = None

    def __post_init__(self):
        if self.parameter is None:
            self.parameter = [0.0, 0.0]
        if self.closest is None:
            self.closest = [np.zeros(3), np.zeros(3)]


class SegmentDistance:
    """Compute the closest points for two segments in N dimensions."""

    @staticmethod
    def get_clamped_root(slope: float, h0: float, h1: float) -> float:
        """
        Compute the root of h(z) = h0 + slope*z and clamp it to [0,1].
        """
        if h0 < 0:
            if h1 > 0:
                r = -h0 / slope
                if r > 1:
                    r = 0.5
            else:
                r = 1.0
        else:
            r = 0.0
        return r

    @staticmethod
    def compute_intersection(s_value: List[float], classify: List[int], b: float,
                             f00: float, f10: float) -> Tuple[List[int], List[List[float]]]:
        """
        Compute intersection of the line dR/ds = 0 with domain [0,1]^2.
        """
        edge = [0, 0]
        end = [[0.0, 0.0], [0.0, 0.0]]

        if classify[0] < 0:
            edge[0] = 0
            end[0][0] = 0.0
            end[0][1] = f00 / b if b != 0 else 0.5
            if end[0][1] < 0 or end[0][1] > 1:
                end[0][1] = 0.5

            if classify[1] == 0:
                edge[1] = 3
                end[1][0] = s_value[1]
                end[1][1] = 1.0
            else:  # classify[1] > 0
                edge[1] = 1
                end[1][0] = 1.0
                end[1][1] = f10 / b if b != 0 else 0.5
                if end[1][1] < 0 or end[1][1] > 1:
                    end[1][1] = 0.5
        elif classify[0] == 0:
            edge[0] = 2
            end[0][0] = s_value[0]
            end[0][1] = 0.0

            if classify[1] < 0:
                edge[1] = 0
                end[1][0] = 0.0
                end[1][1] = f00 / b if b != 0 else 0.5
                if end[1][1] < 0 or end[1][1] > 1:
                    end[1][1] = 0.5
            elif classify[1] == 0:
                edge[1] = 3
                end[1][0] = s_value[1]
                end[1][1] = 1.0
            else:
                edge[1] = 1
                end[1][0] = 1.0
                end[1][1] = f10 / b if b != 0 else 0.5
                if end[1][1] < 0 or end[1][1] > 1:
                    end[1][1] = 0.5
        else:  # classify[0] > 0
            edge[0] = 1
            end[0][0] = 1.0
            end[0][1] = f10 / b if b != 0 else 0.5
            if end[0][1] < 0 or end[0][1] > 1:
                end[0][1] = 0.5

            if classify[1] == 0:
                edge[1] = 3
                end[1][0] = s_value[1]
                end[1][1] = 1.0
            else:
                edge[1] = 0
                end[1][0] = 0.0
                end[1][1] = f00 / b if b != 0 else 0.5
                if end[1][1] < 0 or end[1][1] > 1:
                    end[1][1] = 0.5

        return edge, end

    @staticmethod
    def compute_minimum_parameters(edge: List[int], end: List[List[float]],
                                   b: float, c: float, e: float, g00: float,
                                   g10: float, g01: float, g11: float) -> List[float]:
        """
        Compute minimum parameters for the intersection segment.
        """
        parameter = [0.0, 0.0]
        delta = end[1][1] - end[0][1]
        h0 = delta * (-b * end[0][0] + c * end[0][1] - e)

        if h0 >= 0:
            if edge[0] == 0:
                parameter[0] = 0.0
                parameter[1] = SegmentDistance.get_clamped_root(c, g00, g01)
            elif edge[0] == 1:
                parameter[0] = 1.0
                parameter[1] = SegmentDistance.get_clamped_root(c, g10, g11)
            else:
                parameter[0] = end[0][0]
                parameter[1] = end[0][1]
        else:
            h1 = delta * (-b * end[1][0] + c * end[1][1] - e)
            if h1 <= 0:
                if edge[1] == 0:
                    parameter[0] = 0.0
                    parameter[1] = SegmentDistance.get_clamped_root(c, g00, g01)
                elif edge[1] == 1:
                    parameter[0] = 1.0
                    parameter[1] = SegmentDistance.get_clamped_root(c, g10, g11)
                else:
                    parameter[0] = end[1][0]
                    parameter[1] = end[1][1]
            else:  # h0 < 0 and h1 > 0
                z = min(max(h0 / (h0 - h1), 0.0), 1.0)
                omz = 1.0 - z
                parameter[0] = omz * end[0][0] + z * end[1][0]
                parameter[1] = omz * end[0][1] + z * end[1][1]

        return parameter

    @staticmethod
    def compute_distance(P0: np.ndarray, P1: np.ndarray, Q0: np.ndarray, Q1: np.ndarray) -> Result:
        """
        Compute the distance between two line segments.
        """
        P1mP0 = P1 - P0
        Q1mQ0 = Q1 - Q0
        P0mQ0 = P0 - Q0

        a = np.dot(P1mP0, P1mP0)
        b = np.dot(P1mP0, Q1mQ0)
        c = np.dot(Q1mQ0, Q1mQ0)
        d = np.dot(P1mP0, P0mQ0)
        e = np.dot(Q1mQ0, P0mQ0)

        result = Result()

        if a > 0 and c > 0:
            s_value = [
                SegmentDistance.get_clamped_root(a, d, d + a),
                SegmentDistance.get_clamped_root(a, d - b, d + a - b)
            ]

            classify = []
            for s in s_value:
                if s <= 0:
                    classify.append(-1)
                elif s >= 1:
                    classify.append(1)
                else:
                    classify.append(0)

            if classify[0] == -1 and classify[1] == -1:
                result.parameter[0] = 0.0
                result.parameter[1] = SegmentDistance.get_clamped_root(c, -e, -e + c)
            elif classify[0] == 1 and classify[1] == 1:
                result.parameter[0] = 1.0
                result.parameter[1] = SegmentDistance.get_clamped_root(c, -e - b, -e + c - b)
            else:
                edge, end = SegmentDistance.compute_intersection(s_value, classify, b, d, d + a)
                result.parameter = SegmentDistance.compute_minimum_parameters(
                    edge, end, b, c, e, -e, -e - b, -e + c, -e + c - b)
        else:
            if a > 0:
                result.parameter[0] = SegmentDistance.get_clamped_root(a, d, d + a)
                result.parameter[1] = 0.0
            elif c > 0:
                result.parameter[0] = 0.0
                result.parameter[1] = SegmentDistance.get_clamped_root(c, -e, -e + c)
            else:
                result.parameter[0] = 0.0
                result.parameter[1] = 0.0

        result.closest[0] = P0 + result.parameter[0] * P1mP0
        result.closest[1] = Q0 + result.parameter[1] * Q1mQ0
        diff = result.closest[0] - result.closest[1]
        result.sqr_distance = np.dot(diff, diff)
        result.distance = np.sqrt(result.sqr_distance)

        return result
