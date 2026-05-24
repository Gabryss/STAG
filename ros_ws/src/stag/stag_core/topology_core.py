from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

NODE_TYPE_ENDPOINT = 1
NODE_TYPE_JUNCTION = 2
NODE_TYPE_LOOP_ANCHOR = 3
NODE_TYPE_REGION_MEDOID = 10
NODE_TYPE_GRADIENT = 20

NODE_SOURCE_TOPOLOGY = 1
NODE_SOURCE_REGION = 2
NODE_SOURCE_GRADIENT = 3

NEIGHBOR_OFFSETS = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)


@dataclass(frozen=True)
class CostmapTopologyResult:
    distance_field: np.ndarray
    skeleton_mask: np.ndarray
    node_positions: np.ndarray
    node_degrees: np.ndarray
    node_types: np.ndarray
    edge_node_indices: np.ndarray
    edge_polylines: tuple[np.ndarray, ...]
    edge_lengths: np.ndarray
    edge_min_clearances: np.ndarray
    edge_mean_clearances: np.ndarray


@dataclass(frozen=True)
class AuxiliaryGraphNodes:
    region_positions: np.ndarray
    region_classes: np.ndarray
    gradient_positions: np.ndarray
    gradient_strengths: np.ndarray


@dataclass
class _EdgeRecord:
    start_node: int
    end_node: int
    polyline: np.ndarray
    length: float
    min_clearance: float
    mean_clearance: float
    active: bool = True


def occupancy_grid_to_open_mask(
    data: np.ndarray,
    width: int,
    height: int,
    free_threshold: int,
    unknown_is_obstacle: bool,
) -> np.ndarray:
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    values = np.asarray(data, dtype=np.int16)
    if values.size != width * height:
        raise ValueError("occupancy data length must equal width * height")
    values = values.reshape((height, width))
    if unknown_is_obstacle:
        return (values >= 0) & (values <= int(free_threshold))
    return (values < 0) | (values <= int(free_threshold))


def occupancy_grid_to_traversability(
    data: np.ndarray,
    width: int,
    height: int,
    open_mask: np.ndarray,
) -> np.ndarray:
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    values = np.asarray(data, dtype=np.float32)
    if values.size != width * height:
        raise ValueError("occupancy data length must equal width * height")
    open_mask = np.asarray(open_mask, dtype=bool)
    if open_mask.shape != (height, width):
        raise ValueError("open_mask shape must match (height, width)")
    values = values.reshape((height, width))
    traversability = np.zeros((height, width), dtype=float)
    known_values = np.clip(values, 0.0, 100.0)
    traversability[open_mask] = 1.0 - known_values[open_mask] / 100.0
    return traversability


def flatten_polylines(polylines: tuple[np.ndarray, ...] | list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    if not polylines:
        return np.empty((0, 2), dtype=np.float32), np.asarray([0], dtype=np.uint32)
    offsets = [0]
    chunks: list[np.ndarray] = []
    total = 0
    for polyline in polylines:
        chunks.append(np.asarray(polyline, dtype=np.float32))
        total += len(polyline)
        offsets.append(total)
    return np.vstack(chunks), np.asarray(offsets, dtype=np.uint32)


def _blur9(values: np.ndarray, iters: int = 4) -> np.ndarray:
    blurred = np.asarray(values, dtype=float)
    for _ in range(iters):
        padded = np.pad(blurred, 1, mode="edge")
        blurred = (
            0.40 * padded[1:-1, 1:-1]
            + 0.10
            * (
                padded[:-2, 1:-1]
                + padded[2:, 1:-1]
                + padded[1:-1, :-2]
                + padded[1:-1, 2:]
            )
            + 0.05
            * (
                padded[:-2, :-2]
                + padded[:-2, 2:]
                + padded[2:, :-2]
                + padded[2:, 2:]
            )
        )
    return blurred


def _neighbors(y: int, x: int, mask: np.ndarray) -> list[tuple[int, int]]:
    h, w = mask.shape
    result: list[tuple[int, int]] = []
    for dy, dx in NEIGHBOR_OFFSETS:
        ny = y + dy
        nx = x + dx
        if ny < 0 or ny >= h or nx < 0 or nx >= w:
            continue
        if mask[ny, nx]:
            result.append((ny, nx))
    return result


def _connected_components(mask: np.ndarray) -> list[np.ndarray]:
    components: list[np.ndarray] = []
    if not np.any(mask):
        return components

    h, w = mask.shape
    seen = np.zeros(mask.shape, dtype=bool)
    for start_y, start_x in np.argwhere(mask):
        if seen[start_y, start_x]:
            continue
        queue: deque[tuple[int, int]] = deque([(int(start_y), int(start_x))])
        seen[start_y, start_x] = True
        coords: list[tuple[int, int]] = []
        while queue:
            y, x = queue.popleft()
            coords.append((y, x))
            for ny, nx in _neighbors(y, x, mask):
                if seen[ny, nx]:
                    continue
                seen[ny, nx] = True
                queue.append((ny, nx))
        components.append(np.asarray(coords, dtype=int))
    return components


def _extract_region_nodes(
    traversability: np.ndarray,
    open_mask: np.ndarray,
    quantiles: tuple[float, float, float, float] = (0.2, 0.4, 0.6, 0.8),
) -> tuple[np.ndarray, np.ndarray]:
    if not np.any(open_mask):
        return np.empty((0, 2), dtype=float), np.empty((0,), dtype=int)

    score = _blur9(traversability, iters=4)
    thresholds = np.quantile(score[open_mask], quantiles)
    labels = np.full(score.shape, -1, dtype=int)
    labels[open_mask] = np.digitize(score[open_mask], bins=thresholds)

    positions: list[tuple[float, float]] = []
    classes: list[int] = []
    for region_class in range(5):
        for coords in _connected_components(labels == region_class):
            if len(coords) == 0:
                continue
            centroid = coords.mean(axis=0)
            distances = np.sum((coords - centroid) ** 2, axis=1)
            medoid = coords[int(np.argmin(distances))]
            positions.append((float(medoid[1]), float(medoid[0])))
            classes.append(region_class)

    if not positions:
        return np.empty((0, 2), dtype=float), np.empty((0,), dtype=int)
    return np.asarray(positions, dtype=float), np.asarray(classes, dtype=int)


def _local_maxima_8n(values: np.ndarray) -> np.ndarray:
    padded = np.pad(values, 1, mode="constant", constant_values=-np.inf)
    center = padded[1:-1, 1:-1]
    maxima = np.isfinite(center)
    for dy in range(3):
        for dx in range(3):
            if dy == 1 and dx == 1:
                continue
            maxima &= center > padded[dy : dy + values.shape[0], dx : dx + values.shape[1]]
    maxima[[0, -1], :] = False
    maxima[:, [0, -1]] = False
    return maxima


def _pick_separated_points(
    values: np.ndarray,
    candidate_mask: np.ndarray,
    node_count: int,
    min_separation_cells: float,
) -> tuple[np.ndarray, np.ndarray]:
    ys, xs = np.where(candidate_mask)
    if len(xs) == 0 or node_count <= 0:
        return np.empty((0, 2), dtype=float), np.empty((0,), dtype=float)

    strengths = values[ys, xs]
    order = np.argsort(strengths)[::-1]
    selected: list[tuple[float, float]] = []
    selected_strengths: list[float] = []
    min_separation_sq = float(min_separation_cells) ** 2
    for idx in order:
        x = float(xs[idx])
        y = float(ys[idx])
        if any((sx - x) ** 2 + (sy - y) ** 2 < min_separation_sq for sx, sy in selected):
            continue
        selected.append((x, y))
        selected_strengths.append(float(strengths[idx]))
        if len(selected) >= node_count:
            break

    if not selected:
        return np.empty((0, 2), dtype=float), np.empty((0,), dtype=float)
    return np.asarray(selected, dtype=float), np.asarray(selected_strengths, dtype=float)


def _extract_gradient_nodes(
    traversability: np.ndarray,
    open_mask: np.ndarray,
    *,
    node_count: int = 48,
    node_quantile: float = 0.86,
    min_separation_cells: float = 10.0,
) -> tuple[np.ndarray, np.ndarray]:
    if not np.any(open_mask):
        return np.empty((0, 2), dtype=float), np.empty((0,), dtype=float)

    support = _blur9(open_mask.astype(float), iters=12) + 1e-9
    smoothed = _blur9(traversability * open_mask, iters=12) / support
    smoothed = _blur9(0.72 * traversability + 0.28 * smoothed, iters=3)
    d_t_dy, d_t_dx = np.gradient(smoothed, edge_order=2)
    magnitude = np.hypot(d_t_dx, d_t_dy)
    magnitude[~open_mask] = -np.inf

    threshold = float(np.quantile(magnitude[open_mask], node_quantile))
    candidate_mask = _local_maxima_8n(magnitude)
    candidate_mask &= open_mask
    candidate_mask &= magnitude >= threshold
    return _pick_separated_points(
        magnitude,
        candidate_mask,
        node_count=node_count,
        min_separation_cells=min_separation_cells,
    )


def extract_auxiliary_graph_nodes(
    traversability: np.ndarray,
    open_mask: np.ndarray,
    *,
    gradient_node_count: int = 48,
    gradient_node_quantile: float = 0.86,
    gradient_min_separation_cells: float = 10.0,
) -> AuxiliaryGraphNodes:
    traversability = np.asarray(traversability, dtype=float)
    open_mask = np.asarray(open_mask, dtype=bool)
    if traversability.shape != open_mask.shape:
        raise ValueError("traversability and open_mask must have the same shape")
    if gradient_node_count < 0:
        raise ValueError("gradient_node_count must be >= 0")
    if not 0.0 <= gradient_node_quantile <= 1.0:
        raise ValueError("gradient_node_quantile must be in [0, 1]")
    if gradient_min_separation_cells < 0.0:
        raise ValueError("gradient_min_separation_cells must be >= 0")
    region_positions, region_classes = _extract_region_nodes(traversability, open_mask)
    gradient_positions, gradient_strengths = _extract_gradient_nodes(
        traversability,
        open_mask,
        node_count=gradient_node_count,
        node_quantile=gradient_node_quantile,
        min_separation_cells=gradient_min_separation_cells,
    )
    return AuxiliaryGraphNodes(
        region_positions=region_positions,
        region_classes=region_classes,
        gradient_positions=gradient_positions,
        gradient_strengths=gradient_strengths,
    )


def keep_largest_open_component(open_mask: np.ndarray) -> np.ndarray:
    components = _connected_components(open_mask)
    if not components:
        return open_mask.astype(bool, copy=True)
    largest = max(components, key=len)
    filtered = np.zeros_like(open_mask, dtype=bool)
    filtered[largest[:, 0], largest[:, 1]] = True
    return filtered


def _representative_pixel(component: np.ndarray, distance_field: np.ndarray) -> tuple[int, int]:
    distances = distance_field[component[:, 0], component[:, 1]]
    best = component[int(np.argmax(distances))]
    return int(best[0]), int(best[1])


def _segment_key(a: tuple[int, int], b: tuple[int, int]) -> tuple[tuple[int, int], tuple[int, int]]:
    return (a, b) if a <= b else (b, a)


def _select_cycle_break_nodes(
    component: np.ndarray,
    distance_field: np.ndarray,
) -> tuple[tuple[int, int], tuple[int, int]]:
    if len(component) == 1:
        y, x = component[0]
        pixel = (int(y), int(x))
        return pixel, pixel

    distances = distance_field[component[:, 0], component[:, 1]]
    anchor_a = component[int(np.argmax(distances))]
    delta = component - anchor_a[None, :]
    anchor_b = component[int(np.argmax(np.sum(delta * delta, axis=1)))]
    a = (int(anchor_a[0]), int(anchor_a[1]))
    b = (int(anchor_b[0]), int(anchor_b[1]))
    if a == b and len(component) > 1:
        second_index = int(np.argsort(np.sum(delta * delta, axis=1))[-2])
        anchor_b = component[second_index]
        b = (int(anchor_b[0]), int(anchor_b[1]))
    return a, b


def _component_node_overrides(
    skeleton_mask: np.ndarray,
    degrees: np.ndarray,
    distance_field: np.ndarray,
) -> dict[tuple[int, int], int]:
    overrides: dict[tuple[int, int], int] = {}
    for component in _connected_components(skeleton_mask):
        component_structural = component[degrees[component[:, 0], component[:, 1]] != 2]
        if len(component_structural) > 0:
            continue
        anchor_a, anchor_b = _select_cycle_break_nodes(component, distance_field)
        overrides[anchor_a] = NODE_TYPE_LOOP_ANCHOR
        overrides[anchor_b] = NODE_TYPE_LOOP_ANCHOR
    return overrides


def _segment_has_clearance(
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    open_mask: np.ndarray,
    distance_field: np.ndarray,
    clearance_min_cells: float,
    samples_per_cell: float,
) -> bool:
    x0, y0 = float(start_xy[0]), float(start_xy[1])
    x1, y1 = float(end_xy[0]), float(end_xy[1])
    steps = max(2, int(np.ceil(np.hypot(x1 - x0, y1 - y0) * samples_per_cell)) + 1)
    ix = np.rint(np.linspace(x0, x1, steps)).astype(int)
    iy = np.rint(np.linspace(y0, y1, steps)).astype(int)

    h, w = open_mask.shape
    if np.any(ix < 0) or np.any(ix >= w) or np.any(iy < 0) or np.any(iy >= h):
        return False
    if not np.all(open_mask[iy, ix]):
        return False
    return bool(np.all(distance_field[iy, ix] >= clearance_min_cells))


def _polyline_has_clearance(
    polyline_xy: np.ndarray,
    open_mask: np.ndarray,
    distance_field: np.ndarray,
    clearance_min_cells: float,
    samples_per_cell: float,
) -> bool:
    if len(polyline_xy) < 2:
        return False
    return all(
        _segment_has_clearance(
            start_xy,
            end_xy,
            open_mask,
            distance_field,
            clearance_min_cells,
            samples_per_cell,
        )
        for start_xy, end_xy in zip(polyline_xy[:-1], polyline_xy[1:])
    )


def _simplify_chain(
    chain_xy: np.ndarray,
    open_mask: np.ndarray,
    distance_field: np.ndarray,
    clearance_min_cells: float,
    samples_per_cell: float,
) -> np.ndarray:
    if len(chain_xy) <= 2:
        return chain_xy.astype(float, copy=True)

    simplified = [chain_xy[0].astype(float)]
    index = 0
    while index < len(chain_xy) - 1:
        next_index = index + 1
        for candidate in range(len(chain_xy) - 1, index, -1):
            if _segment_has_clearance(
                chain_xy[index],
                chain_xy[candidate],
                open_mask,
                distance_field,
                clearance_min_cells,
                samples_per_cell,
            ):
                next_index = candidate
                break
        simplified.append(chain_xy[next_index].astype(float))
        index = next_index
    return np.asarray(simplified, dtype=float)


def _polyline_length(polyline_xy: np.ndarray) -> float:
    if len(polyline_xy) < 2:
        return 0.0
    deltas = np.diff(polyline_xy, axis=0)
    return float(np.sum(np.hypot(deltas[:, 0], deltas[:, 1])))


def _trace_edges(
    skeleton_mask: np.ndarray,
    structural_mask: np.ndarray,
) -> list[np.ndarray]:
    edges: list[np.ndarray] = []
    visited_segments: set[tuple[tuple[int, int], tuple[int, int]]] = set()

    for component in _connected_components(structural_mask):
        component_pixels = {(int(y), int(x)) for y, x in component}
        for start_pixel in sorted(component_pixels):
            for neighbor in _neighbors(start_pixel[0], start_pixel[1], skeleton_mask):
                if structural_mask[neighbor[0], neighbor[1]]:
                    continue
                segment = _segment_key(start_pixel, neighbor)
                if segment in visited_segments:
                    continue

                raw_chain = [start_pixel]
                previous = start_pixel
                current = neighbor
                visited_segments.add(segment)
                while True:
                    raw_chain.append(current)
                    if structural_mask[current[0], current[1]] and current not in component_pixels:
                        break

                    next_pixels = [
                        pixel
                        for pixel in _neighbors(current[0], current[1], skeleton_mask)
                        if pixel != previous
                    ]
                    if not next_pixels:
                        break

                    chosen_next = None
                    for pixel in next_pixels:
                        next_segment = _segment_key(current, pixel)
                        if next_segment not in visited_segments:
                            chosen_next = pixel
                            visited_segments.add(next_segment)
                            break
                    if chosen_next is None:
                        chosen_next = next_pixels[0]
                        visited_segments.add(_segment_key(current, chosen_next))

                    previous, current = current, chosen_next

                if len(raw_chain) >= 2:
                    edges.append(np.asarray(raw_chain, dtype=int))
    return edges


def _prune_leaf_edges(
    edge_records: list[_EdgeRecord],
    node_count: int,
    prune_leaf_length_cells: float,
    prune_leaf_to_radius_ratio: float,
) -> None:
    if prune_leaf_length_cells <= 0.0 and prune_leaf_to_radius_ratio <= 0.0:
        return

    while True:
        node_degrees = np.zeros(node_count, dtype=int)
        for edge in edge_records:
            if not edge.active:
                continue
            node_degrees[edge.start_node] += 1
            node_degrees[edge.end_node] += 1

        pruned_any = False
        for edge in edge_records:
            if not edge.active:
                continue
            is_leaf_edge = node_degrees[edge.start_node] == 1 or node_degrees[edge.end_node] == 1
            if not is_leaf_edge:
                continue
            threshold = max(
                float(prune_leaf_length_cells),
                float(prune_leaf_to_radius_ratio) * float(edge.mean_clearance),
            )
            if edge.length < threshold:
                edge.active = False
                pruned_any = True
                break
        if not pruned_any:
            break


def analyze_costmap_topology(
    open_mask: np.ndarray,
    *,
    clearance_min_cells: float = 1.0,
    simplify_samples_per_cell: float = 4.0,
    prune_leaf_length_cells: float = 8.0,
    prune_leaf_to_radius_ratio: float = 2.0,
    keep_largest_component: bool = True,
    seed: int = 0,
) -> CostmapTopologyResult:
    open_mask = np.asarray(open_mask, dtype=bool)
    if open_mask.ndim != 2:
        raise ValueError("open_mask must be a 2D array")
    if clearance_min_cells < 0.0:
        raise ValueError("clearance_min_cells must be >= 0")
    if simplify_samples_per_cell <= 0.0:
        raise ValueError("simplify_samples_per_cell must be > 0")
    if prune_leaf_length_cells < 0.0:
        raise ValueError("prune_leaf_length_cells must be >= 0")
    if prune_leaf_to_radius_ratio < 0.0:
        raise ValueError("prune_leaf_to_radius_ratio must be >= 0")
    if keep_largest_component:
        open_mask = keep_largest_open_component(open_mask)

    if not np.any(open_mask):
        return CostmapTopologyResult(
            distance_field=np.zeros(open_mask.shape, dtype=float),
            skeleton_mask=np.zeros(open_mask.shape, dtype=bool),
            node_positions=np.empty((0, 2), dtype=float),
            node_degrees=np.empty((0,), dtype=int),
            node_types=np.empty((0,), dtype=int),
            edge_node_indices=np.empty((0, 2), dtype=int),
            edge_polylines=(),
            edge_lengths=np.empty((0,), dtype=float),
            edge_min_clearances=np.empty((0,), dtype=float),
            edge_mean_clearances=np.empty((0,), dtype=float),
        )

    from scipy import ndimage as ndi
    from skimage.morphology import medial_axis

    distance_field = ndi.distance_transform_edt(open_mask)
    try:
        skeleton_mask = medial_axis(open_mask, rng=np.random.default_rng(seed))
    except TypeError:
        skeleton_mask = medial_axis(open_mask, random_state=int(seed))
    skeleton_mask &= open_mask

    degrees = np.zeros(skeleton_mask.shape, dtype=int)
    for y, x in np.argwhere(skeleton_mask):
        degrees[y, x] = len(_neighbors(int(y), int(x), skeleton_mask))

    node_type_overrides = _component_node_overrides(skeleton_mask, degrees, distance_field)
    structural_mask = skeleton_mask & (degrees != 2)
    for pixel in node_type_overrides:
        structural_mask[pixel[0], pixel[1]] = True

    node_positions: list[tuple[float, float]] = []
    node_types: list[int] = []
    node_index_by_structural_pixel: dict[tuple[int, int], int] = {}
    for component in _connected_components(structural_mask):
        representative = _representative_pixel(component, distance_field)
        node_positions.append((float(representative[1]), float(representative[0])))

        component_pixels = {(int(y), int(x)) for y, x in component}
        component_degrees = degrees[component[:, 0], component[:, 1]]
        if any(pixel in node_type_overrides for pixel in component_pixels):
            node_types.append(NODE_TYPE_LOOP_ANCHOR)
        elif np.any(component_degrees <= 1):
            node_types.append(NODE_TYPE_ENDPOINT)
        else:
            node_types.append(NODE_TYPE_JUNCTION)

        node_id = len(node_positions) - 1
        for pixel in component_pixels:
            node_index_by_structural_pixel[pixel] = node_id

    edge_records: list[_EdgeRecord] = []
    for raw_edge in _trace_edges(skeleton_mask, structural_mask):
        start_pixel = (int(raw_edge[0, 0]), int(raw_edge[0, 1]))
        end_pixel = (int(raw_edge[-1, 0]), int(raw_edge[-1, 1]))
        if start_pixel not in node_index_by_structural_pixel or end_pixel not in node_index_by_structural_pixel:
            continue

        start_node_id = node_index_by_structural_pixel[start_pixel]
        end_node_id = node_index_by_structural_pixel[end_pixel]
        if start_node_id == end_node_id:
            continue

        raw_edge_xy = np.column_stack([raw_edge[:, 1], raw_edge[:, 0]]).astype(float)
        simplified_polyline = _simplify_chain(
            raw_edge_xy,
            open_mask,
            distance_field,
            clearance_min_cells,
            simplify_samples_per_cell,
        )
        if not _polyline_has_clearance(
            simplified_polyline,
            open_mask,
            distance_field,
            clearance_min_cells,
            simplify_samples_per_cell,
        ):
            continue

        clearances = distance_field[raw_edge[:, 0], raw_edge[:, 1]]
        edge_records.append(
            _EdgeRecord(
                start_node=start_node_id,
                end_node=end_node_id,
                polyline=simplified_polyline,
                length=_polyline_length(simplified_polyline),
                min_clearance=float(clearances.min()),
                mean_clearance=float(clearances.mean()),
            )
        )

    node_positions_array = np.asarray(node_positions, dtype=float) if node_positions else np.empty((0, 2), dtype=float)
    node_types_array = np.asarray(node_types, dtype=int)
    _prune_leaf_edges(
        edge_records,
        len(node_positions_array),
        prune_leaf_length_cells,
        prune_leaf_to_radius_ratio,
    )

    active_edges = [edge for edge in edge_records if edge.active]
    if not active_edges:
        return CostmapTopologyResult(
            distance_field=distance_field.astype(float, copy=False),
            skeleton_mask=skeleton_mask.astype(bool, copy=False),
            node_positions=np.empty((0, 2), dtype=float),
            node_degrees=np.empty((0,), dtype=int),
            node_types=np.empty((0,), dtype=int),
            edge_node_indices=np.empty((0, 2), dtype=int),
            edge_polylines=(),
            edge_lengths=np.empty((0,), dtype=float),
            edge_min_clearances=np.empty((0,), dtype=float),
            edge_mean_clearances=np.empty((0,), dtype=float),
        )

    node_degrees = np.zeros(len(node_positions_array), dtype=int)
    for edge in active_edges:
        node_degrees[edge.start_node] += 1
        node_degrees[edge.end_node] += 1

    active_nodes = node_degrees > 0
    node_remap = {old: new for new, old in enumerate(np.where(active_nodes)[0])}
    edge_node_indices = np.asarray(
        [
            (node_remap[edge.start_node], node_remap[edge.end_node])
            for edge in active_edges
            if edge.start_node in node_remap and edge.end_node in node_remap
        ],
        dtype=int,
    )

    return CostmapTopologyResult(
        distance_field=distance_field.astype(float, copy=False),
        skeleton_mask=skeleton_mask.astype(bool, copy=False),
        node_positions=node_positions_array[active_nodes],
        node_degrees=node_degrees[active_nodes],
        node_types=node_types_array[active_nodes],
        edge_node_indices=edge_node_indices,
        edge_polylines=tuple(edge.polyline for edge in active_edges),
        edge_lengths=np.asarray([edge.length for edge in active_edges], dtype=float),
        edge_min_clearances=np.asarray([edge.min_clearance for edge in active_edges], dtype=float),
        edge_mean_clearances=np.asarray([edge.mean_clearance for edge in active_edges], dtype=float),
    )
