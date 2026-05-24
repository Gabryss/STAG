import sys
from pathlib import Path
import unittest

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from stag_core.topology_core import (  # noqa: E402
    analyze_costmap_topology,
    extract_auxiliary_graph_nodes,
    flatten_polylines,
    occupancy_grid_to_open_mask,
    occupancy_grid_to_traversability,
)


class TopologyCoreTest(unittest.TestCase):
    def test_empty_map_has_no_graph(self):
        mask = np.zeros((20, 20), dtype=bool)
        result = analyze_costmap_topology(mask)
        self.assertEqual(len(result.node_positions), 0)
        self.assertEqual(len(result.edge_polylines), 0)

    def test_fully_blocked_occupancy_grid_has_no_graph(self):
        values = np.full((30, 30), 100, dtype=np.int16)
        mask = occupancy_grid_to_open_mask(values.ravel(), 30, 30, 49, True)
        result = analyze_costmap_topology(mask)
        self.assertFalse(np.any(mask))
        self.assertEqual(len(result.edge_polylines), 0)

    def test_corridor_map_extracts_graph(self):
        mask = np.zeros((60, 60), dtype=bool)
        mask[28:33, 5:55] = True
        result = analyze_costmap_topology(mask, prune_leaf_length_cells=0.0)
        self.assertGreaterEqual(len(result.node_positions), 2)
        self.assertGreaterEqual(len(result.edge_polylines), 1)

    def test_cross_map_extracts_junction_graph(self):
        mask = np.zeros((80, 80), dtype=bool)
        mask[38:43, 10:70] = True
        mask[10:70, 38:43] = True
        result = analyze_costmap_topology(mask, prune_leaf_length_cells=0.0)
        self.assertGreaterEqual(len(result.node_positions), 5)
        self.assertGreaterEqual(len(result.edge_polylines), 4)

    def test_unknown_cell_handling(self):
        values = np.asarray([-1, 0, 50, 100], dtype=np.int16)
        blocked_unknown = occupancy_grid_to_open_mask(values, 4, 1, 49, True)
        open_unknown = occupancy_grid_to_open_mask(values, 4, 1, 49, False)
        self.assertEqual(blocked_unknown.tolist(), [[False, True, False, False]])
        self.assertEqual(open_unknown.tolist(), [[True, True, False, False]])

    def test_invalid_inputs_raise_value_error(self):
        with self.assertRaises(ValueError):
            occupancy_grid_to_open_mask(np.asarray([0, 1]), 3, 1, 49, True)
        with self.assertRaises(ValueError):
            analyze_costmap_topology(np.ones((4,), dtype=bool))
        with self.assertRaises(ValueError):
            analyze_costmap_topology(np.ones((4, 4), dtype=bool), simplify_samples_per_cell=0.0)

    def test_auxiliary_nodes_and_compact_polyline_shape(self):
        values = np.full((80, 80), 100, dtype=np.int16)
        values[38:43, 10:70] = 5
        values[10:70, 38:43] = 20
        values[30:50, 30:50] = 0
        mask = occupancy_grid_to_open_mask(values.ravel(), 80, 80, 49, True)
        traversability = occupancy_grid_to_traversability(values.ravel(), 80, 80, mask)
        topology = analyze_costmap_topology(mask, prune_leaf_length_cells=0.0)
        auxiliary = extract_auxiliary_graph_nodes(
            traversability,
            mask,
            gradient_node_count=8,
            gradient_min_separation_cells=4.0,
        )
        points, offsets = flatten_polylines(topology.edge_polylines)
        self.assertEqual(points.shape[1], 2)
        self.assertEqual(offsets[0], 0)
        self.assertEqual(offsets[-1], len(points))
        self.assertGreater(len(auxiliary.region_positions), 0)
        self.assertLessEqual(len(auxiliary.gradient_positions), 8)


if __name__ == "__main__":
    unittest.main()
