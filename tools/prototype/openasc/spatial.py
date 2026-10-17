"""Small deterministic static AABB hierarchy shared by geometry queries."""
from dataclasses import dataclass


@dataclass
class Node:
    bounds: tuple
    start: int = 0
    end: int = 0
    left: object = None
    right: object = None


def build_bvh(bounds):
    order = list(range(len(bounds)))
    centres = [tuple((b[a] + b[a + 3]) * .5 for a in range(3)) for b in bounds]

    def node(start, end):
        selected = order[start:end]
        box = tuple(min(bounds[i][a] for i in selected) for a in range(3)) + tuple(max(bounds[i][a] for i in selected) for a in range(3, 6))
        if end - start <= 12:
            return Node(box, start, end)
        axis = max(range(3), key=lambda a: box[a + 3] - box[a])
        order[start:end] = sorted(selected, key=lambda i: centres[i][axis])
        middle = (start + end) // 2
        return Node(box, left=node(start, middle), right=node(middle, end))

    return (node(0, len(order)) if order else None), order


def box_candidates(root, order, bounds):
    stack = [root] if root else []
    while stack:
        node = stack.pop()
        if any(node.bounds[a] > bounds[a + 3] or node.bounds[a + 3] < bounds[a] for a in range(3)):
            continue
        if node.left is None:
            yield from order[node.start:node.end]
        else:
            stack.extend((node.right, node.left))
