//! Shared bounded triangle acceleration. Support retains its historical X/Z split policy.
use super::*;
pub(super) fn build(
    triangles: &[Triangle],
    order: &mut [usize],
    nodes: &mut Vec<Node>,
    start: usize,
    end: usize,
    full_3d: bool,
) -> usize {
    let bounds = order[start + 1..end]
        .iter()
        .fold(triangles[order[start]].bounds, |a, &i| {
            a.union(triangles[i].bounds)
        });
    let index = nodes.len();
    nodes.push(Node {
        bounds,
        start,
        end,
        children: None,
    });
    if end - start > 8 {
        let mut axis = if bounds.hi[0] - bounds.lo[0] >= bounds.hi[2] - bounds.lo[2] {
            0
        } else {
            2
        };
        if full_3d && bounds.hi[1] - bounds.lo[1] > bounds.hi[axis] - bounds.lo[axis] {
            axis = 1;
        }
        order[start..end].sort_unstable_by(|&a, &b| {
            let center = |i: usize| triangles[i].bounds.lo[axis] + triangles[i].bounds.hi[axis];
            center(a).total_cmp(&center(b)).then(a.cmp(&b))
        });
        let middle = start + (end - start) / 2;
        let left = build(triangles, order, nodes, start, middle, full_3d);
        let right = build(triangles, order, nodes, middle, end, full_3d);
        nodes[index].children = Some([left, right]);
    }
    index
}
pub(super) fn candidates(
    triangles: &[Triangle],
    order: &[usize],
    nodes: &[Node],
    bounds: Bounds,
    overflow: impl FnOnce() -> Error,
) -> Result<Vec<usize>> {
    let overlaps = |b: Bounds| {
        (0..3).all(|i| b.hi[i] >= bounds.lo[i] - 1e-8 && b.lo[i] <= bounds.hi[i] + 1e-8)
    };
    let mut indices = Vec::new();
    let mut stack = Vec::new();
    if !nodes.is_empty() {
        stack.push(0);
    }
    while let Some(index) = stack.pop() {
        let node = &nodes[index];
        if !overlaps(node.bounds) {
            continue;
        }
        if let Some(children) = node.children {
            stack.extend(children);
            continue;
        }
        for &i in &order[node.start..node.end] {
            if !overlaps(triangles[i].bounds) {
                continue;
            }
            if indices.len() == 4096 {
                return Err(overflow());
            }
            indices.push(i);
        }
    }
    indices.sort_unstable();
    Ok(indices)
}
