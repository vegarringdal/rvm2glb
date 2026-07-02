// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Port of libtess2 dict.c/h
//
// A sorted doubly-linked list (dictionary) used by the sweep algorithm
// to maintain the active edge set ordered by the comparison function.
//
// In C, keys are ActiveRegion*. Here keys are u32 (ActiveRegion index).
// INVALID = u32::MAX represents a null key (sentinel nodes).

use crate::mesh::INVALID;

/// Index into Dict::nodes
pub type NodeIdx = u32;

#[derive(Clone, Debug)]
pub struct DictNode {
    pub key: u32, // ActiveRegion index, or INVALID for sentinel
    pub next: NodeIdx,
    pub prev: NodeIdx,
}

impl Default for DictNode {
    fn default() -> Self {
        DictNode {
            key: INVALID,
            next: INVALID,
            prev: INVALID,
        }
    }
}

/// A sorted doubly-linked list dictionary.
/// The comparison function takes (frame_data, key1, key2) and returns key1 <= key2.
// The "head" sentinel node is always at index 0.
// It forms a circular list: head.prev == head.next == head when empty.
pub struct Dict {
    pub nodes: Vec<DictNode>,
}

/// Index of the head sentinel node.
pub const DICT_HEAD: NodeIdx = 0;

impl Dict {
    pub fn new() -> Self {
        let mut head = DictNode::default();
        head.key = INVALID;
        head.next = DICT_HEAD;
        head.prev = DICT_HEAD;

        Dict { nodes: vec![head] }
    }

    /// dictInsert: insert a key at the back (before the head sentinel).
    pub fn insert<F>(&mut self, key: u32, leq: &F) -> NodeIdx
    where
        F: Fn(u32, u32) -> bool,
    {
        self.insert_before(DICT_HEAD, key, leq)
    }

    /// dictInsertBefore: insert key before `node`, walking backward to find the
    /// correct sorted position.
    pub fn insert_before<F>(&mut self, mut node: NodeIdx, key: u32, leq: &F) -> NodeIdx
    where
        F: Fn(u32, u32) -> bool,
    {
        // Walk backward until we find a node whose key <= key, or hit the sentinel
        loop {
            node = self.nodes[node as usize].prev;
            let node_key = self.nodes[node as usize].key;
            if node_key == INVALID || leq(node_key, key) {
                break;
            }
        }

        let new_idx = self.nodes.len() as NodeIdx;
        let next_node = self.nodes[node as usize].next;

        let new_node = DictNode {
            key,
            next: next_node,
            prev: node,
        };

        self.nodes.push(new_node);
        self.nodes[node as usize].next = new_idx;
        self.nodes[next_node as usize].prev = new_idx;

        new_idx
    }

    /// dictDelete: remove a node from the dictionary.
    pub fn delete(&mut self, node: NodeIdx) {
        let next = self.nodes[node as usize].next;
        let prev = self.nodes[node as usize].prev;
        self.nodes[next as usize].prev = prev;
        self.nodes[prev as usize].next = next;
        // Mark as deleted
        self.nodes[node as usize].next = INVALID;
        self.nodes[node as usize].prev = INVALID;
        self.nodes[node as usize].key = INVALID;
    }

    /// dictSearch: find the first node with key >= given key.
    pub fn search<F>(&self, key: u32, leq: &F) -> NodeIdx
    where
        F: Fn(u32, u32) -> bool,
    {
        let mut node = DICT_HEAD;
        loop {
            node = self.nodes[node as usize].next;
            let node_key = self.nodes[node as usize].key;
            if node_key == INVALID || leq(key, node_key) {
                return node;
            }
        }
    }

    /// dictKey: get the key of a node.
    #[inline]
    pub fn key(&self, node: NodeIdx) -> u32 {
        self.nodes[node as usize].key
    }

    /// dictMin: first real node (after sentinel).
    #[inline]
    pub fn min(&self) -> NodeIdx {
        self.nodes[DICT_HEAD as usize].next
    }

    /// dictMax: last real node (before sentinel, via prev).
    #[inline]
    pub fn max(&self) -> NodeIdx {
        self.nodes[DICT_HEAD as usize].prev
    }

    /// dictSucc: successor of a node.
    #[inline]
    pub fn succ(&self, node: NodeIdx) -> NodeIdx {
        self.nodes[node as usize].next
    }

    /// dictPred: predecessor of a node.
    #[inline]
    pub fn pred(&self, node: NodeIdx) -> NodeIdx {
        self.nodes[node as usize].prev
    }
}

impl Default for Dict {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn leq(a: u32, b: u32) -> bool {
        a <= b
    }

    #[test]
    fn empty_dict() {
        let d = Dict::new();
        assert_eq!(d.min(), DICT_HEAD);
        assert_eq!(d.max(), DICT_HEAD);
    }

    #[test]
    fn insert_and_order() {
        let mut d = Dict::new();
        d.insert(3, &leq);
        d.insert(1, &leq);
        d.insert(2, &leq);

        // Should be in ascending order: 1, 2, 3
        let n1 = d.min();
        assert_eq!(d.key(n1), 1);
        let n2 = d.succ(n1);
        assert_eq!(d.key(n2), 2);
        let n3 = d.succ(n2);
        assert_eq!(d.key(n3), 3);
        let n_end = d.succ(n3);
        assert_eq!(n_end, DICT_HEAD);
    }

    #[test]
    fn delete_node() {
        let mut d = Dict::new();
        d.insert(1, &leq);
        let n2 = d.insert(2, &leq);
        d.insert(3, &leq);

        d.delete(n2);

        let n1 = d.min();
        assert_eq!(d.key(n1), 1);
        let n3 = d.succ(n1);
        assert_eq!(d.key(n3), 3);
        assert_eq!(d.succ(n3), DICT_HEAD);
    }

    #[test]
    fn search_finds_first_geq() {
        let mut d = Dict::new();
        d.insert(1, &leq);
        d.insert(3, &leq);
        d.insert(5, &leq);

        let n = d.search(2, &leq);
        assert_eq!(d.key(n), 3);

        let n2 = d.search(3, &leq);
        assert_eq!(d.key(n2), 3);

        let n3 = d.search(6, &leq);
        assert_eq!(n3, DICT_HEAD); // Not found → sentinel
    }
}
