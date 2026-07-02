// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Port of libtess2 priorityq.c/h
//
// A two-phase priority queue:
//   Phase 1 (pre-init): inserts go into a sorted key array.
//   Phase 2 (post-init): inserts go directly into a min-heap.
// Deletion is supported via handles.
//
// In the original C, PQkey = void* (TESSvertex*).
// Here, PQkey = u32 (VertIdx). INVALID_KEY = u32::MAX means "empty/null".

use crate::mesh::INVALID;

pub const INVALID_HANDLE: i32 = 0x0fff_ffff;

/// A heap-based priority queue (used after initialization).
struct Heap {
    /// nodes[1..=size] are active; nodes[0] unused. Stores handle indices.
    nodes: Vec<i32>,
    /// handles[handle] = (key, node_pos)
    handles: Vec<(u32, i32)>,
    size: usize,
    max: usize,
    free_list: i32,
    initialized: bool,
    /// Comparison function: returns true iff key1 <= key2
    leq: fn(u32, u32) -> bool,
}

impl Heap {
    fn new(size: usize, leq: fn(u32, u32) -> bool) -> Self {
        let mut nodes = vec![0i32; size + 2];
        let mut handles = vec![(INVALID, 0i32); size + 2];
        // nodes[1] = 1 so that minimum() returns NULL when empty
        nodes[1] = 1;
        handles[1] = (INVALID, 1);
        Heap {
            nodes,
            handles,
            size: 0,
            max: size,
            free_list: 0,
            initialized: false,
            leq,
        }
    }

    #[inline]
    fn key_of(&self, handle: i32) -> u32 {
        self.handles[handle as usize].0
    }

    fn float_down(&mut self, mut curr: usize) {
        let h_curr = self.nodes[curr];
        loop {
            let mut child = curr << 1;
            if child < self.size {
                let child_key = self.key_of(self.nodes[child + 1]);
                let child_key0 = self.key_of(self.nodes[child]);
                if (self.leq)(child_key, child_key0) {
                    child += 1;
                }
            }
            let h_child = self.nodes[child];
            if child > self.size || (self.leq)(self.key_of(h_curr), self.key_of(h_child)) {
                self.nodes[curr] = h_curr;
                self.handles[h_curr as usize].1 = curr as i32;
                break;
            }
            self.nodes[curr] = h_child;
            self.handles[h_child as usize].1 = curr as i32;
            curr = child;
        }
    }

    fn float_up(&mut self, mut curr: usize) {
        let h_curr = self.nodes[curr];
        loop {
            let parent = curr >> 1;
            let h_parent = self.nodes[parent];
            if parent == 0 || (self.leq)(self.key_of(h_parent), self.key_of(h_curr)) {
                self.nodes[curr] = h_curr;
                self.handles[h_curr as usize].1 = curr as i32;
                break;
            }
            self.nodes[curr] = h_parent;
            self.handles[h_parent as usize].1 = curr as i32;
            curr = parent;
        }
    }

    fn init(&mut self) {
        for i in (1..=self.size).rev() {
            self.float_down(i);
        }
        self.initialized = true;
    }

    fn insert(&mut self, key: u32) -> i32 {
        self.size += 1;
        let curr = self.size;

        // Grow if needed
        if curr * 2 > self.max {
            self.max <<= 1;
            self.nodes.resize(self.max + 2, 0);
            self.handles.resize(self.max + 2, (INVALID, 0));
        }

        let free_handle = if self.free_list == 0 {
            curr as i32
        } else {
            let f = self.free_list;
            self.free_list = self.handles[f as usize].1;
            f
        };

        self.nodes[curr] = free_handle;
        self.handles[free_handle as usize] = (key, curr as i32);

        if self.initialized {
            self.float_up(curr);
        }

        free_handle
    }

    fn extract_min(&mut self) -> u32 {
        let h_min = self.nodes[1];
        let min_key = self.handles[h_min as usize].0;

        if self.size > 0 {
            self.nodes[1] = self.nodes[self.size];
            self.handles[self.nodes[1] as usize].1 = 1;

            self.handles[h_min as usize].0 = INVALID;
            self.handles[h_min as usize].1 = self.free_list;
            self.free_list = h_min;

            self.size -= 1;
            if self.size > 0 {
                self.float_down(1);
            }
        }

        min_key
    }

    fn delete(&mut self, h_curr: i32) {
        debug_assert!(self.handles[h_curr as usize].0 != INVALID);
        let curr = self.handles[h_curr as usize].1 as usize;

        self.nodes[curr] = self.nodes[self.size];
        self.handles[self.nodes[curr] as usize].1 = curr as i32;

        if curr <= self.size {
            self.size -= 1;
            if curr <= 1 {
                self.float_down(curr);
            } else {
                let parent_key = self.key_of(self.nodes[curr >> 1]);
                let curr_key = self.key_of(self.nodes[curr]);
                if (self.leq)(parent_key, curr_key) {
                    self.float_down(curr);
                } else {
                    self.float_up(curr);
                }
            }
        } else {
            self.size -= 1;
        }

        self.handles[h_curr as usize].0 = INVALID;
        self.handles[h_curr as usize].1 = self.free_list;
        self.free_list = h_curr;
    }

    #[inline]
    fn minimum(&self) -> u32 {
        self.handles[self.nodes[1] as usize].0
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.size == 0
    }
}

/// The combined priority queue (sort-array + heap).
pub struct PriorityQ {
    heap: Heap,
    /// Pre-init key storage
    keys: Vec<u32>,
    /// Sorted indirect pointers into keys (indices)
    order: Vec<usize>,
    size: usize,
    max: usize,
    initialized: bool,
    leq: fn(u32, u32) -> bool,
}

impl PriorityQ {
    pub fn new(size: usize, leq: fn(u32, u32) -> bool) -> Self {
        PriorityQ {
            heap: Heap::new(size, leq),
            keys: Vec::with_capacity(size),
            order: Vec::new(),
            size: 0,
            max: size,
            initialized: false,
            leq,
        }
    }

    /// Initialize the sort-array phase.
    /// Must be called before extract_min/minimum/delete (but after all pre-init inserts).
    pub fn init(&mut self) -> bool {
        // Create indirect pointer array
        self.order = (0..self.size).collect();

        // Sort in descending order (so we pop from the end in ascending order)
        let keys = &self.keys;
        let leq = self.leq;
        self.order.sort_unstable_by(|&a, &b| {
            // descending: if keys[a] <= keys[b], b comes first
            if (leq)(keys[a], keys[b]) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Less
            }
        });

        self.max = self.size;
        self.initialized = true;
        self.heap.init();
        true
    }

    /// Insert a key. Returns a handle.
    /// Negative handles are for the sort-array; non-negative for the heap.
    pub fn insert(&mut self, key: u32) -> i32 {
        if self.initialized {
            return self.heap.insert(key);
        }

        let curr = self.size;
        self.size += 1;

        if self.size > self.max {
            self.max <<= 1;
        }

        if curr >= self.keys.len() {
            self.keys.push(key);
        } else {
            self.keys[curr] = key;
        }

        // Negative handles index the sort array
        -(curr as i32 + 1)
    }

    /// Extract the minimum key.
    pub fn extract_min(&mut self) -> u32 {
        if self.size == 0 {
            return self.heap.extract_min();
        }

        let sort_min = self.keys[self.order[self.size - 1]];

        if !self.heap.is_empty() {
            let heap_min = self.heap.minimum();
            if (self.leq)(heap_min, sort_min) {
                return self.heap.extract_min();
            }
        }

        // Pop from sort array, skipping deleted (INVALID) entries
        loop {
            self.size -= 1;
            if self.size == 0 || self.keys[self.order[self.size - 1]] != INVALID {
                break;
            }
        }

        sort_min
    }

    /// Peek at the minimum key without extracting.
    pub fn minimum(&self) -> u32 {
        if self.size == 0 {
            return self.heap.minimum();
        }

        let sort_min = self.keys[self.order[self.size - 1]];

        if !self.heap.is_empty() {
            let heap_min = self.heap.minimum();
            if (self.leq)(heap_min, sort_min) {
                return heap_min;
            }
        }

        sort_min
    }

    /// Returns true if the queue is empty.
    pub fn is_empty(&self) -> bool {
        self.size == 0 && self.heap.is_empty()
    }

    /// Delete the key with the given handle.
    pub fn delete(&mut self, handle: i32) {
        if handle >= 0 {
            self.heap.delete(handle);
            return;
        }

        let curr = (-(handle + 1)) as usize;
        debug_assert!(curr < self.keys.len() && self.keys[curr] != INVALID);
        self.keys[curr] = INVALID;

        // Trim trailing deleted entries
        while self.size > 0 && self.keys[self.order[self.size - 1]] == INVALID {
            self.size -= 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::vert_leq;

    fn leq_u32(a: u32, b: u32) -> bool {
        a <= b
    }

    #[test]
    fn heap_basic() {
        let mut h = Heap::new(8, leq_u32);
        h.init();
        h.insert(3);
        h.insert(1);
        h.insert(2);
        assert_eq!(h.minimum(), 1);
        assert_eq!(h.extract_min(), 1);
        assert_eq!(h.extract_min(), 2);
        assert_eq!(h.extract_min(), 3);
        assert!(h.is_empty());
    }

    #[test]
    fn pq_pre_init_insert_then_extract() {
        let mut pq = PriorityQ::new(8, leq_u32);
        pq.insert(5);
        pq.insert(2);
        pq.insert(8);
        pq.insert(1);
        pq.init();

        assert_eq!(pq.extract_min(), 1);
        assert_eq!(pq.extract_min(), 2);
        assert_eq!(pq.extract_min(), 5);
        assert_eq!(pq.extract_min(), 8);
        assert!(pq.is_empty());
    }

    #[test]
    fn pq_delete_from_sort_array() {
        let mut pq = PriorityQ::new(8, leq_u32);
        let h1 = pq.insert(10);
        let _h2 = pq.insert(5);
        let h3 = pq.insert(7);
        pq.init();
        pq.delete(h1);
        assert_eq!(pq.extract_min(), 5);
        assert_eq!(pq.extract_min(), 7);
        assert!(pq.is_empty());
    }

    #[test]
    fn pq_post_init_insert() {
        let mut pq = PriorityQ::new(4, leq_u32);
        pq.insert(3);
        pq.init();
        pq.insert(1); // goes into heap
        assert_eq!(pq.minimum(), 1);
        assert_eq!(pq.extract_min(), 1);
        assert_eq!(pq.extract_min(), 3);
    }
}
