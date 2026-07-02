// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Port of libtess2 bucketalloc.c/h
//
// In Rust, the bucket allocator pattern is replaced by Vec-backed arenas.
// This module provides the BucketAlloc type as a thin wrapper used by the
// mesh, dict, and region pool subsystems.

/// A simple arena allocator backed by a Vec.
/// Items are allocated by pushing to the vec and freed via a freelist.
pub struct BucketAlloc<T> {
    items: Vec<Option<T>>,
    free_list: Vec<u32>,
}

impl<T: Default> BucketAlloc<T> {
    pub fn new() -> Self {
        Self {
            items: Vec::new(),
            free_list: Vec::new(),
        }
    }

    /// Allocate a new item, returning its index.
    pub fn alloc(&mut self) -> u32 {
        if let Some(idx) = self.free_list.pop() {
            self.items[idx as usize] = Some(T::default());
            idx
        } else {
            let idx = self.items.len() as u32;
            self.items.push(Some(T::default()));
            idx
        }
    }

    /// Free an item by index (returns it to the free list).
    pub fn free(&mut self, idx: u32) {
        self.items[idx as usize] = None;
        self.free_list.push(idx);
    }

    pub fn get(&self, idx: u32) -> Option<&T> {
        self.items.get(idx as usize)?.as_ref()
    }

    pub fn get_mut(&mut self, idx: u32) -> Option<&mut T> {
        self.items.get_mut(idx as usize)?.as_mut()
    }
}

impl<T: Default> Default for BucketAlloc<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alloc_and_free() {
        let mut ba: BucketAlloc<u32> = BucketAlloc::new();
        let a = ba.alloc();
        let b = ba.alloc();
        assert_ne!(a, b);
        ba.free(a);
        let c = ba.alloc();
        // c should reuse a's slot
        assert_eq!(c, a);
    }

    #[test]
    fn get_returns_default() {
        let mut ba: BucketAlloc<i32> = BucketAlloc::new();
        let idx = ba.alloc();
        assert_eq!(*ba.get(idx).unwrap(), 0);
    }

    #[test]
    fn get_after_free_returns_none() {
        let mut ba: BucketAlloc<i32> = BucketAlloc::new();
        let idx = ba.alloc();
        ba.free(idx);
        assert!(ba.get(idx).is_none());
    }
}
