//! Material index → RGB lookup.
//!
//! Holds the default PDMS colour palette plus any per-file COLR overrides loaded by the
//! parser's pre-scan ([`insert`](ColorStore::insert)); [`get`](ColorStore::get) resolves
//! a material index to a packed `0x00RRGGBB`.

use std::collections::HashMap;

pub struct ColorStore {
    pub map: HashMap<u32, u32>,
}

impl ColorStore {
    pub fn new() -> Self {
        let mut m = HashMap::new();
        let entries: &[(u32, u32)] = &[
            (1, 0x000000),
            (2, 0xcc0000),
            (3, 0xed9900),
            (4, 0xcccc00),
            (5, 0x00cc00),
            (6, 0x00eded),
            (7, 0x0000cc),
            (8, 0xdd00dd),
            (9, 0xcc2b2b),
            (10, 0xffffff),
            (11, 0xf97f70),
            (12, 0xbfbfbf),
            (13, 0xa8a8a8),
            (14, 0x8c668c),
            (15, 0xf4f4f4),
            (16, 0x8e236b),
            (17, 0x00ff7f),
            (18, 0xf4ddb2),
            (19, 0xedc933),
            (20, 0x4775ff),
            (21, 0xede8aa),
            (22, 0xed1189),
            (23, 0x238e23),
            (24, 0xffa500),
            (25, 0xedede0),
            (26, 0xed7521),
            (27, 0x4782b5),
            (28, 0xffffff),
            (29, 0x2d2d4f),
            (30, 0x00007f),
            (31, 0xcc919e),
            (32, 0xcc5b44),
            (33, 0x000000),
            (34, 0xcc0000),
            (35, 0xed9900),
            (36, 0xcccc00),
            (37, 0x00cc00),
            (38, 0x00eded),
            (39, 0x0000cc),
            (40, 0xdd00dd),
            (41, 0xcc2b2b),
            (42, 0xffffff),
            (43, 0xf97f70),
            (44, 0xbfbfbf),
            (45, 0xa8a8a8),
            (46, 0x8c668c),
            (47, 0xf4f4f4),
            (48, 0x8e236b),
            (49, 0x00ff7f),
            (50, 0xf4ddb2),
            (51, 0xedc933),
            (52, 0x4775ff),
            (53, 0xede8aa),
            (54, 0xed1189),
            (55, 0x238e23),
            (56, 0xffa500),
            (57, 0xedede0),
            (58, 0xed7521),
            (59, 0x4782b5),
            (60, 0xffffff),
            (61, 0x2d2d4f),
            (62, 0x00007f),
            (63, 0xcc919e),
            (64, 0xcc5b44),
            (206, 0x000000),
            (207, 0xffffff),
            (208, 0xf4f4f4),
            (209, 0xedede0),
            (210, 0xa8a8a8),
            (211, 0xbfbfbf),
            (212, 0x518c8c),
            (213, 0x2d4f4f),
            (214, 0xcc0000),
            (215, 0xff0000),
            (216, 0xcc5b44),
            (217, 0xff6347),
            (218, 0x8c668c),
            (219, 0xed1189),
            (220, 0xcc919e),
            (221, 0xf97f70),
            (222, 0xed9900),
            (223, 0xffa500),
            (224, 0xff7f00),
            (225, 0x8e236b),
            (226, 0xcccc00),
            (227, 0xedc933),
            (228, 0xededd1),
            (229, 0xede8aa),
            (230, 0x99cc33),
            (231, 0x00ff7f),
            (232, 0x00cc00),
            (233, 0x238e23),
            (234, 0x2d4f2d),
            (235, 0x00eded),
            (236, 0x00bfcc),
            (237, 0x75edc6),
            (238, 0x0000cc),
            (239, 0x4775ff),
            (240, 0x00007f),
            (241, 0xafe0e5),
            (242, 0x2d2d4f),
            (243, 0x4782b5),
            (244, 0x330066),
            (245, 0x660099),
            (246, 0xed82ed),
            (247, 0xdd00dd),
            (248, 0xf4f4db),
            (249, 0xf4ddb2),
            (250, 0xdb9370),
            (251, 0xf4a55e),
            (252, 0xcc2b2b),
            (253, 0x9e9e5e),
            (254, 0xed7521),
            (255, 0x8c4414),
        ];
        for &(k, v) in entries {
            m.insert(k, v);
        }
        Self { map: m }
    }

    pub fn get(&self, id: u32) -> u32 {
        *self.map.get(&id).unwrap_or(&0xffffff)
    }

    /// Insert/override an index→RGB mapping (used by the COLR pre-scan to load the
    /// file's own colour definitions over the built-in defaults).
    pub fn insert(&mut self, id: u32, rgb: u32) {
        self.map.insert(id, rgb);
    }
}
