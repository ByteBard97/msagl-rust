use slotmap::new_key_type;

new_key_type! {
    /// Key for a point in a Polyline's doubly-linked list.
    pub struct PolylinePointKey;
}
