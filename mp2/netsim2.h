#ifndef NETSIM2_H
#define NETSIM2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Sentinel value used in on_link_change when a link goes down.
#define NETSIM2_NO_LINK -1

// Opaque router state. The student defines its members in their own .cc file.
struct RouterState;

// =================================================================
// Callbacks the student must implement
// =================================================================

// Called once at simulation start for each node (in ID ascending order).
// Allocate and return a RouterState* for this node. The returned pointer
// is passed back as `s` in every subsequent callback for this node.
//
// `neighbor_ids[]` and `link_costs[]` are arrays of length `num_neighbors`,
// listing the directly-connected neighbors at start-of-simulation.
// `neighbor_ids[]` is sorted in ascending order by ID.
// When num_neighbors == 0 the two pointers may be NULL.
struct RouterState* router_init(int my_id,
                                int num_nodes,
                                const int* neighbor_ids,
                                const int* link_costs,
                                int num_neighbors);

// Called when a directly-connected link of this router changes:
//   * cost change of an existing link, OR
//   * a brand-new link to a previously-unconnected node, OR
//   * link going down (new_cost == NETSIM2_NO_LINK).
//
// Not called when a LINK event in the scenario does not actually change
// the current cost.
void on_link_change(struct RouterState* s, int neighbor, int new_cost);

// Called when a control message from a directly-connected neighbor arrives.
// `payload` and its `len` bytes are valid only during this call; the
// student must copy what they want to keep.
void on_control(struct RouterState* s, int from,
                const uint8_t* payload, int len);

// Called when a data packet arrives at this node and is not yet at the
// destination. Return the neighbor ID to forward the packet to. The
// returned ID must be a current up-state neighbor.
// Return -1 to drop the packet (causes scenario FAIL).
int on_packet(struct RouterState* s, int dst);

// Called at the virtual time previously requested by schedule_wakeup().
void on_timer(struct RouterState* s);

// Called once at simulation end. Free state if necessary.
void router_shutdown(struct RouterState* s);

// =================================================================
// API the student may call from within any callback (except shutdown)
// =================================================================

// Send `len` bytes of `payload` to a directly-connected neighbor.
// Allowed `len` range is 1 to 65536 (64 KB). The simulator copies the bytes
// internally, so the caller's buffer may be freed/reused immediately.
// If `neighbor` is not currently a neighbor (link down or never existed),
// or `len` is out of range, the call is ignored with a stderr warning
// and counts 0 bytes toward control_bytes_total.
void send_control(int neighbor, const uint8_t* payload, int len);

// Returns the current virtual time.
int64_t get_now(void);

// Schedule on_timer() to be called for the calling router at virtual
// time `when`. If `when <= now`, fires at the next event step.
// Multiple wake-ups can be scheduled; all fire in time order.
void schedule_wakeup(int64_t when);

#ifdef __cplusplus
}
#endif

#endif // NETSIM2_H
