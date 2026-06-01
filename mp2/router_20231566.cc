#include "netsim2.h"
#include <cstring>
#include <cstdlib>
#include <climits>
#include <map>
#include <vector>
#include <queue>

// Message type tag
#define MSG_LSA 1

// LSA header: [type(1), origin(2), seq(4), num_edges(2), edges...]
// Each edge: [neighbor(2), cost(2)]
// All values little-endian.

struct RouterState {
    int my_id;
    int num_nodes;

    // Direct neighbors: neighbor_id -> cost (-1 = down)
    std::map<int, int> neighbors;

    // Link state database: origin -> { seq, {neighbor->cost} }
    struct LSEntry {
        uint32_t seq;
        std::map<int, int> edges; // neighbor->cost
    };
    std::map<int, LSEntry> lsdb;

    // My current LSA sequence number
    uint32_t my_seq;

    // Routing table: dst -> next_hop
    std::map<int, int> routing_table;

    // Timer interval for periodic LSA flood (safety net only, very infrequent)
    static const int FLOOD_INTERVAL = 10000;
    bool timer_scheduled;
};

// ---- helpers ----

static void encode_le2(uint8_t* buf, uint16_t v) {
    buf[0] = v & 0xff;
    buf[1] = (v >> 8) & 0xff;
}
static void encode_le4(uint8_t* buf, uint32_t v) {
    buf[0] = v & 0xff;
    buf[1] = (v >> 8) & 0xff;
    buf[2] = (v >> 16) & 0xff;
    buf[3] = (v >> 24) & 0xff;
}
static uint16_t decode_le2(const uint8_t* buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}
static uint32_t decode_le4(const uint8_t* buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

// Build LSA payload from my current neighbors
static int build_lsa(RouterState* s, uint8_t* buf, int bufsz) {
    // Count active neighbors
    int cnt = 0;
    for (auto& kv : s->neighbors)
        if (kv.second != NETSIM2_NO_LINK) cnt++;

    int needed = 1 + 2 + 4 + 2 + cnt * 4;
    if (needed > bufsz) return -1;

    buf[0] = MSG_LSA;
    encode_le2(buf + 1, (uint16_t)s->my_id);
    encode_le4(buf + 3, s->my_seq);
    encode_le2(buf + 7, (uint16_t)cnt);
    int off = 9;
    for (auto& kv : s->neighbors) {
        if (kv.second == NETSIM2_NO_LINK) continue;
        encode_le2(buf + off, (uint16_t)kv.first);
        encode_le2(buf + off + 2, (uint16_t)kv.second);
        off += 4;
    }
    return off;
}

// Flood LSA to all up neighbors
static void flood_lsa(RouterState* s) {
    uint8_t buf[1 + 2 + 4 + 2 + 100 * 4]; // max 100 neighbors
    int len = build_lsa(s, buf, sizeof(buf));
    if (len < 0) return;
    for (auto& kv : s->neighbors) {
        if (kv.second != NETSIM2_NO_LINK)
            send_control(kv.first, buf, len);
    }
}

// Flood received LSA to all up neighbors except the one it came from
static void reflood_lsa(RouterState* s, int except_neighbor,
                        const uint8_t* payload, int len) {
    for (auto& kv : s->neighbors) {
        if (kv.second != NETSIM2_NO_LINK && kv.first != except_neighbor)
            send_control(kv.first, payload, len);
    }
}

// Dijkstra over lsdb to rebuild routing table
static void run_dijkstra(RouterState* s) {
    int N = s->num_nodes;
    std::vector<int> dist(N, INT_MAX);
    std::vector<int> next_hop(N, -1);

    dist[s->my_id] = 0;

    // min-heap: (dist, node, first_hop)
    using T3 = std::tuple<int,int,int>;
    std::priority_queue<T3, std::vector<T3>, std::greater<T3>> pq;
    pq.push({0, s->my_id, -1});

    while (!pq.empty()) {
        auto [d, u, hop] = pq.top(); pq.pop();
        if (d > dist[u]) continue;

        auto it = s->lsdb.find(u);
        if (it == s->lsdb.end()) continue;

        for (auto& [v, c] : it->second.edges) {
            int nd = d + c;
            if (nd < dist[v]) {
                dist[v] = nd;
                next_hop[v] = (u == s->my_id) ? v : hop;
                pq.push({nd, v, next_hop[v]});
            }
        }
    }

    s->routing_table.clear();
    for (int i = 0; i < N; i++) {
        if (i != s->my_id && next_hop[i] != -1)
            s->routing_table[i] = next_hop[i];
    }
}

// Update my own entry in lsdb and rerun dijkstra
static void update_my_lsdb(RouterState* s) {
    RouterState::LSEntry& e = s->lsdb[s->my_id];
    e.seq = s->my_seq;
    e.edges.clear();
    for (auto& kv : s->neighbors)
        if (kv.second != NETSIM2_NO_LINK)
            e.edges[kv.first] = kv.second;
    run_dijkstra(s);
}

// ---- callbacks ----

struct RouterState* router_init(int my_id, int num_nodes,
                                const int* neighbor_ids,
                                const int* link_costs,
                                int num_neighbors) {
    RouterState* s = new RouterState();
    s->my_id = my_id;
    s->num_nodes = num_nodes;
    s->my_seq = 0;

    for (int i = 0; i < num_neighbors; i++)
        s->neighbors[neighbor_ids[i]] = link_costs[i];

    // Seed lsdb with my own info
    update_my_lsdb(s);

    // Send initial LSA
    s->my_seq++;
    update_my_lsdb(s);
    flood_lsa(s);

    // Schedule periodic re-flood
    schedule_wakeup(get_now() + RouterState::FLOOD_INTERVAL);
    return s;
}

void on_link_change(struct RouterState* s, int neighbor, int new_cost) {
    s->neighbors[neighbor] = new_cost;

    s->my_seq++;
    update_my_lsdb(s);
    flood_lsa(s);
}

void on_control(struct RouterState* s, int from,
                const uint8_t* payload, int len) {
    if (len < 9) return;
    if (payload[0] != MSG_LSA) return;

    int origin = (int)decode_le2(payload + 1);
    uint32_t seq = decode_le4(payload + 3);
    int num_edges = (int)decode_le2(payload + 7);

    if (origin < 0 || origin >= s->num_nodes) return;
    if (len < 9 + num_edges * 4) return;

    // Check if we already have a newer/equal version
    auto it = s->lsdb.find(origin);
    if (it != s->lsdb.end() && it->second.seq >= seq) return;

    // Store new LSA
    RouterState::LSEntry& e = s->lsdb[origin];
    e.seq = seq;
    e.edges.clear();
    for (int i = 0; i < num_edges; i++) {
        int nb = (int)decode_le2(payload + 9 + i * 4);
        int co = (int)decode_le2(payload + 9 + i * 4 + 2);
        e.edges[nb] = co;
    }

    // Rerun Dijkstra
    run_dijkstra(s);

    // Flood to others
    reflood_lsa(s, from, payload, len);
}

int on_packet(struct RouterState* s, int dst) {
    auto it = s->routing_table.find(dst);
    if (it == s->routing_table.end()) {
        // Fallback: pick any up neighbor
        for (auto& kv : s->neighbors)
            if (kv.second != NETSIM2_NO_LINK) return kv.first;
        return -1;
    }
    int nh = it->second;
    // Verify next hop is still up
    auto nb_it = s->neighbors.find(nh);
    if (nb_it == s->neighbors.end() || nb_it->second == NETSIM2_NO_LINK) {
        // Try to find alternate via routing table or any up neighbor
        for (auto& kv : s->neighbors)
            if (kv.second != NETSIM2_NO_LINK) return kv.first;
        return -1;
    }
    return nh;
}

void on_timer(struct RouterState* s) {
    // Periodic LSA re-flood to ensure convergence
    s->my_seq++;
    update_my_lsdb(s);
    flood_lsa(s);
    schedule_wakeup(get_now() + RouterState::FLOOD_INTERVAL);
}

void router_shutdown(struct RouterState* s) {
    delete s;
}
