// netsim2_lib.cc
//
// Discrete-event simulation runtime for MP2. Linked with the student's
// router code to produce a single executable. Contains main().
//
// Architecture (A1): single process; one student router instance per
// node; the simulator drives event-driven callbacks against each
// router's state.

#include "netsim2.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <queue>
#include <string>
#include <fstream>
#include <sstream>
#include <tuple>

namespace {

// ---------- Constants ----------
constexpr int K_VALUE       = 10;     // cost weight: 1 control byte = K path cost
constexpr int MAX_PAYLOAD   = 65536;  // per send_control call (64 KB)
constexpr int MAX_NODES     = 100;
constexpr int MIN_LINK_COST = 1;
constexpr int MAX_LINK_COST = 100;

// ---------- Event types ----------
enum EventType {
    ET_SCENARIO_LINK,
    ET_SCENARIO_PKT,
    ET_CONTROL_ARRIVAL,
    ET_PACKET_ARRIVAL,
    ET_TIMER_WAKEUP
};

struct Event {
    int64_t time = 0;
    int64_t seq  = 0;       // creation order; tiebreaker
    EventType type;

    // type-specific fields
    int from_id = -1;
    int to_id   = -1;
    int dst_id  = -1;
    int cost    = 0;
    int ttl_remaining = 0;
    int64_t path_cost_so_far = 0;
    std::vector<uint8_t> payload;
};

struct EventCmp {
    // Returns true if a should be processed AFTER b
    // (priority_queue is a max-heap by default, so this gives min-heap by time).
    bool operator()(const Event* a, const Event* b) const {
        if (a->time != b->time) return a->time > b->time;
        return a->seq > b->seq;
    }
};

// ---------- Simulator state (all internal to this translation unit) ----------

int g_num_nodes = 0;
int64_t g_now = 0;
int g_current_router = -1;    // who is "calling" us right now (for send_control/etc)

// Adjacency matrix: g_adj[u][v] = cost (>=1) if up, NETSIM2_NO_LINK (-1) if down/absent.
std::vector<std::vector<int>> g_adj;

// Per-router student-defined state.
std::vector<RouterState*> g_states;

// Event queue + monotonic sequence counter.
std::priority_queue<Event*, std::vector<Event*>, EventCmp> g_events;
int64_t g_next_seq = 0;

// Termination counters: simulation ends when both reach 0.
int g_pending_scenario_links = 0;
int g_pending_packets        = 0;   // packets that have been generated but not yet resolved

// Statistics.
int64_t g_path_cost_total    = 0;
int64_t g_control_bytes_total = 0;
int g_packets_total          = 0;
int g_packets_delivered      = 0;
int g_packets_dropped        = 0;

enum SimStatus {
    SIM_SUCCESS,
    SIM_FAIL_DROPPED,
    SIM_PROTOCOL_ERROR
};
SimStatus g_status = SIM_SUCCESS;

// ---------- Helpers ----------

void enqueue(Event* e) {
    if (e->time < g_now) e->time = g_now;
    e->seq = g_next_seq++;
    g_events.push(e);
}

bool valid_node_id(int id) {
    return id >= 0 && id < g_num_nodes;
}

bool is_up_neighbor(int from, int to) {
    if (!valid_node_id(from) || !valid_node_id(to)) return false;
    if (from == to) return false;
    return g_adj[from][to] >= MIN_LINK_COST;
}

void escalate_status(SimStatus s) {
    // PROTOCOL_ERROR > FAIL_DROPPED > SUCCESS
    if (s == SIM_PROTOCOL_ERROR) g_status = SIM_PROTOCOL_ERROR;
    else if (s == SIM_FAIL_DROPPED && g_status == SIM_SUCCESS) g_status = SIM_FAIL_DROPPED;
}

// ---------- Scenario parsing ----------

struct ScenarioFile {
    int num_nodes = 0;
    std::vector<std::tuple<int,int,int>> edges;  // (u, v, cost)

    struct Evt {
        int64_t t;
        char kind;     // 'L' or 'P'
        int a, b, c;   // L: u,v,cost; P: src,dst,(unused)
        int line_no;
    };
    std::vector<Evt> events;
};

bool parse_scenario(const char* path, ScenarioFile& sf) {
    std::ifstream f(path);
    if (!f) {
        fprintf(stderr, "netsim2: cannot open scenario file '%s'\n", path);
        return false;
    }

    std::string line;
    int line_no = 0;
    bool nodes_seen = false;
    bool last_t_set = false;
    int64_t last_t = 0;

    while (std::getline(f, line)) {
        ++line_no;
        // strip leading whitespace
        size_t s = line.find_first_not_of(" \t\r");
        if (s == std::string::npos) continue;          // blank
        if (line[s] == '#') continue;                  // comment

        std::istringstream iss(line.substr(s));
        std::string kw;
        iss >> kw;

        if (kw == "NODES") {
            int n; iss >> n;
            if (!iss || n < 1 || n > MAX_NODES) {
                fprintf(stderr, "netsim2: line %d: NODES out of range [1, %d]\n", line_no, MAX_NODES);
                return false;
            }
            if (nodes_seen) {
                fprintf(stderr, "netsim2: line %d: duplicate NODES line\n", line_no);
                return false;
            }
            sf.num_nodes = n;
            nodes_seen = true;
        } else if (kw == "EDGE") {
            if (!nodes_seen) {
                fprintf(stderr, "netsim2: line %d: EDGE before NODES\n", line_no);
                return false;
            }
            int u, v, c;
            iss >> u >> v >> c;
            if (!iss) {
                fprintf(stderr, "netsim2: line %d: EDGE malformed\n", line_no);
                return false;
            }
            if (u < 0 || u >= sf.num_nodes || v < 0 || v >= sf.num_nodes || u == v) {
                fprintf(stderr, "netsim2: line %d: EDGE invalid node IDs (u=%d v=%d N=%d)\n",
                        line_no, u, v, sf.num_nodes);
                return false;
            }
            if (c < MIN_LINK_COST || c > MAX_LINK_COST) {
                fprintf(stderr, "netsim2: line %d: EDGE cost %d out of range [%d, %d]\n",
                        line_no, c, MIN_LINK_COST, MAX_LINK_COST);
                return false;
            }
            sf.edges.emplace_back(u, v, c);
        } else if (kw == "T") {
            if (!nodes_seen) {
                fprintf(stderr, "netsim2: line %d: T before NODES\n", line_no);
                return false;
            }
            int64_t t;
            std::string ekind;
            iss >> t >> ekind;
            if (!iss || t < 0) {
                fprintf(stderr, "netsim2: line %d: T event malformed\n", line_no);
                return false;
            }
            if (last_t_set && t < last_t) {
                fprintf(stderr, "netsim2: line %d: T value must be non-decreasing (%lld < %lld)\n",
                        line_no, (long long)t, (long long)last_t);
                return false;
            }
            last_t = t; last_t_set = true;

            ScenarioFile::Evt e;
            e.t = t; e.line_no = line_no;
            if (ekind == "LINK") {
                int u, v, c;
                iss >> u >> v >> c;
                if (!iss) { fprintf(stderr, "netsim2: line %d: LINK malformed\n", line_no); return false; }
                if (u < 0 || u >= sf.num_nodes || v < 0 || v >= sf.num_nodes || u == v) {
                    fprintf(stderr, "netsim2: line %d: LINK invalid node IDs\n", line_no);
                    return false;
                }
                if (c != -1 && (c < MIN_LINK_COST || c > MAX_LINK_COST)) {
                    fprintf(stderr, "netsim2: line %d: LINK cost %d out of range\n", line_no, c);
                    return false;
                }
                e.kind = 'L'; e.a = u; e.b = v; e.c = c;
            } else if (ekind == "PKT") {
                int src, dst;
                iss >> src >> dst;
                if (!iss) { fprintf(stderr, "netsim2: line %d: PKT malformed\n", line_no); return false; }
                if (src < 0 || src >= sf.num_nodes || dst < 0 || dst >= sf.num_nodes) {
                    fprintf(stderr, "netsim2: line %d: PKT invalid node IDs\n", line_no);
                    return false;
                }
                e.kind = 'P'; e.a = src; e.b = dst; e.c = 0;
            } else {
                fprintf(stderr, "netsim2: line %d: unknown event kind '%s'\n", line_no, ekind.c_str());
                return false;
            }
            sf.events.push_back(e);
        } else {
            fprintf(stderr, "netsim2: line %d: unknown keyword '%s'\n", line_no, kw.c_str());
            return false;
        }
    }

    if (!nodes_seen) {
        fprintf(stderr, "netsim2: NODES line missing\n");
        return false;
    }
    return true;
}

// ---------- Simulation steps ----------

void initialize_topology(const ScenarioFile& sf) {
    g_num_nodes = sf.num_nodes;
    g_adj.assign(g_num_nodes, std::vector<int>(g_num_nodes, NETSIM2_NO_LINK));
    for (auto& [u, v, c] : sf.edges) {
        g_adj[u][v] = c;
        g_adj[v][u] = c;
    }
}

void initialize_routers() {
    g_states.assign(g_num_nodes, nullptr);
    for (int i = 0; i < g_num_nodes; ++i) {
        std::vector<int> ids, costs;
        for (int j = 0; j < g_num_nodes; ++j) {
            if (g_adj[i][j] >= MIN_LINK_COST) {
                ids.push_back(j);
                costs.push_back(g_adj[i][j]);
            }
        }
        g_current_router = i;
        const int* ids_p   = ids.empty()   ? nullptr : ids.data();
        const int* costs_p = costs.empty() ? nullptr : costs.data();
        g_states[i] = router_init(i, g_num_nodes, ids_p, costs_p, (int)ids.size());
    }
    g_current_router = -1;
}

void enqueue_scenario(const ScenarioFile& sf) {
    for (auto& se : sf.events) {
        Event* e = new Event();
        e->time = se.t;
        if (se.kind == 'L') {
            e->type    = ET_SCENARIO_LINK;
            e->from_id = se.a;
            e->to_id   = se.b;
            e->cost    = se.c;
            ++g_pending_scenario_links;
        } else {
            e->type    = ET_SCENARIO_PKT;
            e->from_id = se.a;
            e->dst_id  = se.b;
            ++g_pending_packets;
        }
        enqueue(e);
    }
}

// Try to forward a packet from `at` to next hop (calling on_packet).
// Returns true if forwarding succeeded (event scheduled); false if the
// packet was resolved here (dropped explicitly or via PROTOCOL_ERROR).
bool forward_packet(int at, int dst, int ttl_after_call, int64_t accumulated_cost) {
    g_current_router = at;
    int next_hop = on_packet(g_states[at], dst);
    g_current_router = -1;

    if (next_hop == -1) {
        // Explicit drop
        ++g_packets_dropped;
        --g_pending_packets;
        escalate_status(SIM_FAIL_DROPPED);
        return false;
    }

    if (!is_up_neighbor(at, next_hop)) {
        fprintf(stderr, "netsim2: PROTOCOL_ERROR: node %d on_packet returned %d which is not a "
                        "current up-state neighbor\n", at, next_hop);
        escalate_status(SIM_PROTOCOL_ERROR);
        // Don't decrement pending; sim will halt anyway.
        return false;
    }

    int link_cost = g_adj[at][next_hop];
    Event* nxt = new Event();
    nxt->time = g_now + link_cost;
    nxt->type = ET_PACKET_ARRIVAL;
    nxt->to_id = next_hop;
    nxt->dst_id = dst;
    nxt->ttl_remaining = ttl_after_call;
    nxt->path_cost_so_far = accumulated_cost + link_cost;
    enqueue(nxt);
    return true;
}

void handle_scenario_link(Event* e) {
    int u = e->from_id, v = e->to_id;
    int new_cost = e->cost;   // raw scenario value: -1 means NO_LINK
    int current  = g_adj[u][v];
    int normalized = (new_cost == -1) ? NETSIM2_NO_LINK : new_cost;

    --g_pending_scenario_links;
    if (current == normalized) return;   // no actual change

    g_adj[u][v] = normalized;
    g_adj[v][u] = normalized;

    int report = (normalized == NETSIM2_NO_LINK) ? NETSIM2_NO_LINK : normalized;

    g_current_router = u;
    on_link_change(g_states[u], v, report);
    g_current_router = v;
    on_link_change(g_states[v], u, report);
    g_current_router = -1;
}

void handle_scenario_pkt(Event* e) {
    int src = e->from_id;
    int dst = e->dst_id;
    ++g_packets_total;

    if (src == dst) {
        // Auto-deliver, path cost 0.
        ++g_packets_delivered;
        --g_pending_packets;
        return;
    }

    int initial_ttl = 4 * g_num_nodes;
    // Make first forwarding call.
    int ttl_after = initial_ttl - 1;
    if (!forward_packet(src, dst, ttl_after, 0)) {
        // resolved (drop or protocol error)
        return;
    }
}

void handle_packet_arrival(Event* e) {
    int at = e->to_id;
    int dst = e->dst_id;

    if (at == dst) {
        ++g_packets_delivered;
        g_path_cost_total += e->path_cost_so_far;
        --g_pending_packets;
        return;
    }

    if (e->ttl_remaining == 0) {
        // No more forwarding allowed.
        fprintf(stderr, "netsim2: packet to %d dropped at node %d: TTL exhausted\n", dst, at);
        ++g_packets_dropped;
        --g_pending_packets;
        escalate_status(SIM_FAIL_DROPPED);
        return;
    }

    int ttl_after = e->ttl_remaining - 1;
    forward_packet(at, dst, ttl_after, e->path_cost_so_far);
}

void handle_control_arrival(Event* e) {
    int at   = e->to_id;
    int from = e->from_id;
    g_current_router = at;
    on_control(g_states[at], from, e->payload.data(), (int)e->payload.size());
    g_current_router = -1;
}

void handle_timer_wakeup(Event* e) {
    int at = e->to_id;
    g_current_router = at;
    on_timer(g_states[at]);
    g_current_router = -1;
}

void run_loop() {
    while (!g_events.empty()) {
        Event* e = g_events.top();
        g_events.pop();
        g_now = e->time;

        switch (e->type) {
        case ET_SCENARIO_LINK:    handle_scenario_link(e); break;
        case ET_SCENARIO_PKT:     handle_scenario_pkt(e); break;
        case ET_PACKET_ARRIVAL:   handle_packet_arrival(e); break;
        case ET_CONTROL_ARRIVAL:  handle_control_arrival(e); break;
        case ET_TIMER_WAKEUP:     handle_timer_wakeup(e); break;
        }
        delete e;

        if (g_status == SIM_PROTOCOL_ERROR) break;
        if (g_pending_scenario_links == 0 && g_pending_packets == 0) break;
    }
}

void cleanup_queue() {
    while (!g_events.empty()) {
        delete g_events.top();
        g_events.pop();
    }
}

void shutdown_routers() {
    for (int i = 0; i < g_num_nodes; ++i) {
        if (g_states[i]) router_shutdown(g_states[i]);
        g_states[i] = nullptr;
    }
}

const char* status_str(SimStatus s) {
    switch (s) {
    case SIM_SUCCESS:        return "SUCCESS";
    case SIM_FAIL_DROPPED:   return "FAIL_DROPPED";
    case SIM_PROTOCOL_ERROR: return "PROTOCOL_ERROR";
    }
    return "UNKNOWN";
}

void print_summary() {
    int64_t cost = g_path_cost_total + (int64_t)K_VALUE * g_control_bytes_total;
    fprintf(stderr, "=== netsim2 summary ===\n");
    fprintf(stderr, "status: %s\n", status_str(g_status));
    fprintf(stderr, "num_nodes: %d\n", g_num_nodes);
    fprintf(stderr, "sim_time: %lld\n", (long long)g_now);
    fprintf(stderr, "packets_total: %d\n", g_packets_total);
    fprintf(stderr, "packets_delivered: %d\n", g_packets_delivered);
    fprintf(stderr, "packets_dropped: %d\n", g_packets_dropped);
    fprintf(stderr, "path_cost_total: %lld\n", (long long)g_path_cost_total);
    fprintf(stderr, "control_bytes_total: %lld\n", (long long)g_control_bytes_total);
    fprintf(stderr, "K: %d\n", K_VALUE);
    fprintf(stderr, "cost: %lld\n", (long long)cost);
}

}  // anonymous namespace

// =================================================================
// Public API exposed to the student's router code.
// =================================================================

extern "C" {

void send_control(int neighbor, const uint8_t* payload, int len) {
    if (g_current_router < 0) {
        fprintf(stderr, "netsim2: send_control called outside any callback context\n");
        return;
    }
    int from = g_current_router;

    if (len < 1 || len > MAX_PAYLOAD) {
        fprintf(stderr, "netsim2: send_control: len=%d out of range [1, %d] (call ignored)\n",
                len, MAX_PAYLOAD);
        return;
    }
    if (!is_up_neighbor(from, neighbor)) {
        fprintf(stderr, "netsim2: send_control: node %d is not a current neighbor of %d "
                        "(call ignored)\n", neighbor, from);
        return;
    }
    if (payload == nullptr) {
        fprintf(stderr, "netsim2: send_control: payload is NULL (call ignored)\n");
        return;
    }

    int link_cost = g_adj[from][neighbor];
    g_control_bytes_total += len;

    Event* e = new Event();
    e->time    = g_now + link_cost;
    e->type    = ET_CONTROL_ARRIVAL;
    e->from_id = from;
    e->to_id   = neighbor;
    e->payload.assign(payload, payload + len);
    enqueue(e);
}

int64_t get_now(void) {
    return g_now;
}

void schedule_wakeup(int64_t when) {
    if (g_current_router < 0) {
        fprintf(stderr, "netsim2: schedule_wakeup called outside any callback context\n");
        return;
    }
    Event* e = new Event();
    e->time  = (when < g_now) ? g_now : when;
    e->type  = ET_TIMER_WAKEUP;
    e->to_id = g_current_router;
    enqueue(e);
}

}  // extern "C"

// =================================================================
// main()
// =================================================================

int main(int argc, char** argv) {
    const char* scenario_path = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario_path = argv[++i];
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            fprintf(stderr, "Usage: %s --scenario FILE\n", argv[0]);
            return 0;
        } else {
            fprintf(stderr, "%s: unknown argument '%s'\n", argv[0], argv[i]);
            fprintf(stderr, "Usage: %s --scenario FILE\n", argv[0]);
            return 1;
        }
    }
    if (!scenario_path) {
        fprintf(stderr, "Usage: %s --scenario FILE\n", argv[0]);
        return 1;
    }

    ScenarioFile sf;
    if (!parse_scenario(scenario_path, sf)) return 1;

    initialize_topology(sf);
    initialize_routers();
    enqueue_scenario(sf);

    run_loop();

    print_summary();

    cleanup_queue();
    shutdown_routers();
    return 0;
}
