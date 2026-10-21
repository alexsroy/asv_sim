#include "asv/track_manager.hpp"

#include <algorithm>
#include <utility>

namespace asv {

TrackManager::TrackManager(TrackManagerParams params) : p_(params) {}

ManagedTrack* TrackManager::find(int id) {
    for (auto& t : tracks_) {
        if (t.id == id) return &t;
    }
    return nullptr;
}

const ManagedTrack* TrackManager::find(int id) const {
    for (const auto& t : tracks_) {
        if (t.id == id) return &t;
    }
    return nullptr;
}

AssociationStats TrackManager::process(double t_now,
                                       const std::vector<Contact>& contacts) {
    AssociationStats stats;
    std::vector<Contact> radar;
    radar.reserve(contacts.size());

    for (const Contact& c : contacts) {
        if (c.id != -1) {
            process_ais(t_now, c, stats);
        } else {
            radar.push_back(c);
        }
    }
    if (!radar.empty()) process_radar_batch(t_now, radar, stats);

    totals_.ais_updates += stats.ais_updates;
    totals_.ais_births += stats.ais_births;
    totals_.radar_assigned += stats.radar_assigned;
    totals_.radar_rejected += stats.radar_rejected;
    return stats;
}

void TrackManager::process_ais(double t_now, const Contact& c,
                               AssociationStats& stats) {
    if (ManagedTrack* existing = find(c.id)) {
        if (existing->filter.update(c)) {
            existing->last_update_t = t_now;
            ++stats.ais_updates;
        }
        return;
    }

    tracks_.push_back(
        ManagedTrack{c.id, KalmanTrack(c, p_.kalman), t_now});
    ++stats.ais_births;
}

void TrackManager::process_radar_batch(double t_now,
                                       const std::vector<Contact>& radar,
                                       AssociationStats& stats) {
    // Candidate edges (track_index, radar_index, cost), gated then sorted.
    struct Edge {
        std::size_t track_i;
        std::size_t radar_i;
        double cost;
    };
    std::vector<Edge> edges;
    edges.reserve(tracks_.size() * radar.size());

    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
        for (std::size_t ri = 0; ri < radar.size(); ++ri) {
            const double d2 = tracks_[ti].filter.gate_distance2(radar[ri]);
            if (d2 < p_.radar_gate2) {
                edges.push_back({ti, ri, d2});
            }
        }
    }

    std::sort(edges.begin(), edges.end(),
              [](const Edge& a, const Edge& b) { return a.cost < b.cost; });

    std::vector<char> track_taken(tracks_.size(), 0);
    std::vector<char> radar_taken(radar.size(), 0);

    for (const Edge& e : edges) {
        if (track_taken[e.track_i] || radar_taken[e.radar_i]) continue;
        if (tracks_[e.track_i].filter.update(radar[e.radar_i])) {
            tracks_[e.track_i].last_update_t = t_now;
            ++stats.radar_assigned;
        }
        track_taken[e.track_i] = 1;
        radar_taken[e.radar_i] = 1;
    }

    for (std::size_t ri = 0; ri < radar.size(); ++ri) {
        if (!radar_taken[ri]) ++stats.radar_rejected;
    }
}

void TrackManager::prune(double t_now) {
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const ManagedTrack& t) {
                                     return (t_now - t.last_update_t) >
                                            p_.coast_timeout;
                                 }),
                  tracks_.end());
}

std::vector<std::pair<int, TrackEstimate>> TrackManager::estimates_at(
    double t) const {
    std::vector<std::pair<int, TrackEstimate>> out;
    out.reserve(tracks_.size());
    for (const auto& tr : tracks_) {
        out.emplace_back(tr.id, tr.filter.estimate_at(t));
    }
    return out;
}

}  // namespace asv
