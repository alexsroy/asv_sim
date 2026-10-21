#pragma once
// Multi-target tracker: one KalmanTrack per AIS identity, with global nearest-
// neighbour association for anonymous radar contacts.
//
// AIS (id != -1): hard match by identity. A new id starts a new track.
// Radar (id == -1): Mahalanobis gating via KalmanTrack::gate_distance2, then
// one-to-one greedy GNN on those costs so two tracks never claim the same hit.
// Unassigned radar is treated as clutter.
//
// Does not read Contact::truth_id.

#include <cstddef>
#include <utility>
#include <vector>

#include "asv/contact.hpp"
#include "asv/kalman_track.hpp"

namespace asv {

struct TrackManagerParams {
    // Chi-square gate on squared Mahalanobis distance (2 dof). 9.21 = 99%.
    double radar_gate2 = 9.21;

    // Drop a track that has not been updated for this long [s].
    double coast_timeout = 60.0;

    KalmanParams kalman;
};

// One live track: AIS identity plus its Kalman filter.
struct ManagedTrack {
    int id = -1;  // AIS identity (never -1 once created)
    KalmanTrack filter;
    double last_update_t = 0.0;  // sim time of last accepted contact delivery
};

struct AssociationStats {
    int ais_updates = 0;
    int ais_births = 0;
    int radar_assigned = 0;
    int radar_rejected = 0;  // gated out or lost the GNN race
};

class TrackManager {
public:
    explicit TrackManager(TrackManagerParams params = {});

    // Fold in every contact that became available at (or before) t_now.
    // AIS is applied first (birth + id update), then radar is associated as a
    // batch against the resulting track set.
    AssociationStats process(double t_now, const std::vector<Contact>& contacts);

    // Drop tracks whose last update is older than coast_timeout.
    void prune(double t_now);

    const std::vector<ManagedTrack>& tracks() const { return tracks_; }
    std::size_t size() const { return tracks_.size(); }

    // Estimates extrapolated to time t, one per live track (same order as tracks()).
    std::vector<std::pair<int, TrackEstimate>> estimates_at(double t) const;

    // Find by AIS id; nullptr if absent.
    ManagedTrack* find(int id);
    const ManagedTrack* find(int id) const;

    const TrackManagerParams& params() const { return p_; }
    const AssociationStats& totals() const { return totals_; }

private:
    void process_ais(double t_now, const Contact& c, AssociationStats& stats);
    void process_radar_batch(double t_now, const std::vector<Contact>& radar,
                             AssociationStats& stats);

    TrackManagerParams p_;
    std::vector<ManagedTrack> tracks_;
    AssociationStats totals_;
};

}  // namespace asv
