#include "mlat_aux.hpp"

#include <cfloat>
#include <cmath>
#include <utility>

using namespace time_literals;

namespace
{

// solve cadence and beacon freshness
constexpr hrt_abstime kUpdateInterval = 500_ms;
constexpr hrt_abstime kMaxBeaconAge = 1_s;

// geometry gate
constexpr float kMaxHdop = 5.0f;

// MLAT solver (ported from lion::filters::MLAT / ArduPilot AP_NavEKF3_RngBcnFusion SolveMlat)
constexpr int kNumSeeds = 5;
constexpr float kLocalAnchorOffset = 1000.f;           // m
constexpr float kBasinRadius2 = 50.f * 50.f;           // m^2
constexpr float kResidualEpsilon = 10.f;               // m^2
constexpr float kMinBeaconSeparation2 = 100.f * 100.f; // m^2
constexpr float kMaxResidual2 = 255.f * 255.f;         // m^2
constexpr float kLearningRate = 0.1f;
constexpr int kMaxIterations = 30;
constexpr float kTolerance = 1.0f; // m

// AGP publication: matches EKF2_AGP0_ID in the 10046_navput_quadx airframe, i.e. the
// already-enabled slot 0 (Navputer::Navputer() hardcodes _fc.agp[0].enabled = true).
constexpr uint8_t kAgpId = 111;

struct BeaconInput {
	matrix::Vector2f pos; // local north/east, m
	float range{0.f};     // flattened 2D ground range, m
	float range_accuracy{0.f};
	uint64_t timestamp_sample{0};
};

struct Candidate {
	bool valid{false};
	matrix::Vector2f anchor;
	matrix::Vector2f pos;
	float residual{1e10f};
	uint8_t votes{0};
};

float squaredNorm(const matrix::Vector2f &v)
{
	return v(0) * v(0) + v(1) * v(1);
}

// counts geometrically distinct beacons: beacons closer than kMinBeaconSeparation2 to an
// already-counted one add no independent geometry and are folded together
int countDistinctBeacons(const BeaconInput *inputs, int num_inputs)
{
	bool counted[MlatAux::kMaxBeacons] {};
	int distinct = 0;

	for (int i = 0; i < num_inputs; i++) {
		if (counted[i]) {
			continue;
		}

		counted[i] = true;
		distinct++;

		for (int j = i + 1; j < num_inputs; j++) {
			if (!counted[j] && squaredNorm(inputs[i].pos - inputs[j].pos) < kMinBeaconSeparation2) {
				counted[j] = true;
			}
		}
	}

	return distinct;
}

// standard 2-D HDOP from the Fisher information matrix of unit line-of-sight vectors
bool calcHdop(const matrix::Vector2f &from, const BeaconInput *inputs, int num_inputs, float &hdop)
{
	float hxx = 0.f;
	float hyy = 0.f;
	float hxy = 0.f;
	int valid = 0;

	for (int i = 0; i < num_inputs; i++) {
		const matrix::Vector2f delta = inputs[i].pos - from;
		const float norm = delta.norm();

		if (norm < 0.1f) {
			continue;
		}

		const float ux = delta(0) / norm;
		const float uy = delta(1) / norm;
		hxx += ux * ux;
		hyy += uy * uy;
		hxy += ux * uy;
		valid++;
	}

	if (valid < 3) {
		return false;
	}

	const float det = hxx * hyy - hxy * hxy;

	if (det < 1e-6f) {
		return false;
	}

	hdop = sqrtf((hxx + hyy) / det);
	return true;
}

void solveCandidate(Candidate &candidate, const BeaconInput *inputs, int num_inputs)
{
	matrix::Vector2f solution = candidate.anchor;
	float residual = 1e10f;

	for (int iter = 0; iter < kMaxIterations; iter++) {
		float gx = 0.f;
		float gy = 0.f;
		float sum_sq_err = 0.f;
		int used = 0;

		for (int i = 0; i < num_inputs; i++) {
			const matrix::Vector2f delta = solution - inputs[i].pos;
			const float estimated = delta.norm();

			if (estimated > 0.f) {
				const float err = estimated - inputs[i].range;
				const float err_deriv = 2.f * err;
				gx += err_deriv * delta(0) / estimated;
				gy += err_deriv * delta(1) / estimated;
				sum_sq_err += err * err;
				used++;
			}
		}

		if (used == 0) {
			break;
		}

		residual = sum_sq_err / used;

		const matrix::Vector2f next(solution(0) - kLearningRate * gx, solution(1) - kLearningRate * gy);
		const bool converged = fabsf(next(0) - solution(0)) < kTolerance && fabsf(next(1) - solution(1)) < kTolerance;
		solution = next;

		if (converged) {
			break;
		}
	}

	candidate.residual = residual;

	if (residual > kMaxResidual2) {
		candidate.valid = false;
		candidate.votes = 0;
		return;
	}

	candidate.pos = solution;
	candidate.valid = true;
	candidate.votes = 1;
}

void buildLocalAnchors(Candidate (&candidates)[kNumSeeds], const matrix::Vector2f &last_pos)
{
	candidates[0].anchor = last_pos;
	candidates[1].anchor = last_pos + matrix::Vector2f(kLocalAnchorOffset, 0.f); // N
	candidates[2].anchor = last_pos + matrix::Vector2f(0.f, kLocalAnchorOffset); // E
	candidates[3].anchor = last_pos + matrix::Vector2f(-kLocalAnchorOffset, 0.f); // S
	candidates[4].anchor = last_pos + matrix::Vector2f(0.f, -kLocalAnchorOffset); // W
}

void buildGlobalAnchors(Candidate (&candidates)[kNumSeeds], const BeaconInput *inputs, int num_inputs)
{
	float min_n = FLT_MAX;
	float max_n = -FLT_MAX;
	float min_e = FLT_MAX;
	float max_e = -FLT_MAX;

	for (int i = 0; i < num_inputs; i++) {
		const float n = inputs[i].pos(0);
		const float e = inputs[i].pos(1);
		const float r = inputs[i].range;
		min_n = fminf(min_n, n - r);
		max_n = fmaxf(max_n, n + r);
		min_e = fminf(min_e, e - r);
		max_e = fmaxf(max_e, e + r);
	}

	candidates[0].anchor = matrix::Vector2f(min_n, min_e);
	candidates[1].anchor = matrix::Vector2f(min_n, max_e);
	candidates[2].anchor = matrix::Vector2f(max_n, min_e);
	candidates[3].anchor = matrix::Vector2f(max_n, max_e);
	candidates[4].anchor = matrix::Vector2f((min_n + max_n) * 0.5f, (min_e + max_e) * 0.5f);
}

// merges candidates that converged within kBasinRadius2 of each other, keeping the
// lower-residual member as representative and summing votes (ported from MLAT::evaluate())
void mergeBasins(Candidate (&candidates)[kNumSeeds])
{
	for (int i = 0; i < kNumSeeds; i++) {
		if (!candidates[i].valid) {
			continue;
		}

		for (int j = 0; j < kNumSeeds; j++) {
			if (i == j || !candidates[j].valid) {
				continue;
			}

			if (squaredNorm(candidates[i].pos - candidates[j].pos) < kBasinRadius2) {
				const uint8_t votes = candidates[i].votes + candidates[j].votes;

				if (candidates[i].residual < candidates[j].residual) {
					candidates[j] = candidates[i];
				}

				candidates[j].votes = votes;
				candidates[i].valid = false;
				break;
			}
		}
	}
}

// picks the accepted solution among the merged basins, rejecting ambiguous cold-start fixes
bool evaluate(Candidate (&candidates)[kNumSeeds], bool have_last_pos, const matrix::Vector2f &last_pos,
	      matrix::Vector2f &result)
{
	mergeBasins(candidates);

	int best = -1;
	int second = -1;

	for (int i = 0; i < kNumSeeds; i++) {
		if (!candidates[i].valid) {
			continue;
		}

		if (best < 0 || candidates[i].residual < candidates[best].residual) {
			second = best;
			best = i;

		} else if (second < 0 || candidates[i].residual < candidates[second].residual) {
			second = i;
		}
	}

	if (best < 0) {
		return false;
	}

	if (second < 0) {
		result = candidates[best].pos;
		return true;
	}

	if (candidates[second].votes > candidates[best].votes + 1) {
		std::swap(best, second);
	}

	if (fabsf(candidates[best].residual - candidates[second].residual) <= kResidualEpsilon) {
		if (!have_last_pos) {
			return false; // ambiguous cold-start fix, reject rather than guess
		}

		const float dist_best = squaredNorm(candidates[best].pos - last_pos);
		const float dist_second = squaredNorm(candidates[second].pos - last_pos);
		result = (dist_second < dist_best) ? candidates[second].pos : candidates[best].pos;
		return true;
	}

	result = candidates[best].pos;
	return true;
}

} // namespace

void MlatAux::updateBeaconStore()
{
	ranging_beacon_s beacon;

	if (!_ranging_beacon_sub.update(&beacon)) {
		return;
	}

	int slot = -1;
	int oldest = 0;

	for (int i = 0; i < kMaxBeacons; i++) {
		if (_beacons[i].valid && _beacons[i].beacon_id == beacon.beacon_id) {
			slot = i;
			break;
		}

		if (!_beacons[i].valid && slot < 0) {
			slot = i;
		}

		if (_beacons[i].timestamp_sample < _beacons[oldest].timestamp_sample) {
			oldest = i;
		}
	}

	if (slot < 0) {
		slot = oldest; // table full and beacon_id unseen: evict the oldest entry
	}

	_beacons[slot].valid = true;
	_beacons[slot].beacon_id = beacon.beacon_id;
	_beacons[slot].lat = beacon.lat;
	_beacons[slot].lon = beacon.lon;
	_beacons[slot].alt = beacon.alt;
	_beacons[slot].range = beacon.range;
	_beacons[slot].range_accuracy = beacon.range_accuracy;
	_beacons[slot].timestamp_sample = beacon.timestamp_sample;
}

bool MlatAux::solve()
{
	navput_local_position_s local_pos;

	if (!_local_position_sub.copy(&local_pos)) {
		return false;
	}

	if (!_projection.isInitialized()) {
		if (local_pos.xy_global) {
			_projection.initReference(local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp);

		} else {
			return false;
		}
	}

	const bool have_last_pos = local_pos.xy_valid;
	const matrix::Vector2f last_pos(local_pos.x, local_pos.y);
	const bool have_own_alt = local_pos.z_global;
	const float own_alt = local_pos.ref_alt - local_pos.z;

	const hrt_abstime now = hrt_absolute_time();

	BeaconInput inputs[kMaxBeacons];
	int num_inputs = 0;
	uint64_t latest_timestamp_sample = 0;

	for (int i = 0; i < kMaxBeacons; i++) {
		const BeaconEntry &beacon = _beacons[i];

		if (!beacon.valid || (now - beacon.timestamp_sample) > kMaxBeaconAge) {
			continue;
		}

		BeaconInput &input = inputs[num_inputs];
		float x;
		float y;
		_projection.project(beacon.lat, beacon.lon, x, y);
		input.pos = matrix::Vector2f(x, y);

		float range = beacon.range;

		if (have_own_alt) {
			const float dz = own_alt - beacon.alt;

			if (fabsf(dz) < beacon.range) {
				range = sqrtf(beacon.range * beacon.range - dz * dz);
			}
		}

		input.range = range;
		input.range_accuracy = beacon.range_accuracy;
		input.timestamp_sample = beacon.timestamp_sample;
		latest_timestamp_sample = math::max(latest_timestamp_sample, beacon.timestamp_sample);
		num_inputs++;
	}

	const int min_required = have_last_pos ? 2 : 3;

	if (countDistinctBeacons(inputs, num_inputs) < min_required) {
		return false;
	}

	const matrix::Vector2f best_guess = have_last_pos ? last_pos : (inputs[0].pos + inputs[num_inputs - 1].pos) * 0.5f;

	float hdop = kMaxHdop;

	if (!calcHdop(best_guess, inputs, num_inputs, hdop) || hdop > kMaxHdop) {
		return false;
	}

	Candidate candidates[kNumSeeds];

	if (have_last_pos) {
		buildLocalAnchors(candidates, last_pos);

	} else {
		buildGlobalAnchors(candidates, inputs, num_inputs);
	}

	for (int i = 0; i < kNumSeeds; i++) {
		solveCandidate(candidates[i], inputs, num_inputs);
	}

	matrix::Vector2f solution;

	if (!evaluate(candidates, have_last_pos, last_pos, solution)) {
		return false;
	}

	float solution_hdop = kMaxHdop;
	calcHdop(solution, inputs, num_inputs, solution_hdop);

	float mean_range_accuracy = 0.f;

	for (int i = 0; i < num_inputs; i++) {
		mean_range_accuracy += inputs[i].range_accuracy;
	}

	mean_range_accuracy /= num_inputs;

	const float hdop_factor = solution_hdop <= 3.0f ? 1.0f : fminf(10.0f, 1.0f + (solution_hdop - 3.0f) / 3.0f);

	double lat;
	double lon;
	_projection.reproject(solution(0), solution(1), lat, lon);

	aux_global_position_s agp{};
	agp.timestamp_sample = latest_timestamp_sample;
	agp.id = kAgpId;
	agp.source = aux_global_position_s::SOURCE_PSEUDOLITES;
	agp.lat = lat;
	agp.lon = lon;
	agp.alt = have_own_alt ? own_alt : NAN;
	agp.eph = mean_range_accuracy * hdop_factor;
	agp.epv = have_own_alt ? local_pos.epv : NAN;
	agp.lat_lon_reset_counter = 0;
	agp.timestamp = hrt_absolute_time();
	_aux_global_position_pub.publish(agp);

	return true;
}

void MlatAux::update()
{
	updateBeaconStore();

	if (hrt_elapsed_time(&_last_run) < kUpdateInterval) {
		return;
	}

	_last_run = hrt_absolute_time();

	solve();
}
