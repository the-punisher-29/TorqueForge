// Headless damping tuner — builds the same 30-link chain as the live demo,
// sweeps over candidate JOINT_DAMPING values, and reports max|dq| at each
// second of simulated time so we can pick a value that settles in ~10 s.

#include "articulatedbody.hpp"
#include "rigidbody.hpp"
#include "rigidworld.h"
#include "spshapes.hpp"

#include <iomanip>
#include <iostream>

using namespace Eigen;
using namespace SPD;

static const float LINK_HALF_Y   = 0.25f;
static const float LINK_HALF_XZ  = 0.08f;
static const float LINK_SPACING  = 2.0f * LINK_HALF_Y;
static const float LINK_DENSITY  = 8.0f;
static const float ANCHOR_HALF_Y = 0.10f;
static const float INITIAL_ANGLE = 0.7f;

static std::shared_ptr<ArticulatedBody> build_chain(int N, float d) {
	RigidBody::Config anchor_cfg;
	anchor_cfg.name        = "anchor";
	anchor_cfg.shape       = std::make_shared<Cuboid>(Vector3f(0.25f, ANCHOR_HALF_Y, 0.25f));
	anchor_cfg.translation = Vector3f::Zero();
	anchor_cfg.type        = RigidBody::DynamicType::Static;
	anchor_cfg.density     = 1.0f;
	RigidBody anchor(anchor_cfg);

	auto art = std::make_shared<ArticulatedBody>(anchor);

	Vector3f link_half(LINK_HALF_XZ, LINK_HALF_Y, LINK_HALF_XZ);
	for (int i = 0; i < N; ++i) {
		RigidBody::Config cfg;
		cfg.name        = "link_" + std::to_string(i);
		cfg.shape       = std::make_shared<Cuboid>(link_half);
		float cy        = -ANCHOR_HALF_Y - LINK_HALF_Y - i * LINK_SPACING;
		cfg.translation = Vector3f(0.0f, cy, 0.0f);
		cfg.type        = RigidBody::DynamicType::Dynamic;
		cfg.density     = LINK_DENSITY;
		RigidBody rb(cfg);
		art->add_body(rb);
	}

	Matrix3f joint_bases = Matrix3f::Identity();
	std::vector<ArticulatedBody::SpringParam> sp = {{0.0f, d}};

	art->add_joint("j0", ArticulatedBody::JointType::Revolute,
	               0, 1, joint_bases, Vector3f(0.0f, -ANCHOR_HALF_Y, 0.0f), true, sp);
	for (int i = 1; i < N; ++i) {
		art->add_joint("j" + std::to_string(i), ArticulatedBody::JointType::Revolute,
		               (size_t)i, (size_t)(i + 1), joint_bases,
		               Vector3f(0.0f, -LINK_HALF_Y, 0.0f), true, sp);
	}
	return art;
}

static float max_abs_dq(const std::shared_ptr<ArticulatedBody>& art) {
	float m = 0.0f;
	for (size_t i = 1; i < art->tree_joints.size(); ++i) {
		m = std::max(m, std::abs(art->tree_joints[i]->dq(0)));
	}
	return m;
}

static float abs_q_top(const std::shared_ptr<ArticulatedBody>& art) {
	return std::abs(art->tree_joints[1]->q(0));
}

int main() {
	const int N = 30;
	const float dt = 1.0f / 60.0f;
	const int sim_seconds = 12;
	const int steps_per_sec = (int)(1.0f / dt);

	std::vector<float> damping_values = {50.0f, 80.0f, 120.0f, 180.0f};

	std::cout << std::left << std::setw(10) << "t (s)";
	for (float d : damping_values) {
		std::ostringstream tag;
		tag << "d=" << d;
		std::cout << std::right << std::setw(12) << tag.str();
	}
	std::cout << "\n";
	std::cout << std::string(10 + 12 * damping_values.size(), '-') << "\n";

	// Build one chain per damping value, step them in parallel, log per-second.
	struct Run {
		float d;
		std::shared_ptr<RigidWorld> world;
		std::shared_ptr<ArticulatedBody> art;
	};
	std::vector<Run> runs;
	for (float d : damping_values) {
		Run r;
		r.d     = d;
		r.world = std::make_shared<RigidWorld>(Vector3f(0.0f, -10.0f, 0.0f));
		r.art   = build_chain(N, d);
		r.world->add_body(r.art);
		r.art->tree_joints[1]->q(0) = INITIAL_ANGLE;
		r.art->move_joints();
		r.art->project_position();
		runs.push_back(r);
	}

	// For each second-window, track the PEAK max|dq| observed during that second
	// so we see the envelope of the motion, not a lucky zero-crossing.
	std::vector<float> peak_this_window(runs.size(), 0.0f);

	std::cout << "  (per second: peak |dq| across all joints  |  peak |q_top|)\n";
	std::cout << std::left << std::setw(10) << "0 (start)";
	for (const Run& r : runs) {
		std::ostringstream cell;
		cell << std::fixed << std::setprecision(2) << max_abs_dq(r.art)
		     << "/" << std::setprecision(2) << abs_q_top(r.art);
		std::cout << std::right << std::setw(14) << cell.str();
	}
	std::cout << "\n";

	std::vector<float> peak_q_window(runs.size(), 0.0f);
	for (int s = 1; s <= sim_seconds; ++s) {
		std::fill(peak_this_window.begin(), peak_this_window.end(), 0.0f);
		std::fill(peak_q_window.begin(),    peak_q_window.end(),    0.0f);
		for (int k = 0; k < steps_per_sec; ++k) {
			for (size_t i = 0; i < runs.size(); ++i) {
				runs[i].world->step(dt);
				peak_this_window[i] = std::max(peak_this_window[i], max_abs_dq(runs[i].art));
				peak_q_window[i]    = std::max(peak_q_window[i],    abs_q_top(runs[i].art));
			}
		}
		std::cout << std::left << std::setw(10) << s;
		for (size_t i = 0; i < runs.size(); ++i) {
			std::ostringstream cell;
			cell << std::fixed << std::setprecision(2) << peak_this_window[i]
			     << "/" << std::setprecision(2) << peak_q_window[i];
			std::cout << std::right << std::setw(14) << cell.str();
		}
		std::cout << "\n";
	}
	return 0;
}
