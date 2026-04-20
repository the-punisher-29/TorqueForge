#include "demo_common.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace Eigen;
using namespace SPD;

// ─────────────────────────────────────────────────────────────────────────────
// Chain / pendulum geometry
// ─────────────────────────────────────────────────────────────────────────────

static const float LINK_HALF_Y    = 0.25f;   // half-length of each link along Y
static const float LINK_HALF_XZ   = 0.08f;   // half-thickness on the other two axes
static const float LINK_SPACING   = 2.0f * LINK_HALF_Y; // joint-to-joint distance
static const float LINK_DENSITY   = 8.0f;                // kg / m³ (uniform)
static const float ANCHOR_HALF_Y  = 0.10f;
static const float INITIAL_ANGLE  = 0.7f;    // rad — first joint is tilted off vertical so the chain swings
static const float JOINT_DAMPING  = 80.0f;   // Nm·s/rad per joint — empirically tuned so the chain
                                             // envelope decays to ~10 % of its initial amplitude by t ≈ 10 s
                                             // (see tune_chain_damping.cpp for the sweep).

struct Chain {
	std::shared_ptr<ArticulatedBody> art;
	std::vector<size_t> render_keys;         // one per body (0 = anchor, 1..N = links)
};

// Build an N-link revolute chain hung from a static anchor at `base_pos`.
// Renderer bodies are added and registered; the caller still owns adding
// the ArticulatedBody to the world.
static Chain build_chain(int N, Vector3f base_pos) {
	Chain ch;

	// ── anchor ───────────────────────────────────────────────────────────────
	RigidBody::Config anchor_cfg;
	anchor_cfg.name        = "anchor";
	anchor_cfg.shape       = std::make_shared<Cuboid>(Vector3f(0.25f, ANCHOR_HALF_Y, 0.25f));
	anchor_cfg.rotation    = Quaternionf::Identity();
	anchor_cfg.translation = base_pos;
	anchor_cfg.type        = RigidBody::DynamicType::Static;
	anchor_cfg.density     = 1.0f;
	RigidBody anchor(anchor_cfg);

	ch.art = std::make_shared<ArticulatedBody>(anchor);

	// ── links ────────────────────────────────────────────────────────────────
	Vector3f link_half(LINK_HALF_XZ, LINK_HALF_Y, LINK_HALF_XZ);
	for (int i = 0; i < N; ++i) {
		RigidBody::Config cfg;
		cfg.name        = "link_" + std::to_string(i);
		cfg.shape       = std::make_shared<Cuboid>(link_half);
		cfg.rotation    = Quaternionf::Identity();
		float cy        = base_pos.y() - ANCHOR_HALF_Y - LINK_HALF_Y - i * LINK_SPACING;
		cfg.translation = Vector3f(base_pos.x(), cy, base_pos.z());
		cfg.type        = RigidBody::DynamicType::Dynamic;
		cfg.density     = LINK_DENSITY;
		RigidBody rb(cfg);
		ch.art->add_body(rb);
	}

	// ── joints: anchor → link_0 → link_1 → ... → link_{N-1} ─────────────────
	// Revolute axis is the Z column of the joint basis; identity basis leaves it
	// aligned with world-Z, so the chain swings in the X–Y plane.
	Matrix3f joint_bases = Matrix3f::Identity();

	// k=0 (no spring) + d>0 (viscous damping) — energy is dissipated in joint-space,
	// so the chain settles toward the vertical equilibrium instead of swinging forever.
	std::vector<ArticulatedBody::SpringParam> sp = {{0.0f, JOINT_DAMPING}};

	ch.art->add_joint(
		"j0", ArticulatedBody::JointType::Revolute,
		/*id0=*/0, /*id1=*/1,
		joint_bases,
		/*trans0 in anchor frame:*/ Vector3f(0.0f, -ANCHOR_HALF_Y, 0.0f),
		/*disable_collision=*/true, sp);

	for (int i = 1; i < N; ++i) {
		ch.art->add_joint(
			"j" + std::to_string(i), ArticulatedBody::JointType::Revolute,
			/*id0=*/(size_t)i, /*id1=*/(size_t)(i + 1),
			joint_bases,
			/*trans0 (bottom of parent link):*/ Vector3f(0.0f, -LINK_HALF_Y, 0.0f),
			/*disable_collision=*/true, sp);
	}

	return ch;
}

// Register every body of the chain with the renderer. Call once after build_chain.
static void register_chain_with_renderer(Chain& ch) {
	auto make_cuboid_mesh = [](const Vector3f& half) {
		return g_renderer->build_mesh(RigidWorldRenderer::Shape::Cuboid, v3(half));
	};

	// anchor
	{
		Vector3f half(0.25f, ANCHOR_HALF_Y, 0.25f);
		Mesh collider  = make_cuboid_mesh(half);
		Mesh renderable = make_cuboid_mesh(half);
		size_t key = g_renderer->add_body(
			{collider}, renderable,
			v3(ch.art->bodies[0]->translation),
			q(ch.art->bodies[0]->rotation));
		ch.render_keys.push_back(key);
	}

	// links
	Vector3f link_half(LINK_HALF_XZ, LINK_HALF_Y, LINK_HALF_XZ);
	for (size_t i = 1; i < ch.art->bodies.size(); ++i) {
		Mesh collider   = make_cuboid_mesh(link_half);
		Mesh renderable = make_cuboid_mesh(link_half);
		size_t key = g_renderer->add_body(
			{collider}, renderable,
			v3(ch.art->bodies[i]->translation),
			q(ch.art->bodies[i]->rotation));
		ch.render_keys.push_back(key);
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// Scaling benchmark — stress-tests the O(n) vs O(n³) story
// ─────────────────────────────────────────────────────────────────────────────

// Steps a freshly built chain M times and returns mean wall-clock time per
// step, in microseconds. This measures the complete physics step: bias
// forces (RNEA, O(n)), composite inertia (CRBA, O(n²)), dense Cholesky
// factorize + solve (O(n³)) — so the overall cost is dominated by the
// O(n³) term as N grows.
static double benchmark_step_us(int N, int M_steps, float dt = 0.01f) {
	auto world = std::make_shared<RigidWorld>(Vector3f(0.0f, -10.0f, 0.0f));
	Chain ch   = build_chain(N, Vector3f::Zero());
	world->add_body(ch.art);

	// Small perturbation so dq≠0 without inducing self-collision at large N.
	// A pure vertical chain is an equilibrium: gravity does nothing and the
	// benchmark wouldn't exercise the dynamics.
	ch.art->tree_joints[1]->q(0) = 0.03f;
	ch.art->move_joints();
	ch.art->project_position();

	// warm-up (first step allocates / factorizes)
	world->step(dt);

	using clk = std::chrono::high_resolution_clock;
	auto t0 = clk::now();
	for (int s = 0; s < M_steps; ++s) {
		world->step(dt);
	}
	auto t1 = clk::now();
	double total_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
	return total_us / (double)M_steps;
}

static void print_scaling_table(const std::vector<int>& Ns, int M_steps) {
	std::cout << "\n";
	std::cout << "═════════════════════════════════════════════════════════════════════\n";
	std::cout << "  Chain-Pendulum Scaling Benchmark  (" << M_steps << " steps per N)\n";
	std::cout << "═════════════════════════════════════════════════════════════════════\n";
	std::cout << std::left  << std::setw(6)  << "  N"
	          << std::right << std::setw(14) << "step (µs)"
	          << std::right << std::setw(16) << "ratio vs N=" << Ns.front()
	          << std::right << std::setw(14) << "O(n³) pred"
	          << std::right << std::setw(12) << "O(n) pred"
	          << "\n";
	std::cout << "─────────────────────────────────────────────────────────────────────\n";

	double t_base = benchmark_step_us(Ns.front(), M_steps);
	int    N_base = Ns.front();

	std::cout << std::left  << std::setw(6)  << ("  " + std::to_string(N_base))
	          << std::right << std::setw(14) << std::fixed << std::setprecision(1) << t_base
	          << std::right << std::setw(16) << "1.00x"
	          << std::right << std::setw(14) << "1.00x"
	          << std::right << std::setw(12) << "1.00x"
	          << "\n";

	for (size_t i = 1; i < Ns.size(); ++i) {
		int N = Ns[i];
		double t = benchmark_step_us(N, M_steps);
		double ratio = t / t_base;
		double cubic = std::pow((double)N / N_base, 3.0);
		double linear = (double)N / N_base;
		std::cout << std::left  << std::setw(6)  << ("  " + std::to_string(N))
		          << std::right << std::setw(14) << std::fixed << std::setprecision(1) << t
		          << std::right << std::setw(15) << std::fixed << std::setprecision(2) << ratio << "x"
		          << std::right << std::setw(13) << std::fixed << std::setprecision(2) << cubic << "x"
		          << std::right << std::setw(11) << std::fixed << std::setprecision(2) << linear << "x"
		          << "\n";
	}
	std::cout << "─────────────────────────────────────────────────────────────────────\n";
	std::cout << "  The dynamics solver uses CRBA (O(n²) build H) + dense Cholesky\n";
	std::cout << "  (O(n³) factorize). At small-to-moderate N, measured times sit\n";
	std::cout << "  close to the O(n) column because broadphase / per-step fixed\n";
	std::cout << "  overhead dominates. As N grows, the gap to the O(n³) column\n";
	std::cout << "  closes and the cubic term takes over — a true Featherstone ABA\n";
	std::cout << "  pass would keep tracking the O(n) column instead.\n";
	std::cout << "═════════════════════════════════════════════════════════════════════\n\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
	// Parse the live-chain size from CLI, default N=30.
	int N_live = 30;
	if (argc >= 2) {
		try {
			N_live = std::max(2, std::min(120, std::stoi(argv[1])));
		} catch (...) { N_live = 30; }
	}

	// ── 1. Scaling sweep (terminal) ──────────────────────────────────────────
	std::cout << "Running scaling benchmark — this takes a few seconds…\n";
	print_scaling_table({5, 10, 20, 30, 40, 50, 60}, /*M_steps=*/200);

	// ── 2. Live simulation ───────────────────────────────────────────────────
	std::cout << "Launching live chain (N = " << N_live << " links).\n";
	std::cout << "  SPACE  — kick the top link\n";
	std::cout << "  R      — reset to initial angle\n";
	std::cout << "  ESC/Q  — quit\n\n";

	init_renderer();
	init_world();

	Vector3f base_pos(0.0f, 5.0f, 0.0f);
	Chain ch = build_chain(N_live, base_pos);
	register_chain_with_renderer(ch);
	g_world->add_body(ch.art);

	// Perturb initial angle and propagate body world transforms
	ch.art->tree_joints[1]->q(0) = INITIAL_ANGLE;
	ch.art->move_joints();
	ch.art->project_position();

	// Running timing stats
	double ema_step_us = 0.0;
	const double ema_alpha = 0.05;
	size_t hud_print_interval = 60; // print every ~1s at 60 fps

	auto update_world = [&](float frame_dt, size_t frame_id) {
		// Interactive kick
		if (IsKeyPressed(KEY_SPACE)) {
			ch.art->tree_joints[1]->taue(0) += 50.0f; // impulse-ish: applied for one frame
			std::cout << "[kick] top-link torque impulse\n";
		}
		if (IsKeyPressed(KEY_R)) {
			for (auto& j : ch.art->tree_joints) {
				if (j) { j->q.setZero(); j->dq.setZero(); j->ddq.setZero(); }
			}
			ch.art->tree_joints[1]->q(0) = INITIAL_ANGLE;
			ch.art->move_joints();
			ch.art->project_position();
			std::cout << "[reset]\n";
		}

		using clk = std::chrono::high_resolution_clock;
		auto t0 = clk::now();
		g_world->step(0.016667f);
		auto t1 = clk::now();
		double step_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
		ema_step_us = (ema_step_us == 0.0) ? step_us : (ema_alpha * step_us + (1.0 - ema_alpha) * ema_step_us);

		// Clear one-shot torque so the kick doesn't stick
		ch.art->tree_joints[1]->taue(0) = 0.0f;

		// Render-state sync
		for (size_t i = 0; i < ch.art->bodies.size(); ++i) {
			g_renderer->update_body(
				ch.render_keys[i],
				q(ch.art->bodies[i]->rotation),
				v3(ch.art->bodies[i]->translation));
		}

		if (frame_id % hud_print_interval == 0) {
			std::cout << "[live] N=" << N_live
			          << "   step=" << std::fixed << std::setprecision(1) << ema_step_us << " µs"
			          << "   (" << std::setprecision(2) << (ema_step_us / 1000.0) << " ms, "
			          << std::setprecision(0) << (1e6 / std::max(1.0, ema_step_us)) << " steps/s max)"
			          << "\n";
		}
	};

	auto debug_draw = [&](float frame_dt, size_t frame_id) {
		// Draw joint frames so the kinematic chain is visible
		for (size_t i = 1; i < ch.art->tree_joints.size(); ++i) {
			auto j = ch.art->tree_joints[i];
			g_renderer->draw_bases(
				v3(j->b0->translation + j->b0->bases * j->bt0),
				m3(j->b0->bases * j->bb0),
				0.15f);
		}
	};

	RigidWorldRenderer::Options opts;
	opts.movable_light = false;
	g_renderer->run(update_world, debug_draw, opts);

	return 0;
}
