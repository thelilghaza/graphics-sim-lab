#ifndef DESTRUCTION_SANDBOX_APP_HPP
#define DESTRUCTION_SANDBOX_APP_HPP

#include "destruction/render/gl_loader.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include "destruction/render/camera.hpp"
#include "destruction/render/renderer.hpp"
#include "destruction/dynamics/physics_world.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/fracture/voronoi3d.hpp"
#include "destruction/fracture/site_generator.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/solver/solver_settings.hpp"
#include "destruction/graph/material_params.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>

namespace destruction::render {

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;
using namespace destruction::fracture;
using namespace destruction::solver;
using namespace destruction::graph;

struct Telemetry {
    float fps{0.0f};
    float physics_time_ms{0.0f};
    size_t body_count{0};
    size_t shard_count{0};
    size_t contact_count{0};
    size_t broadphase_pairs{0};
    int velocity_iterations{10};
    size_t unsupported_shards{0};
    size_t broken_edges{0};
    float max_penetration{0.0f};
    float total_kinetic_energy{0.0f};
};

class SandboxApp {
private:
    GLFWwindow* window_{nullptr};
    int window_width_{1280};
    int window_height_{720};

    PhysicsWorld world_;
    Camera camera_;
    Renderer renderer_;
    RenderFlags render_flags_;
    Telemetry telemetry_;

    float accumulator_{0.0f};
    const float fixed_dt_{1.0f / 60.0f};
    bool paused_{false};
    bool single_step_requested_{false};

    uint32_t next_body_id_{1};
    uint32_t next_projectile_id_{1000};
    std::vector<uint32_t> fracture_targets_;

    double last_frame_time_{0.0};
    double last_telemetry_print_{0.0};
    int frame_count_{0};

    bool headless_mode_{false};
    int max_frames_{-1};

public:
    SandboxApp()
        : camera_(Vec3(0.0f, 4.5f, 10.0f), -90.0f, -15.0f) {}

    ~SandboxApp() {
        if (window_) {
            glfwDestroyWindow(window_);
            glfwTerminate();
            window_ = nullptr;
        }
    }

    void set_headless(bool headless, int max_frames = 120) {
        headless_mode_ = headless;
        max_frames_ = max_frames;
    }

    bool init() {
        if (!glfwInit()) {
            std::cerr << "[GLFW Error] Failed to initialize GLFW.\n";
            return false;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        if (headless_mode_) {
            glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        }

        window_ = glfwCreateWindow(window_width_, window_height_,
            "Project 04 — Procedural Destruction Sandbox", nullptr, nullptr);
        if (!window_) {
            std::cerr << "[GLFW Error] Failed to create OpenGL 3.3 window.\n";
            glfwTerminate();
            return false;
        }

        glfwMakeContextCurrent(window_);

        if (!init_gl_loader(reinterpret_cast<GLProcLoader>(glfwGetProcAddress))) {
            std::cerr << "[OpenGL Error] Failed to load OpenGL functions.\n";
            return false;
        }

        glfwSetWindowUserPointer(window_, this);

        if (!headless_mode_) {
            glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            glfwSetCursorPosCallback(window_, mouse_callback_dispatch);
            glfwSetScrollCallback(window_, scroll_callback_dispatch);
            glfwSetFramebufferSizeCallback(window_, resize_callback_dispatch);
        }

        if (!renderer_.init()) {
            return false;
        }

        reset_scene();
        return true;
    }

    void reset_scene() {
        world_.clear();
        renderer_.clear_cache();
        next_body_id_ = 1;
        fracture_targets_.clear();

        // 1. Configure Physics World & Solver Settings
        world_.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

        SolverSettings settings;
        settings.velocity_iterations = 10;
        settings.position_iterations = 5;
        settings.default_friction = 0.4f;
        settings.default_restitution = 0.15f;
        settings.penetration_slop = 0.005f;
        settings.baumgarte_beta = 0.25f;
        world_.set_solver_settings(settings);

        MaterialParams mat;
        mat.density = 2400.0f;
        mat.tensile_strength = 4.0e4f;
        mat.capacity_multiplier = 4.0f;
        world_.get_graph().set_material_params(mat);

        // 2. Ground Plane (Static Body)
        uint32_t ground_id = next_body_id_++;
        RigidBody ground = RigidBody::create_static(ground_id, Vec3(0.0f, -0.5f, 0.0f));
        Collider ground_col = Collider::create_box(ground_id, ground_id, Vec3(15.0f, 0.5f, 15.0f), ground.get_transform());
        world_.add_body(ground);
        world_.add_collider(ground_col);

        // 3. Multi-Level Structural Tower (Stacked Fracture-Ready Blocks)
        // Level 1: Base Blocks (y = 0.5)
        create_tower_block(Vec3(-0.6f, 0.5f, 0.0f), Vec3(0.5f, 0.5f, 0.5f));
        create_tower_block(Vec3( 0.6f, 0.5f, 0.0f), Vec3(0.5f, 0.5f, 0.5f));

        // Level 2: Middle Structural Spanning Block (y = 1.5) - Designated primary fracture target
        uint32_t mid_id = create_tower_block(Vec3(0.0f, 1.5f, 0.0f), Vec3(1.1f, 0.5f, 0.5f));
        fracture_targets_.push_back(mid_id);

        // Level 3: Upper Column Blocks (y = 2.5)
        create_tower_block(Vec3(-0.5f, 2.5f, 0.0f), Vec3(0.4f, 0.5f, 0.4f));
        create_tower_block(Vec3( 0.5f, 2.5f, 0.0f), Vec3(0.4f, 0.5f, 0.4f));

        // Level 4: Top Crown Cap (y = 3.5)
        create_tower_block(Vec3(0.0f, 3.5f, 0.0f), Vec3(0.9f, 0.4f, 0.6f));

        accumulator_ = 0.0f;
        camera_.reset(Vec3(0.0f, 3.5f, 9.0f), -90.0f, -12.0f);
    }

    uint32_t create_tower_block(const Vec3& pos, const Vec3& half_extents) {
        uint32_t id = next_body_id_++;
        float mass = 8.0f * half_extents.x * half_extents.y * half_extents.z * 2400.0f;
        InertiaTensor inertia = InertiaTensor::box(mass, half_extents.x * 2.0f, half_extents.y * 2.0f, half_extents.z * 2.0f);

        RigidBody body = RigidBody::create_dynamic(id, mass, inertia, pos);
        Collider col = Collider::create_box(id, id, half_extents, body.get_transform());

        world_.add_body(body);
        world_.add_collider(col);
        return id;
    }

    void trigger_fracture(uint32_t body_id) {
        RigidBody* target_body = world_.get_body(body_id);
        Collider* target_col = world_.get_collider(body_id);
        if (!target_body || !target_col) return;

        Vec3 pos = target_body->position;
        Quat rot = target_body->orientation;
        Vec3 h = target_col->box_half_extents;

        // 1. Generate 3D Voronoi partition from local volume
        FractureVolume vol(Vec3(-h.x, -h.y, -h.z), Vec3(h.x, h.y, h.z), 2400.0f);
        std::vector<Vec3> sites = SiteGenerator::generate_3d(vol, 6, 0.2f, 42);
        std::vector<Shard> shards = Voronoi3D::compute_partition(vol, sites);

        // 2. Remove the intact body and collider
        auto& bodies = world_.get_bodies();
        bodies.erase(std::remove_if(bodies.begin(), bodies.end(),
            [body_id](const RigidBody& b) { return b.id == body_id; }), bodies.end());

        auto& colliders = world_.get_colliders();
        colliders.erase(std::remove_if(colliders.begin(), colliders.end(),
            [body_id](const Collider& c) { return c.id == body_id; }), colliders.end());

        // 3. Register Voronoi shards as dynamic rigid bodies & colliders
        Transform parent_t(pos, rot);
        for (const auto& s : shards) {
            if (!s.is_valid) continue;

            uint32_t shard_id = next_body_id_++;
            Vec3 shard_world_pos = parent_t.transform_point(s.centroid);
            InertiaTensor inertia = InertiaTensor::box(s.mass, 0.4f, 0.4f, 0.4f);

            // Shard geometry vertices in local space centered at centroid
            ConvexPolyhedron local_poly = s.mesh;
            for (auto& v : local_poly.vertices) {
                v -= s.centroid;
            }

            RigidBody shard_body = RigidBody::create_dynamic(shard_id, s.mass, inertia, shard_world_pos, rot);
            Collider shard_col = Collider::create_polyhedron(shard_id, shard_id, local_poly, shard_body.get_transform());

            world_.add_body(shard_body);
            world_.add_collider(shard_col);
        }

        renderer_.clear_cache();
    }

    void launch_projectile() {
        uint32_t proj_id = next_projectile_id_++;
        Vec3 origin = camera_.position + camera_.front * 0.8f;
        Vec3 vel = camera_.front * 28.0f; // 28 m/s projectile launch speed

        float mass = 15.0f; // 15 kg iron projectile
        InertiaTensor inertia = InertiaTensor::box(mass, 0.4f, 0.4f, 0.4f);

        RigidBody body = RigidBody::create_dynamic(proj_id, mass, inertia, origin);
        body.linear_velocity = vel;

        Collider col = Collider::create_box(proj_id, proj_id, Vec3(0.2f, 0.2f, 0.2f), body.get_transform());

        world_.add_body(body);
        world_.add_collider(col);
    }

    void process_input(float dt) {
        if (headless_mode_ || !window_) return;

        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window_, true);

        bool fast = (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);
        if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS)
            camera_.process_keyboard(CameraMovement::Forward, dt, fast);
        if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS)
            camera_.process_keyboard(CameraMovement::Backward, dt, fast);
        if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS)
            camera_.process_keyboard(CameraMovement::Left, dt, fast);
        if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS)
            camera_.process_keyboard(CameraMovement::Right, dt, fast);
        if (glfwGetKey(window_, GLFW_KEY_E) == GLFW_PRESS || glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS)
            camera_.process_keyboard(CameraMovement::Up, dt, fast);
        if (glfwGetKey(window_, GLFW_KEY_Q) == GLFW_PRESS || glfwGetKey(window_, GLFW_KEY_C) == GLFW_PRESS)
            camera_.process_keyboard(CameraMovement::Down, dt, fast);

        // Key debounce helper
        static bool p_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_P) == GLFW_PRESS) {
            if (!p_pressed) {
                paused_ = !paused_;
                p_pressed = true;
            }
        } else {
            p_pressed = false;
        }

        static bool o_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_O) == GLFW_PRESS) {
            if (!o_pressed) {
                single_step_requested_ = true;
                o_pressed = true;
            }
        } else {
            o_pressed = false;
        }

        static bool r_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS) {
            if (!r_pressed) {
                reset_scene();
                r_pressed = true;
            }
        } else {
            r_pressed = false;
        }

        static bool f_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_F) == GLFW_PRESS) {
            if (!f_pressed) {
                if (!fracture_targets_.empty()) {
                    trigger_fracture(fracture_targets_.front());
                    fracture_targets_.erase(fracture_targets_.begin());
                }
                f_pressed = true;
            }
        } else {
            f_pressed = false;
        }

        static bool space_launch_pressed = false;
        if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            if (!space_launch_pressed) {
                launch_projectile();
                space_launch_pressed = true;
            }
        } else {
            space_launch_pressed = false;
        }

        // Debug Overlay Toggles (1-5)
        static bool k1_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_1) == GLFW_PRESS) {
            if (!k1_pressed) { render_flags_.wireframe = !render_flags_.wireframe; k1_pressed = true; }
        } else { k1_pressed = false; }

        static bool k2_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_2) == GLFW_PRESS) {
            if (!k2_pressed) { render_flags_.show_aabbs = !render_flags_.show_aabbs; k2_pressed = true; }
        } else { k2_pressed = false; }

        static bool k3_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_3) == GLFW_PRESS) {
            if (!k3_pressed) { render_flags_.show_contacts = !render_flags_.show_contacts; k3_pressed = true; }
        } else { k3_pressed = false; }

        static bool k4_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_4) == GLFW_PRESS) {
            if (!k4_pressed) { render_flags_.show_support_graph = !render_flags_.show_support_graph; k4_pressed = true; }
        } else { k4_pressed = false; }

        static bool k5_pressed = false;
        if (glfwGetKey(window_, GLFW_KEY_5) == GLFW_PRESS) {
            if (!k5_pressed) { render_flags_.show_centers_of_mass = !render_flags_.show_centers_of_mass; k5_pressed = true; }
        } else { k5_pressed = false; }
    }

    void step_simulation() {
        auto t0 = std::chrono::steady_clock::now();

        // Check if any projectile impacted a fracture-ready target
        for (const auto& manifold : world_.get_active_manifolds()) {
            uint32_t a = manifold.body_a_id;
            uint32_t b = manifold.body_b_id;
            if (a >= 1000 || b >= 1000) {
                uint32_t target_id = (a >= 1000) ? b : a;
                auto it = std::find(fracture_targets_.begin(), fracture_targets_.end(), target_id);
                if (it != fracture_targets_.end()) {
                    trigger_fracture(target_id);
                    fracture_targets_.erase(it);
                    break;
                }
            }
        }

        // Execute full M4 physical simulation step
        world_.step_full(fixed_dt_);

        auto t1 = std::chrono::steady_clock::now();
        telemetry_.physics_time_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    }

    void update_telemetry(double current_time) {
        telemetry_.body_count = world_.body_count();
        telemetry_.contact_count = 0;
        telemetry_.max_penetration = 0.0f;
        telemetry_.shard_count = 0;

        for (const auto& col : world_.get_colliders()) {
            if (col.type == ColliderType::ConvexPolyhedron) {
                telemetry_.shard_count++;
            }
        }

        for (const auto& manifold : world_.get_active_manifolds()) {
            telemetry_.contact_count += manifold.points.size();
            telemetry_.max_penetration = std::max(telemetry_.max_penetration, manifold.max_penetration_depth);
        }

        telemetry_.broadphase_pairs = world_.get_active_manifolds().size();
        telemetry_.unsupported_shards = world_.get_graph().unsupported_count();
        telemetry_.broken_edges = world_.get_graph().broken_edge_count();

        float total_ke = 0.0f;
        for (const auto& b : world_.get_bodies()) {
            if (b.is_static) continue;
            total_ke += 0.5f * b.mass * b.linear_velocity.length_sq();
        }
        telemetry_.total_kinetic_energy = total_ke;

        if (current_time - last_telemetry_print_ >= 1.0) {
            telemetry_.fps = static_cast<float>(frame_count_) / static_cast<float>(current_time - last_telemetry_print_);
            frame_count_ = 0;
            last_telemetry_print_ = current_time;

            if (!headless_mode_) {
                std::cout << "[Telemetry] FPS: " << std::fixed << std::setprecision(1) << telemetry_.fps
                          << " | Physics: " << std::setprecision(2) << telemetry_.physics_time_ms << " ms"
                          << " | Bodies: " << telemetry_.body_count
                          << " | Shards: " << telemetry_.shard_count
                          << " | Contacts: " << telemetry_.contact_count
                          << " | MaxPen: " << std::setprecision(4) << telemetry_.max_penetration << " m"
                          << " | Unsupported: " << telemetry_.unsupported_shards
                          << " | BrokenEdges: " << telemetry_.broken_edges << "\n";
            }
        }
    }

    void run() {
        last_frame_time_ = glfwGetTime();
        last_telemetry_print_ = last_frame_time_;

        int frames_run = 0;

        while (!glfwWindowShouldClose(window_)) {
            double current_time = glfwGetTime();
            float delta_time = static_cast<float>(current_time - last_frame_time_);
            last_frame_time_ = current_time;

            if (delta_time > 0.25f) delta_time = 0.25f; // Cap delta to prevent spiral of death

            process_input(delta_time);

            // Fixed-timestep simulation loop
            if (!paused_) {
                accumulator_ += delta_time;
                while (accumulator_ >= fixed_dt_) {
                    step_simulation();
                    accumulator_ -= fixed_dt_;
                }
            } else if (single_step_requested_) {
                step_simulation();
                single_step_requested_ = false;
            }

            // Render
            glClearColor(0.12f, 0.14f, 0.18f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            float aspect = static_cast<float>(window_width_) / static_cast<float>(window_height_ > 0 ? window_height_ : 1);
            renderer_.render_scene(world_, camera_, aspect, render_flags_);

            glfwSwapBuffers(window_);
            glfwPollEvents();

            frame_count_++;
            frames_run++;
            update_telemetry(current_time);

            if (max_frames_ > 0 && frames_run >= max_frames_) {
                break;
            }
        }
    }

    // Static GLFW dispatch callbacks
    static void mouse_callback_dispatch(GLFWwindow* w, double xpos, double ypos) {
        auto* app = static_cast<SandboxApp*>(glfwGetWindowUserPointer(w));
        if (!app) return;

        static double last_x = xpos;
        static double last_y = ypos;
        static bool first_mouse = true;

        if (first_mouse) {
            last_x = xpos;
            last_y = ypos;
            first_mouse = false;
        }

        float xoffset = static_cast<float>(xpos - last_x);
        float yoffset = static_cast<float>(last_y - ypos); // Reversed since y-coordinates go bottom to top
        last_x = xpos;
        last_y = ypos;

        app->camera_.process_mouse_movement(xoffset, yoffset);
    }

    static void scroll_callback_dispatch(GLFWwindow* w, double /*xoffset*/, double yoffset) {
        auto* app = static_cast<SandboxApp*>(glfwGetWindowUserPointer(w));
        if (!app) return;

        app->camera_.fov -= static_cast<float>(yoffset) * 2.0f;
        app->camera_.fov = std::clamp(app->camera_.fov, 20.0f, 90.0f);
    }

    static void resize_callback_dispatch(GLFWwindow* w, int width, int height) {
        auto* app = static_cast<SandboxApp*>(glfwGetWindowUserPointer(w));
        if (!app) return;

        app->window_width_ = width;
        app->window_height_ = height;
        glViewport(0, 0, width, height);
    }
};

} // namespace destruction::render

#endif // DESTRUCTION_SANDBOX_APP_HPP
