#include "raylab/bvh_node.hpp"
#include "raylab/camera.hpp"
#include "raylab/color.hpp"
#include "raylab/hittable_list.hpp"
#include "raylab/ppm_writer.hpp"
#include "raylab/raytracer.hpp"
#include "raylab/render_stats.hpp"
#include "raylab/rng.hpp"
#include "raylab/scene_builder.hpp"

#include "../scenes/demo_scene.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

void print_usage(const char* exec_name) {
    std::cout << "Usage: " << exec_name << " [options]\n"
              << "Options:\n"
              << "  --width <int>        Image width in pixels (default: 400)\n"
              << "  --height <int>       Image height in pixels (default: 225)\n"
              << "  --samples <int>      Samples per pixel for anti-aliasing (default: 1)\n"
              << "  --max-depth <int>    Maximum ray recursion bounce depth (default: 10)\n"
              << "  --aperture <double>  Lens aperture diameter (default: 0.0 for pinhole)\n"
              << "  --focus-dist <double>Distance to focal plane (default: 1.0)\n"
              << "  --scene <string>     Scene choice: 'demo' or 'random' (default: demo)\n"
              << "  --seed <uint>        RNG seed for deterministic sampling (default: 42)\n"
              << "  --accel <string>     Acceleration structure: 'naive' or 'bvh' (default: bvh)\n"
              << "  --threads <int>      Number of threads, 0 for auto (default: 1)\n"
              << "  --output <file>      Output PPM filename (default: image.ppm)\n"
              << "  --help, -h           Display this help message\n";
}

int main(int argc, char* argv[]) {
    int width = 400;
    int height = 225;
    int samples = 1;
    int max_depth = 10;
    double aperture = 0.0;
    double focus_dist = 1.0;
    bool focus_dist_custom = false;
    uint64_t seed = 42;
    std::string scene_name = "demo";
    std::string accel_name = "bvh";
    int requested_threads = 1;
    std::string output_filename = "image.ppm";

    // Standard library argument parsing
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--width" && i + 1 < argc) {
            try {
                width = std::stoi(argv[++i]);
                if (width <= 0) throw std::invalid_argument("width must be positive");
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid argument for --width: " << e.what() << "\n";
                return 1;
            }
        } else if (arg == "--height" && i + 1 < argc) {
            try {
                height = std::stoi(argv[++i]);
                if (height <= 0) throw std::invalid_argument("height must be positive");
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid argument for --height: " << e.what() << "\n";
                return 1;
            }
        } else if (arg == "--samples" && i + 1 < argc) {
            try {
                samples = std::stoi(argv[++i]);
                if (samples < 1) throw std::invalid_argument("samples must be >= 1");
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid argument for --samples: " << e.what() << "\n";
                return 1;
            }
        } else if (arg == "--max-depth" && i + 1 < argc) {
            try {
                max_depth = std::stoi(argv[++i]);
                if (max_depth < 1) throw std::invalid_argument("max-depth must be >= 1");
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid argument for --max-depth: " << e.what() << "\n";
                return 1;
            }
        } else if (arg == "--aperture" && i + 1 < argc) {
            try {
                aperture = std::stod(argv[++i]);
                if (aperture < 0.0) throw std::invalid_argument("aperture must be >= 0");
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid argument for --aperture: " << e.what() << "\n";
                return 1;
            }
        } else if (arg == "--focus-dist" && i + 1 < argc) {
            try {
                focus_dist = std::stod(argv[++i]);
                if (focus_dist <= 0.0) throw std::invalid_argument("focus-dist must be > 0");
                focus_dist_custom = true;
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid argument for --focus-dist: " << e.what() << "\n";
                return 1;
            }
        } else if (arg == "--scene" && i + 1 < argc) {
            scene_name = argv[++i];
        } else if (arg == "--seed" && i + 1 < argc) {
            try {
                seed = std::stoull(argv[++i]);
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid argument for --seed: " << e.what() << "\n";
                return 1;
            }
        } else if (arg == "--accel" && i + 1 < argc) {
            accel_name = argv[++i];
            if (accel_name != "naive" && accel_name != "bvh") {
                std::cerr << "Error: Invalid argument for --accel (must be 'naive' or 'bvh'): " << accel_name << "\n";
                return 1;
            }
        } else if (arg == "--threads" && i + 1 < argc) {
            try {
                requested_threads = std::stoi(argv[++i]);
                if (requested_threads < 0) throw std::invalid_argument("threads must be >= 0");
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid argument for --threads: " << e.what() << "\n";
                return 1;
            }
        } else if (arg == "--output" && i + 1 < argc) {
            output_filename = argv[++i];
        } else {
            std::cerr << "Error: Unknown or incomplete argument: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    // Resolve active thread count
    unsigned int hardware_threads = std::thread::hardware_concurrency();
    int num_threads = requested_threads;
    if (requested_threads == 0) {
        num_threads = (hardware_threads > 0) ? static_cast<int>(hardware_threads) : 1;
    }

    // Load selected scene
    raylab::SceneData scene;
    raylab::Point3 lookfrom(0.0, 0.0, 0.0);
    raylab::Point3 lookat(0.0, 0.0, -1.0);
    raylab::Vec3 vup(0.0, 1.0, 0.0);
    double vfov = 90.0;

    if (scene_name == "random") {
        scene = raylab::create_procedural_random_scene(seed);
        lookfrom = raylab::Point3(13.0, 2.0, 3.0);
        lookat = raylab::Point3(0.0, 0.0, 0.0);
        vfov = 20.0;
        if (!focus_dist_custom) {
            focus_dist = 10.0; // Focus distance for procedural scene overview
        }
    } else {
        scene = raylab::create_demo_scene();
        lookfrom = raylab::Point3(0.0, 0.0, 0.0);
        lookat = raylab::Point3(0.0, 0.0, -1.0);
        vfov = 90.0;
        if (!focus_dist_custom) {
            focus_dist = 1.0;
        }
    }

    // Construct world acceleration hierarchy / reference list
    std::shared_ptr<raylab::Hittable> render_world;
    if (accel_name == "bvh") {
        render_world = std::make_shared<raylab::BVHNode>(scene.world);
    } else {
        render_world = std::make_shared<raylab::HittableList>(scene.world);
    }

    // Initialize Camera
    raylab::Camera cam(lookfrom, lookat, vup, vfov, width, height, aperture, focus_dist);

    std::cout << "========================================================\n"
              << "Raylab CPU Ray Tracer (Milestone 5)\n"
              << "========================================================\n"
              << "  Resolution : " << width << " x " << height << "\n"
              << "  Samples/Px : " << samples << "\n"
              << "  Max Depth  : " << max_depth << "\n"
              << "  Aperture   : " << aperture << " (Lens Radius: " << cam.lens_radius() << ")\n"
              << "  Focus Dist : " << focus_dist << "\n"
              << "  Scene      : " << scene_name << "\n"
              << "  RNG Seed   : " << seed << "\n"
              << "  Accel Mode : " << accel_name << "\n"
              << "  Threads    : " << num_threads << " (Requested: " << requested_threads << ")\n"
              << "  Output File: " << output_filename << "\n"
              << "--------------------------------------------------------\n";

    std::vector<raylab::Color> image_buffer(static_cast<size_t>(width * height));
    raylab::RenderStats total_stats;

    auto start_time = std::chrono::high_resolution_clock::now();

    // Multithreaded tile/row rendering with per-thread RenderStats aggregation
    std::vector<std::thread> workers;
    std::vector<raylab::RenderStats> thread_stats(num_threads);

    auto render_rows = [&](int thread_id, int row_start, int row_end) {
        auto& local_stats = thread_stats[thread_id];

        for (int j = row_start; j < row_end; ++j) {
            for (int i = 0; i < width; ++i) {
                raylab::Color pixel_color(0.0, 0.0, 0.0);

                for (int s = 0; s < samples; ++s) {
                    uint64_t sample_seed = raylab::make_sample_seed(seed, i, j, s);
                    raylab::RNG sample_rng(sample_seed);

                    double u_offset = (samples == 1) ? 0.5 : sample_rng.next_double();
                    double v_offset = (samples == 1) ? 0.5 : sample_rng.next_double();

                    raylab::Ray r = cam.get_ray(i, j, u_offset, v_offset, sample_rng);
                    pixel_color += raylab::ray_color(r, *render_world, scene.light, 0, max_depth, sample_rng, &local_stats);
                }

                pixel_color /= static_cast<double>(samples);
                image_buffer[static_cast<size_t>(j * width + i)] = pixel_color;
            }
        }
    };

    int rows_per_thread = height / num_threads;
    int remaining_rows = height % num_threads;

    int current_row = 0;
    for (int t = 0; t < num_threads; ++t) {
        int count = rows_per_thread + (t < remaining_rows ? 1 : 0);
        int start_r = current_row;
        int end_r = current_row + count;
        current_row = end_r;

        workers.emplace_back(render_rows, t, start_r, end_r);
    }

    for (auto& w : workers) {
        if (w.joinable()) {
            w.join();
        }
    }

    // Aggregate statistics from all worker threads cleanly
    for (const auto& ts : thread_stats) {
        total_stats.merge(ts);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> render_duration = end_time - start_time;
    double seconds = render_duration.count() / 1000.0;

    double pixels_per_sec = (seconds > 0) ? (width * height) / seconds : 0.0;
    double samples_per_sec = (seconds > 0) ? total_stats.primary_samples / seconds : 0.0;
    double total_rays_per_sec = (seconds > 0) ? total_stats.total_rays() / seconds : 0.0;

    std::cout << "Render completed in " << render_duration.count() << " ms.\n"
              << "--------------------------------------------------------\n"
              << "Ray & Intersection Statistics:\n"
              << "  Primary Samples : " << total_stats.primary_samples << "\n"
              << "  Shadow Rays     : " << total_stats.shadow_rays << "\n"
              << "  Secondary Rays  : " << total_stats.secondary_rays << "\n"
              << "  Total Rays      : " << total_stats.total_rays() << "\n"
              << "  Sphere Tests    : " << total_stats.sphere_intersection_tests << "\n"
              << "  AABB Tests      : " << total_stats.aabb_tests << "\n"
              << "Throughput Performance:\n"
              << "  Pixels / sec    : " << static_cast<uint64_t>(pixels_per_sec) << "\n"
              << "  Samples / sec   : " << static_cast<uint64_t>(samples_per_sec) << "\n"
              << "  Total Rays / sec: " << static_cast<uint64_t>(total_rays_per_sec) << "\n"
              << "--------------------------------------------------------\n";

    // Write image
    if (!raylab::PPMWriter::write_ppm(output_filename, width, height, image_buffer)) {
        std::cerr << "Error: Failed to write output file.\n";
        return 1;
    }

    std::cout << "Successfully saved rendered image to " << output_filename << "\n";
    return 0;
}
