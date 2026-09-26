#include "voxel_lab/camera.hpp"
#include "voxel_lab/gl_loader.hpp"
#include "voxel_lab/gl_shader.hpp"
#include "voxel_lab/scene_manager.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace voxel_lab;

namespace {

Camera g_camera;
bool g_first_mouse = true;
double g_last_x = 640.0;
double g_last_y = 360.0;
bool g_wireframe = false;

// Shaders
const char* VERTEX_SHADER_SRC = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vNormal;
out vec3 vFragPos;

void main() {
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vFragPos = worldPos.xyz;
    vNormal = mat3(uModel) * aNormal;
    gl_Position = uProjection * uView * worldPos;
}
)";

const char* FRAGMENT_SHADER_SRC = R"(#version 330 core
in vec3 vNormal;
in vec3 vFragPos;

uniform vec3 uLightDir;
uniform vec3 uBaseColor;

out vec4 FragColor;

void main() {
    vec3 norm = normalize(vNormal);
    vec3 lightDir = normalize(uLightDir);

    float ambient = 0.35;
    float diff = max(dot(norm, lightDir), 0.0) * 0.65;
    vec3 litColor = (ambient + diff) * uBaseColor;

    FragColor = vec4(litColor, 1.0);
}
)";

void mouse_callback(GLFWwindow* /*window*/, double xpos, double ypos) {
    if (g_first_mouse) {
        g_last_x = xpos;
        g_last_y = ypos;
        g_first_mouse = false;
        return;
    }

    float x_offset = static_cast<float>(xpos - g_last_x);
    float y_offset = static_cast<float>(g_last_y - ypos); // Reversed since y-coordinates range from bottom to top

    g_last_x = xpos;
    g_last_y = ypos;

    g_camera.process_mouse_movement(x_offset, y_offset);
}

void process_input(GLFWwindow* window, float delta_time, SceneManager& scene_mgr) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        g_camera.process_keyboard(CameraMovement::Forward, delta_time);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        g_camera.process_keyboard(CameraMovement::Backward, delta_time);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        g_camera.process_keyboard(CameraMovement::Left, delta_time);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        g_camera.process_keyboard(CameraMovement::Right, delta_time);
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        g_camera.process_keyboard(CameraMovement::Up, delta_time);
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        g_camera.process_keyboard(CameraMovement::Down, delta_time);

    // Scene switching keys
    static bool key_1_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
        if (!key_1_pressed) {
            scene_mgr.switch_scene(1, g_camera);
            key_1_pressed = true;
        }
    } else {
        key_1_pressed = false;
    }

    static bool key_2_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
        if (!key_2_pressed) {
            scene_mgr.switch_scene(2, g_camera);
            key_2_pressed = true;
        }
    } else {
        key_2_pressed = false;
    }

    static bool key_3_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS) {
        if (!key_3_pressed) {
            scene_mgr.switch_scene(3, g_camera);
            key_3_pressed = true;
        }
    } else {
        key_3_pressed = false;
    }

    static bool key_4_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS) {
        if (!key_4_pressed) {
            scene_mgr.switch_scene(4, g_camera);
            key_4_pressed = true;
        }
    } else {
        key_4_pressed = false;
    }

    // Mesher switching keys
    static bool key_n_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) {
        if (!key_n_pressed) {
            scene_mgr.set_mesher(MesherType::Naive);
            key_n_pressed = true;
        }
    } else {
        key_n_pressed = false;
    }

    static bool key_g_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS) {
        if (!key_g_pressed) {
            scene_mgr.set_mesher(MesherType::Greedy);
            key_g_pressed = true;
        }
    } else {
        key_g_pressed = false;
    }

    // Wireframe toggle
    static bool key_f_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
        if (!key_f_pressed) {
            g_wireframe = !g_wireframe;
            glPolygonMode(GL_FRONT_AND_BACK, g_wireframe ? GL_LINE : GL_FILL);
            key_f_pressed = true;
        }
    } else {
        key_f_pressed = false;
    }

    // Reset camera key
    static bool key_r_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
        if (!key_r_pressed) {
            scene_mgr.switch_scene(scene_mgr.get_current_scene_index(), g_camera);
            key_r_pressed = true;
        }
    } else {
        key_r_pressed = false;
    }
}

} // anonymous namespace

int main(int argc, char** argv) {
    std::cout << "=== Graphics Sim Lab — Project 02 Voxel Engine Viewer ===\n";

    int max_test_frames = -1;
    bool test_all_scenes = false;
    bool headless = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--test-frames") == 0 && i + 1 < argc) {
            max_test_frames = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--test-all-scenes") == 0) {
            test_all_scenes = true;
            max_test_frames = 200; // 20 frames per mode/scene combination
        } else if (std::strcmp(argv[i], "--headless") == 0) {
            headless = true;
        }
    }

    if (!glfwInit()) {
        std::cerr << "[GLFW Error] Failed to initialize GLFW library.\n";
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    if (headless) {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // Headless-capable test mode
    }

    const int window_width = 1280;
    const int window_height = 720;
    GLFWwindow* window = glfwCreateWindow(window_width, window_height,
                                          "Voxel Engine — Milestone 7 Viewer",
                                          nullptr, nullptr);
    if (!window) {
        std::cerr << "[GLFW Error] Failed to create OpenGL 3.3 Core window.\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);

    // Initialize modern OpenGL function loader
    if (!init_gl_loader(reinterpret_cast<GLProcLoader>(glfwGetProcAddress))) {
        std::cerr << "[OpenGL Error] Failed to load OpenGL 3.3 Core function pointers.\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    std::cout << "[OpenGL Info] Vendor:   " << glGetString(GL_VENDOR) << "\n";
    std::cout << "[OpenGL Info] Renderer: " << glGetString(GL_RENDERER) << "\n";
    std::cout << "[OpenGL Info] Version:  " << glGetString(GL_VERSION) << "\n";

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClearColor(0.08f, 0.10f, 0.13f, 1.0f);

    if (max_test_frames < 0) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwSetCursorPosCallback(window, mouse_callback);
    }

    GLShader shader;
    if (!shader.compile(VERTEX_SHADER_SRC, FRAGMENT_SHADER_SRC)) {
        std::cerr << "[Shader Error] Failed to compile visualization shader.\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    SceneManager scene_mgr;
    scene_mgr.switch_scene(4, g_camera); // Default to Milestone 7 Dynamic Streaming World

    std::cout << "\nControls:\n";
    std::cout << "  [W/A/S/D]   Move Camera (Horizontal)\n";
    std::cout << "  [Q/E]       Move Camera (Down/Up)\n";
    std::cout << "  [Mouse]     Look around (Pitch/Yaw)\n";
    std::cout << "  [1/2/3/4]   Switch Scene (1: Solid, 2: Plane, 3: Sphere, 4: Streaming)\n";
    std::cout << "  [N]         Switch to Naive Mesher\n";
    std::cout << "  [G]         Switch to Greedy Mesher\n";
    std::cout << "  [F]         Toggle Wireframe Mode\n";
    std::cout << "  [R]         Reset Camera\n";
    std::cout << "  [ESC]       Exit Viewer\n\n";

    auto last_frame_time = std::chrono::high_resolution_clock::now();
    double fps_timer = 0.0;
    int frame_count = 0;
    int total_frames = 0;

    while (!glfwWindowShouldClose(window)) {
        auto current_time = std::chrono::high_resolution_clock::now();
        float delta_time = std::chrono::duration<float>(current_time - last_frame_time).count();
        last_frame_time = current_time;

        if (max_test_frames < 0) {
            process_input(window, delta_time, scene_mgr);
        } else if (test_all_scenes) {
            if (total_frames == 20) {
                scene_mgr.set_mesher(MesherType::Naive);
            } else if (total_frames == 40) {
                scene_mgr.set_mesher(MesherType::Greedy);
                scene_mgr.switch_scene(1, g_camera);
            } else if (total_frames == 60) {
                scene_mgr.switch_scene(2, g_camera);
            } else if (total_frames == 80) {
                scene_mgr.switch_scene(3, g_camera);
            } else if (total_frames == 100) {
                scene_mgr.switch_scene(4, g_camera);
            } else if (total_frames == 120) {
                // Positive boundary crossing
                g_camera.reset(Vec3(40.0f, 25.0f, 40.0f), -90.0f, -20.0f);
            } else if (total_frames == 140) {
                // Negative boundary crossing
                g_camera.reset(Vec3(-40.0f, 25.0f, -40.0f), -90.0f, -20.0f);
            } else if (total_frames == 160) {
                // Wireframe mode
                g_wireframe = !g_wireframe;
                glPolygonMode(GL_FRONT_AND_BACK, g_wireframe ? GL_LINE : GL_FILL);
            } else if (total_frames == 180) {
                scene_mgr.set_mesher(MesherType::Naive);
            }
        }

        // Update dynamic scenes (e.g. streaming around camera in Scene 4)
        scene_mgr.update(g_camera);

        // Render pass
        int display_w = 0, display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        float aspect = (display_h > 0) ? (static_cast<float>(display_w) / static_cast<float>(display_h)) : 1.777f;
        Mat4 view = g_camera.get_view_matrix();
        Mat4 proj = g_camera.get_projection_matrix(aspect);

        scene_mgr.render(shader, view, proj);

        glfwSwapBuffers(window);
        glfwPollEvents();

        // FPS & title updates
        fps_timer += delta_time;
        frame_count++;
        total_frames++;

        if (fps_timer >= 0.5) {
            double fps = static_cast<double>(frame_count) / fps_timer;
            const SceneStats& stats = scene_mgr.get_current_stats();
            std::stringstream title;
            title << "Voxel Engine | " << stats.name
                  << " | Mesher: [" << mesher_type_name(stats.mesher) << "]";
            if (scene_mgr.get_current_scene_index() == 4) {
                title << " | CamChunk: (" << stats.cam_chunk.x << "," << stats.cam_chunk.y << "," << stats.cam_chunk.z << ")"
                      << " | Chunks: " << stats.chunk_count << " (+" << stats.chunks_loaded_last_update << "/-" << stats.chunks_unloaded_last_update << ")";
            } else {
                title << " | Chunks: " << stats.chunk_count;
            }
            title << " | Quads/Faces: " << stats.face_count
                  << " | Verts: " << stats.vertex_count
                  << " | FPS: " << std::fixed << std::setprecision(1) << fps;
            glfwSetWindowTitle(window, title.str().c_str());

            fps_timer = 0.0;
            frame_count = 0;
        }

        if (max_test_frames > 0 && total_frames >= max_test_frames) {
            std::cout << "[Test Mode] Rendered " << total_frames << " frames successfully. Exiting cleanly.\n";
            break;
        }
    }

    const SceneStats& final_stats = scene_mgr.get_current_stats();
    std::cout << "[Viewer Shutdown] Active Scene: " << final_stats.name
              << " | Mesher: [" << mesher_type_name(final_stats.mesher) << "]"
              << " | Chunks: " << final_stats.chunk_count
              << " | Quads/Faces: " << final_stats.face_count
              << " | Verts: " << final_stats.vertex_count
              << " | Total Rendered Frames: " << total_frames << "\n";

    shader.destroy();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
