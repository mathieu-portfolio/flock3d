#include "app/Application.hpp"

#include <raylib.h>

namespace flock3d::app {

Application::Application()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(screen_width, screen_height, "flock3d");
    SetTargetFPS(0);

    camera_.position = Vector3{0.0F, 35.0F, 85.0F};
    camera_.target = Vector3{0.0F, 0.0F, 0.0F};
    camera_.up = Vector3{0.0F, 1.0F, 0.0F};
    camera_.fovy = 45.0F;
    camera_.projection = CAMERA_PERSPECTIVE;

    DisableCursor();
}

Application::~Application()
{
    if (IsWindowReady()) {
        CloseWindow();
    }
}

void Application::run()
{
    while (!WindowShouldClose()) {
        UpdateCamera(&camera_, CAMERA_FREE);

        timestep_.add_frame_time(GetFrameTime());
        while (timestep_.consume_step()) {
            simulation_.update(static_cast<float>(timestep_.fixed_dt()));
        }

        BeginDrawing();
        ClearBackground(Color{12, 16, 24, 255});

        BeginMode3D(camera_);
        DrawGrid(20, 4.0F);
        renderer_.draw(simulation_);
        EndMode3D();

        DrawText("flock3d - fixed simulation @ 120 Hz", 16, 16, 20, RAYWHITE);
        DrawText("WASD/mouse: free camera", 16, 42, 18, LIGHTGRAY);
        DrawFPS(GetScreenWidth() - 100, 16);

        EndDrawing();
    }
}

} // namespace flock3d::app
