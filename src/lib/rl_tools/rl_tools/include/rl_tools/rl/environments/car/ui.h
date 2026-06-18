#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_CAR_UI_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_CAR_UI_H

#include "car.h"

#include <gtk/gtk.h>
#include <thread>
#include <chrono>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::car{
    namespace ui{
        template<typename T_T, typename T_TI, typename T_ENVIRONMENT, T_TI T_SIZE, T_TI T_PLAYBACK_SPEED>
        struct Specification{
            using T = T_T;
            using TI = T_TI;
            using ENVIRONMENT = T_ENVIRONMENT;
            static constexpr TI SIZE = T_SIZE;
            static constexpr T PLAYBACK_SPEED = T_PLAYBACK_SPEED/100.0;
            static constexpr T ACTION_INDICATOR_SIZE = SIZE * 1.0/10.0;
            static constexpr T UI_SCALE = 500;
        };
    }

    template<typename T_SPEC>
    struct UI {
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_render_time;
        typename SPEC::ENVIRONMENT::State state;
        typename SPEC::ENVIRONMENT::Parameters parameters;
        MatrixDynamic<matrix::Specification<T, TI, 1, SPEC::ENVIRONMENT::ACTION_DIM>> action;
        GtkWidget *window;
        GtkWidget *canvas;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::car::ui{
    template <typename T>
    void R(T alpha, T result[2][2]) {
        result[0][0] = cos(alpha);
        result[0][1] = -sin(alpha);
        result[1][0] = sin(alpha);
        result[1][1] = cos(alpha);
    }
    template <typename T, typename TI>
    void global_position(T x_offset, T y_offset, T result[2], T x, T y, T alpha, const rl::environments::car::State<T, TI>& s, T scale) {
        T rotation[2][2];
        R(alpha, rotation);
        result[0] = x_offset + s.x * scale + (rotation[0][0] * x + rotation[0][1] * y) * scale;
        result[1] = y_offset - s.y * scale - (rotation[1][0] * x + rotation[1][1] * y) * scale;
    }
    template <typename T, typename TI>
    void draw_wheel(cairo_t* cr, T x_offset, T y_offset, T x, T y, T size, T width, T delta, const rl::environments::car::State<T, TI>& s, T scale){
        T start[2], finish[2];
        start[0] = -size / 2 * cos(delta);
        start[1] = -size / 2 * sin(delta);
        finish[0] = size / 2 * cos(delta);
        finish[1] = size / 2 * sin(delta);

        T position[2];
        global_position(x_offset, y_offset, position, x + start[0], y + start[1], s.mu, s, scale);
        cairo_move_to(cr, position[0], position[1]);
        global_position(x_offset, y_offset, position, x + finish[0], y + finish[1], s.mu, s, scale);
        cairo_line_to(cr, position[0], position[1]);
        cairo_set_source_rgb(cr, 1, 0, 0);
        cairo_set_line_width(cr, width*scale);
        cairo_stroke(cr);
    }
    template <typename T>
    void draw_track(cairo_t* cr, T offset_x, T offset_y, T ui_scale, Parameters<T>& params){ }
    template <typename T, typename TI, TI HEIGHT, TI WIDTH, TI SCALE>
    void draw_track(cairo_t* cr, T offset_x, T offset_y, T ui_scale, ParametersTrack<T, TI, HEIGHT, WIDTH, SCALE>& params){
        T scale = ui_scale * ParametersTrack<T, TI, HEIGHT, WIDTH, SCALE>::TRACK_SCALE;

        for(TI i = 0; i < HEIGHT; i++){
            for(TI j = 0; j < WIDTH; j++){
                if(!params.track[i][j]){
                    cairo_rectangle(cr, offset_x - WIDTH/2.0*scale + j*scale, offset_y - HEIGHT/2.0*scale + i*scale, 1*scale, 1*scale);
                    cairo_set_source_rgb(cr, 0, 0, 0);
                    cairo_fill(cr);
                }
            }
        }

    }

    template <typename SPEC>
    static gboolean draw_callback(GtkWidget *c, cairo_t *cr, gpointer data){
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        UI<SPEC>& ui = *(UI<SPEC>*)data;
        auto& p = ui.parameters;
        auto& s = ui.state;
        auto& a = ui.action;
        T h = gtk_widget_get_allocated_height(c);
        T w = gtk_widget_get_allocated_width(c);
        T x_offset = w / 2;
        T y_offset = h / 2;

        cairo_rectangle(cr, 0, 0, w, h);
        cairo_set_source_rgb(cr, 1, 1, 1);
        cairo_fill(cr);

        T scale = SPEC::UI_SCALE;
        draw_track(cr, x_offset, y_offset, scale, p);

        T L = p.lr + p.lf;
        T W = L / 2.5;

        T position[2];
        global_position<T, TI>(x_offset, y_offset, position, -p.lr, 0, s.mu, s, scale);
        cairo_move_to(cr, position[0], position[1]);
        global_position<T, TI>(x_offset, y_offset, position, p.lf, 0, s.mu, s, scale);
        cairo_line_to(cr, position[0], position[1]);
        cairo_set_source_rgb(cr, 0, 0, 1);
        cairo_set_line_width(cr, W*scale);
        cairo_stroke(cr);

        T wheel_size = L / 2;
        T wheel_width = L / 10;

        draw_wheel<T, TI>(cr, x_offset, y_offset, -p.lr, W/2 + wheel_width/2, wheel_size, wheel_width, 0, s, scale);
        draw_wheel<T, TI>(cr, x_offset, y_offset, -p.lr, -W/2 - wheel_width/2, wheel_size, wheel_width, 0, s, scale);
        draw_wheel<T, TI>(cr, x_offset, y_offset, p.lf, W/2 + wheel_width/2, wheel_size, wheel_width, get(a, 0, 1), s, scale);
        draw_wheel<T, TI>(cr, x_offset, y_offset, p.lf, -W/2 - wheel_width/2, wheel_size, wheel_width, get(a, 0, 1), s, scale);
        return FALSE;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename ENV_SPEC, typename SPEC>
    void render(DEVICE& device, const rl::environments::Car<ENV_SPEC>& env, const typename rl::environments::Car<ENV_SPEC>::Parameters& parameters, rl::environments::car::UI<SPEC>& ui){
        auto now = std::chrono::high_resolution_clock::now();
        auto interval = (typename DEVICE::index_t)(1000.0 * parameters.dt / SPEC::PLAYBACK_SPEED);
        auto next_render_time = ui.last_render_time + std::chrono::milliseconds(interval);
        if(now < next_render_time){
            auto diff = next_render_time - now;
            std::this_thread::sleep_for(diff);
        }

        using T = typename SPEC::T;
        while(gtk_events_pending()){
            gtk_main_iteration_do(FALSE);
        }
        gtk_widget_queue_draw(ui.canvas);


        now = std::chrono::high_resolution_clock::now();
        ui.last_render_time = now;
    }
    template <typename DEVICE, typename ENV_SPEC, typename SPEC>
    void init(DEVICE& device, const rl::environments::Car<ENV_SPEC>& env, typename rl::environments::Car<ENV_SPEC>::Parameters& parameters, rl::environments::car::UI<SPEC>& ui){
        malloc(device, ui.action);
        ui.parameters = parameters;
        gtk_init(nullptr, nullptr);
        ui.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_default_size(GTK_WINDOW(ui.window), 1000, 1000);
        g_signal_connect(ui.window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
        ui.canvas = gtk_drawing_area_new();
        gtk_container_add(GTK_CONTAINER(ui.window), ui.canvas);
        g_signal_connect(G_OBJECT(ui.canvas), "draw", G_CALLBACK(rl::environments::car::ui::draw_callback<SPEC>), &ui);
        gtk_widget_show_all(ui.window);
        render(device, env, parameters, ui);
        ui.last_render_time = std::chrono::high_resolution_clock::now();
    }
    template <typename DEVICE, typename ENV_SPEC, typename SPEC, typename T, typename TI>
    void set_state(DEVICE& device, const rl::environments::Car<ENV_SPEC>& env, typename rl::environments::Car<ENV_SPEC>::Parameters& parameters, rl::environments::car::UI<SPEC>& ui, const rl::environments::car::State<T, TI>& state){
        ui.state = state;
    }
    template <typename DEVICE, typename ENV_SPEC, typename SPEC, typename ACTION_SPEC>
    void set_action(DEVICE& device, const rl::environments::Car<ENV_SPEC>& env, typename rl::environments::Car<ENV_SPEC>::Parameters& parameters, rl::environments::car::UI<SPEC>& ui, const Matrix<ACTION_SPEC>& action){
        using ENVIRONMENT = rl::environments::Car<ENV_SPEC>;
        static_assert(ACTION_SPEC::ROWS == 1 && ACTION_SPEC::COLS == ENVIRONMENT::ACTION_DIM);
        copy(device, device, action, ui.action);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
