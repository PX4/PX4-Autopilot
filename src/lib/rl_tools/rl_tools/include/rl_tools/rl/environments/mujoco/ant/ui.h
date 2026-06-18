#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT_UI_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT_UI_H
#include <GLFW/glfw3.h>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::mujoco::ant {
    template <typename T_ENVIRONMENT>
    struct UI{
        using ENVIRONMENT = T_ENVIRONMENT;
        ENVIRONMENT* env;
        GLFWwindow* window;
        mjvCamera camera;
        mjvOption option;
        mjvScene scene;
        mjrContext context;
        bool button_left = false;
        bool button_middle = false;
        bool button_right =  false;
        double lastx = 0;
        double lasty = 0;
    };
    namespace ui::callbacks{
        template <typename UI>
        void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
            UI* ui = (UI*)glfwGetWindowUserPointer(window);
            if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
                mj_resetData(ui->env->model, ui->env->data);
                mj_forward(ui->env->model, ui->env->data);
            }
        }
        template <typename UI>
        void mouse_button(GLFWwindow* window, int button, int act, int mods) {
            UI* ui = (UI*)glfwGetWindowUserPointer(window);
            ui->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
            ui->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
            ui->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

            glfwGetCursorPos(window, &ui->lastx, &ui->lasty);
        }

        template <typename UI>
        void mouse_move(GLFWwindow* window, double xpos, double ypos) {
            UI* ui = (UI*)glfwGetWindowUserPointer(window);
            if (!ui->button_left && !ui->button_middle && !ui->button_right) {
                return;
            }
            double dx = xpos - ui->lastx;
            double dy = ypos - ui->lasty;
            ui->lastx = xpos;
            ui->lasty = ypos;
            int width, height;
            glfwGetWindowSize(window, &width, &height);
            bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
            mjtMouse action;
            if (ui->button_right) {
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            } else if (ui->button_left) {
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            } else {
                action = mjMOUSE_ZOOM;
            }
            mjv_moveCamera(ui->env->model, action, dx/height, dy/height, &ui->scene, &ui->camera);
        }

        template <typename UI>
        void scroll(GLFWwindow* window, double xoffset, double yoffset) {
            UI* ui = (UI*)glfwGetWindowUserPointer(window);
            mjv_moveCamera(ui->env->model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &ui->scene, &ui->camera);
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename ENVIRONMENT>
    void init(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, rl::environments::mujoco::ant::UI<ENVIRONMENT>& ui){
        using UI = rl::environments::mujoco::ant::UI<ENVIRONMENT>;
        ui.env = &env;
        if (!glfwInit()) {
            mju_error("Could not initialize GLFW");
        }
        ui.window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
        glfwSetWindowUserPointer(ui.window, &ui);
        glfwMakeContextCurrent(ui.window);
        glfwSwapInterval(1);

        mjv_defaultCamera(&ui.camera);
        ui.camera.type = mjCAMERA_TRACKING;
        ui.camera.trackbodyid = env.torso_id;
        ui.camera.distance = 6;
        mjv_defaultOption(&ui.option);
        mjv_defaultScene(&ui.scene);
        mjr_defaultContext(&ui.context);

        mjv_makeScene(ui.env->model, &ui.scene, 2000);
        mjr_makeContext(ui.env->model, &ui.context, mjFONTSCALE_150);

        glfwSetKeyCallback(ui.window, rl::environments::mujoco::ant::ui::callbacks::keyboard<UI>);
        glfwSetCursorPosCallback(ui.window, rl::environments::mujoco::ant::ui::callbacks::mouse_move<UI>);
        glfwSetMouseButtonCallback(ui.window, rl::environments::mujoco::ant::ui::callbacks::mouse_button<UI>);
        glfwSetScrollCallback(ui.window, rl::environments::mujoco::ant::ui::callbacks::scroll<UI>);
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void set_state(DEVICE& dev, ENVIRONMENT& environment, typename ENVIRONMENT::Parameters& parameters, rl::environments::mujoco::ant::UI<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state){
        using TI = typename DEVICE::index_t;
        for(TI state_i = 0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q; state_i++){
            ui.env->data->qpos[state_i] = state.q[state_i];
        }
        for(TI state_i = 0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q_DOT; state_i++){
            ui.env->data->qvel[state_i] = state.q_dot[state_i];
        }
        mj_forward(ui.env->model, ui.env->data);
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(ui.window, &viewport.width, &viewport.height);

        mjv_updateScene(ui.env->model, ui.env->data, &ui.option, NULL, &ui.camera, mjCAT_ALL, &ui.scene);
        mjr_render(viewport, &ui.scene, &ui.context);

        glfwSwapBuffers(ui.window);

        glfwPollEvents();
    }

    template <typename DEVICE, typename ENVIRONMENT>
    void free(DEVICE& dev, ENVIRONMENT& env, rl::environments::mujoco::ant::UI<ENVIRONMENT>& ui){
        mjv_freeScene(&ui.scene);
        mjr_freeContext(&ui.context);

#if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
#endif
    }


}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
