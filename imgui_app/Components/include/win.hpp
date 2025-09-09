#pragma once
#include <iostream>
#include <string>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "components.hpp"
#include "platform.hpp"

#include <GLFW/glfw3.h>

namespace UI::Platform {
	class Window {
		static std::shared_ptr<GLFWwindow> m_winPtr;
		static int m_winWidth;
		static int m_winHeight;
		static std::string m_winName;

		Window() = delete;
		Window(const Window&) = delete;
		Window(Window&&) = delete;
		~Window() = delete;

		Window& operator=(const Window&) = delete;
		Window& operator=(Window&&) = delete;

		static bool createContext();
		static void destroyContext();


		static void initImGui();
		static void destroyImGui();

		static bool init();
		static void destroy(GLFWwindow* window);

		static void glfw_error_callback(int error, const char* description);

	public:

		static bool shouldClose();
		static bool initProperties(int width, int height, const std::string& name);

		static bool beginFrame();
		static void endFrame();

		static std::shared_ptr<GLFWwindow> getWindow();
	};
}