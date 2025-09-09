#include "win.hpp"

namespace UI::Platform {
	std::shared_ptr<GLFWwindow> Window::m_winPtr{ nullptr };
	int Window::m_winWidth{ 1280 };
	int Window::m_winHeight{ 720 };
	std::string Window::m_winName{ "WINDOW" };

	bool Window::init() {
		if (!createContext()) return false;
		
		m_winPtr = std::shared_ptr<GLFWwindow>(glfwCreateWindow(m_winWidth, m_winHeight, m_winName.c_str(), nullptr, nullptr), &destroy);
		if (m_winPtr == nullptr) {
			destroyContext();
			return false;
		}
	
		glfwMakeContextCurrent(m_winPtr.get());
		glfwSwapInterval(1);

		initImGui();

		return true;
	}

	void Window::initImGui() {
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImPlot::CreateContext();
		ImGuiIO& io = ImGui::GetIO();
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
		io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

		ImGui::StyleColorsDark();

		ImPlot::GetInputMap().OverrideMod = ImGuiMod_None;
		UI::Data::GlobalUIConfig::apply();

		ImGui_ImplGlfw_InitForOpenGL(m_winPtr.get(), true);
		ImGui_ImplOpenGL3_Init("#version 130");
	}

	bool Window::beginFrame() {
		if (m_winPtr == nullptr && !init()) return false;
		glfwPollEvents();

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		return true;
	}

	void Window::endFrame() {
		if (m_winPtr == nullptr) return;
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(m_winPtr.get(), &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		m_winWidth = display_w;
		m_winHeight = display_h;
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(m_winPtr.get());
	}

	void Window::destroyImGui() {
		if (m_winPtr) return;
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot::DestroyContext();
		ImGui::DestroyContext();
	}

	void Window::destroyContext() {
		if (m_winPtr) return;
		glfwTerminate();
	}

	void Window::destroy(GLFWwindow* win_ptr) {
		glfwDestroyWindow(win_ptr);

		destroyImGui();
		destroyContext();

		std::cout << "Get rekt" << "\n";
	}

	bool Window::shouldClose() {
		if (m_winPtr == nullptr && !init()) return true;

		return glfwWindowShouldClose(m_winPtr.get());
	}

	std::shared_ptr<GLFWwindow> Window::getWindow() {
		if (m_winPtr == nullptr) init();
		return m_winPtr;
	}

	bool Window::createContext() {
		if (m_winPtr) return true;
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit()) return false;

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

		return true;
	}

	bool Window::initProperties(int width, int height, const std::string& name) {
		if (m_winPtr) return false;

		m_winWidth = width;
		m_winHeight = height;
		m_winName = name;

		return true;
	}

	void Window::glfw_error_callback(int error, const char* description) {
		std::cout << "GLFW Error %d: %s\n" << error << description << "\n";
	}


}