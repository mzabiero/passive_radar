#pragma once
#include <iostream>
#include <string>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include <GLFW/glfw3.h>

namespace UI::Data {
	struct GlobalUIConfig {
		//font size
		//window movement only from titlebar
		//layout parameters
		inline static bool move_from_title{ true };
		//change to proper scale setting in the future
		inline static float font_scale{ 1.20f };
		inline static ImVec2 frame_padding{ 10, 3 };
		inline static ImVec2 item_spacing{ 10, 5 };
		inline static ImVec2 item_inn_spacing{ 4, 4 };
		inline static float scroll_size{ 20 };
		inline static float grab_size{ 20 };
		inline static float win_b{ 1 };
		inline static float child_b{ 1 };
		inline static float pop_b{ 1 };
		inline static float frame_b{ 1 };
		inline static float tab_b{ 0 };
		inline static float tab_bar_b{ 2 };

		inline static float scroll_round{ 0 };
		inline static float tab_round{ 0 };
		inline static ImVec2 cell_pad{ 5, 4 };
		inline static float sep_text_b{ 5 };
		inline static float dock_sp{ 6 };
		inline static ImGuiHoveredFlags tool_hov_del_sh{ ImGuiHoveredFlags_DelayShort };

		static void apply();
	};
}