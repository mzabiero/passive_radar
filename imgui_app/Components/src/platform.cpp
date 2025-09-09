#include "platform.hpp"

namespace UI::Platform {
	void Window::initProperties(int, int, const std::string&) {}
	bool Window::shouldClose() { return false; }
	bool Window::beginFrame() { return true; }
	void Window::endFrame() {}
}

namespace UI::Data {
	void GlobalUIConfig::apply() {
		ImGuiIO& io = ImGui::GetIO();
		ImGuiStyle& style = ImGui::GetStyle();

		io.ConfigWindowsMoveFromTitleBarOnly = move_from_title;
		io.FontGlobalScale = font_scale;
		
		style.FramePadding = frame_padding;
		style.ItemSpacing = item_spacing;
		style.ItemInnerSpacing = item_inn_spacing;
		style.ScrollbarSize = scroll_size;
		style.GrabMinSize = grab_size;
		style.WindowBorderSize = win_b;
		style.ChildBorderSize = child_b;
		style.PopupBorderSize = pop_b;
		style.FrameBorderSize = frame_b;
		style.TabBorderSize = tab_b;
		style.TabBarBorderSize = tab_bar_b;

		style.ScrollbarRounding = scroll_round;
		style.TabRounding = tab_round;
		style.CellPadding = cell_pad;
		style.SeparatorTextBorderSize = sep_text_b;
		style.DockingSeparatorSize = dock_sp;
		style.HoverFlagsForTooltipMouse |= tool_hov_del_sh;
	}
}