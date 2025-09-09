#pragma once
#include "data.hpp"
#include "stl.hpp"
#include "imgui.h"
#include "implot.h"

namespace UI::Components
{

	void draw_bg_window();
	void draw_tester(std::shared_ptr<Data::MyData> state);

}
