#include <iostream>
#include "tester.hpp"

int main()
{
	UI::Platform::Window::initProperties(1920, 1080, "MyApp");

	UI::Tester t1{};

	t1.run(UI::Components::draw_tester);

	return 0;
}