#include "tester.hpp"

namespace UI {

void Tester::run(std::function<void(std::shared_ptr<Data::MyData>)> draw)
{
	while (!Platform::Window::shouldClose())
	{
		if (!Platform::Window::beginFrame())
			return;

		Components::draw_bg_window();

		draw(this->state);
		this->update();

		Platform::Window::endFrame();
	}
}

void Tester::update()
{
	this->app.update(this->getState());
}

Tester::Tester()
{
	// TestedFitting to be initialized here

	// mock code to present the UI

	// to be deleted
	this->state = std::make_shared<Data::MyData>();
}

std::shared_ptr<Data::MyData> Tester::getState()
{
	return this->state;
}

} // namespace UI
