#pragma once
#include "includes.hpp"
#include "MyApp.hpp"

namespace UI {

class Tester
{
	std::shared_ptr<Data::MyData> state{nullptr};
	MyApp app{};
	void update();

public:
	Tester();

	Tester(const Tester &t) = delete;
	Tester &operator=(const Tester &t) = delete;

	Tester(Tester &&t) noexcept = default;
	Tester &operator=(Tester &&t) noexcept = default;
	~Tester() = default;

	std::shared_ptr<Data::MyData> getState();

	void run(std::function<void(std::shared_ptr<Data::MyData>)> draw);
};

} // namespace UI