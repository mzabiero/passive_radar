#pragma once
// NS to be included
#include "includes.hpp"

// this class is to be responsible for managing fitting and other stuff
// it is a field of Tester class and its update function is called on every
// frame after UI is rendered and its state changes become valid
class MyApp
{
	// fitting fields to be declared

public:
	// initialization of fitting
	MyApp();

	// check for changes in the UI state and run procedures when needed
	void update(std::shared_ptr<UI::Data::MyData> state);
};