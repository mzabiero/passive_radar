-- include "utils.lua"



workspace "MyUI"
	kind "ConsoleApp"   -- <--- this makes it an executable
	configurations { "Debug", "Release" }
	platforms { "x86", "x86_64" }
	language "C++"
	cppdialect "C++20"

	location "./"

	srunt = "off"


	include "./Components/build.lua"
	include "./Vendor/build.lua"
	include "./MyUI/build.lua"


