workspace "MyUI"
    kind "ConsoleApp"   -- <--- this makes it an executable
    configurations { "Debug", "Release" }
    platforms { "x86", "x86_64" }
    language "C++"
    cppdialect "C++20"
    location "./"

    -- Static runtime setting (off by default; adjust if needed)
    srunt = "off"

    -- Define Debug/Release settings (add symbols and optimizations)
    filter "configurations:Debug"
        defines { "DEBUG" }
        symbols "On"
    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "On"
    filter {}  -- clear filter

    -- Include subproject build scripts
    include "./Components/build.lua"
    include "./Vendor/build.lua"
    include "./MyUI/build.lua"

project "MyUIApp"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++20"
    location "%{wks.location}/MyUI"
    targetdir ("%{wks.location}/bin/")
    objdir ("%{wks.location}/obj/%{cfg.buildcfg}")
    files {
        "src/**.cpp",
        "src/**.hpp",
        "%{wks.location}/Vendor/implot/*.hpp",
        "%{wks.location}/Components/include/*.hpp",
        "%{wks.location}/Components/src/*.hpp",
        "%{wks.location}/Vendor/imgui/*.hpp",
        "%{wks.location}/Vendor/imgui/backends/*.hpp",
        "%{wks.location}/Vendor/imgui/misc/cpp/*.hpp",
        "%{wks.location}/Vendor/implot/*.hpp",
        "%{wks.location}/Vendor/libs/glfw/include/*.yeshpp"
    }
    includedirs {
        path.getabsolute("src"),
        path.getabsolute("../Components/include"),
        path.getabsolute("../Components/src"),
        path.getabsolute("../Vendor/imgui"),
        path.getabsolute("../Vendor/imgui/backends"),
        path.getabsolute("../Vendor/imgui/misc/cpp"),
        path.getabsolute("../Vendor/implot"),
        path.getabsolute("../Vendor/libs/glfw/include")
    }
    links { "Components", "imgui", "implot", "pthread", "dl", "GL", "X11", "glfw" }
    filter "system:windows"
        links { "opengl32", "glfw3" }
    filter "system:linux"
        links { "GL", "X11", "pthread", "dl", "glfw" }
    filter {}
