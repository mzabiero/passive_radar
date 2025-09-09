project "Components"
    kind "StaticLib"
    language "C++"

    location "%{wks.location}/Components"
    staticruntime(srunt)
    files {
        "%{prj.location}/**.cpp",
        "%{prj.location}/**.hpp",
    }

    removefiles { "%{prj.location}/implot_demos/**.*" }

    includedirs {
        path.getabsolute("include"),
        path.getabsolute("../Vendor/imgui"),
        path.getabsolute("../Vendor/imgui/backends"),
        path.getabsolute("../Vendor/imgui/misc/cpp"),
        path.getabsolute("../Vendor/implot"),
        path.getabsolute("../Vendor/libs/glfw/include")
    }
    targetdir ( "%{wks.location}/lib/" )
    objdir ( "%{wks.location}/obj/%{cfg.buildcfg}" )

    libdirs { "%{wks.location}/lib" }

    -- Windows 32/64-bit GLFW libs (unchanged for Windows)
    filter { "system:windows", "platforms:x86" }
        libdirs { "%{wks.location}/Vendor/libs/glfw/lib-vc2010-32" }
    filter { "system:windows", "platforms:x86_64" }
        libdirs { "%{wks.location}/Vendor/libs/glfw/lib-vc2010-64" }

    -- Configuration-specific settings
    filter { "configurations:Debug" }
        runtime "Debug"
    filter { "configurations:Release" }
        runtime "Release"

    -- Windows-specific linking
    filter { "system:windows" }
        ignoredefaultlibraries { "msvcrt" }
        links { "legacy_stdio_definitions", "opengl32", "glfw3", "imgui", "implot" }

    -- Linux-specific linking
    filter { "system:linux" }
        links { 
            "pthread",       -- POSIX threads
            "dl",            -- dynamic linking
            "GL",            -- OpenGL (libGL.so):contentReference[oaicite:6]{index=6} 
            "X11",           -- X Window System (for GLFW)
            "glfw",          -- GLFW library (use -lglfw, not -lglfw3 on Linux):contentReference[oaicite:7]{index=7}
            "imgui", 
            "implot"
        }
