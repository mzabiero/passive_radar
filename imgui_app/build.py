import os
import sys
import subprocess

if __name__ == "__main__":
    if sys.platform.startswith("win"):
        premake_cmd = "premake5.exe"
    else:
        premake_cmd = "./premake5"  # explicitly call the local binary

    subprocess.run([premake_cmd, "gmake"])
