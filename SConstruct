#!/usr/bin/env python
# GDExtension build

import os
import subprocess

GODOT_CPP_REF = "master"

if not os.path.isdir("godot-cpp"):
    print("godot-cpp/ not found, cloning {}...".format(GODOT_CPP_REF))
    try:
        subprocess.run(
            [
                "git", "clone",
                "--branch", GODOT_CPP_REF,
                "--depth", "1",
                "https://github.com/godotengine/godot-cpp.git",
                "godot-cpp",
            ],
            check=True,
        )
    except (subprocess.CalledProcessError, OSError):
        print("Error: could not clone godot-cpp. Clone it manually into godot-cpp/.")
        Exit(1)

env = SConscript("godot-cpp/SConstruct")

env.Append(CPPPATH=["src/"])

sources = Glob("src/*.cpp") + Glob("register_types.cpp")

if env["target"] in ["editor", "template_debug"]:
    doc_data = env.GodotCPPDocData("src/gen/doc_data.gen.cpp", source=Glob("doc_classes/*.xml"))
    sources.append(doc_data)

library_name = "libspreen{}".format(env["suffix"])
bin_dir = "demo/addons/spreen/bin"

if env["platform"] == "macos":
    macos_env = env.Clone()
    macos_env["SHLIBSUFFIX"] = ""
    library = macos_env.SharedLibrary(
        "{}/{}.framework/{}".format(bin_dir, library_name, library_name),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "{}/{}{}".format(bin_dir, library_name, env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
