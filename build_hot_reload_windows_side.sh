#!/usr/bin/env bash
set -eu

buildDir="C:\\Users\\micro\\build"
buildDirOnWindowsFromLinuxPerspective="/mnt/c/Users/micro/build"
mkdir -p $buildDirOnWindowsFromLinuxPerspective

projectName=${PWD##*/}
projectName=${projectName:-/} # Correct for the case where pwd is "/" (shouldn't matter for this script)

targetDir="$buildDir\\$projectName"
targetDirOnWindowsFromLinuxPerspective="$buildDirOnWindowsFromLinuxPerspective/$projectName"
mkdir -p $targetDirOnWindowsFromLinuxPerspective

# I have to copy the source to windows side and build it there.
# The odin compiler has trouble building a dll when the files are under WSL paths.
# cp -r -t $targetDirOnWindowsFromLinuxPerspective .
rsync --recursive --update . $targetDirOnWindowsFromLinuxPerspective

echo "Copied project files to $targetDirOnWindowsFromLinuxPerspective"

# I then also have to run everything from the windows side otherwise the dll symbols don't resolve correctly
cmd.exe /c "cd /d $targetDir && build_hot_reload.bat run"
