"""PlatformIO pre-build script that generates build_version.h.

The header is written into the build directory, not the source tree, so a
build never modifies tracked files. FW_VERSION comes from `git describe`
(tag, commits since tag, commit hash, and -dirty for uncommitted changes).
Set the KUKIRIN_FW_VERSION environment variable to override it.
"""
import datetime
import os
import subprocess

Import("env")  # noqa: F821 (provided by PlatformIO/SCons)


def firmware_version(project_dir):
    override = os.environ.get("KUKIRIN_FW_VERSION")
    if override:
        return override
    try:
        result = subprocess.run(
            ["git", "describe", "--tags", "--always", "--dirty"],
            cwd=project_dir, capture_output=True, text=True, check=True)
        return result.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


project_dir = env.subst("$PROJECT_DIR")  # noqa: F821
generated_dir = os.path.join(env.subst("$BUILD_DIR"), "generated")  # noqa: F821
os.makedirs(generated_dir, exist_ok=True)

fw_version = firmware_version(project_dir)
timestamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")

header = f"""#ifndef BUILD_VERSION_H
#define BUILD_VERSION_H

#define FW_VERSION "{fw_version}"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__
#define BUILD_TIMESTAMP "{timestamp}"

#endif // BUILD_VERSION_H
"""

with open(os.path.join(generated_dir, "build_version.h"), "w", encoding="utf-8", newline="\n") as f:
    f.write(header)

env.Append(CPPPATH=[generated_dir])  # noqa: F821
print(f"BuildVersion: {fw_version} | {timestamp}")
