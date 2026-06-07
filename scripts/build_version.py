"""Inject git/build metadata into firmware builds."""

import datetime
import subprocess

Import("env")


def git_output(*args):
    try:
        return subprocess.check_output(
            ["git", *args],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return "unknown"


def git_is_dirty():
    try:
        subprocess.check_call(
            ["git", "diff", "--quiet", "--ignore-submodules", "--"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        subprocess.check_call(
            ["git", "diff", "--cached", "--quiet", "--ignore-submodules", "--"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return False
    except Exception:
        return True


def c_string(value):
    escaped = str(value).replace("\\", "\\\\").replace('"', '\\"')
    return '\\"{}\\"'.format(escaped)


git_sha = git_output("rev-parse", "--short=12", "HEAD")
git_branch = git_output("rev-parse", "--abbrev-ref", "HEAD")
dirty_suffix = "-dirty" if git_is_dirty() else ""
firmware_version = "{}-{}{}".format(git_branch, git_sha, dirty_suffix)
build_utc = datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"

env.Append(
    CPPDEFINES=[
        ("FIRMWARE_GIT_SHA", c_string(git_sha)),
        ("FIRMWARE_GIT_BRANCH", c_string(git_branch)),
        ("FIRMWARE_VERSION", c_string(firmware_version)),
        ("FIRMWARE_BUILD_UTC", c_string(build_utc)),
    ]
)
