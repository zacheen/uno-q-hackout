# Git Bash rewrites POSIX-looking adb arguments, such as the push target and
# TMPDIR=/tmp, into Windows paths before adb.exe sees them.
export MSYS_NO_PATHCONV := 1
# GNU Make reads GNUmakefile before Makefile, so plain make gets the export above.
include Makefile
