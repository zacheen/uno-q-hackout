# Git Bash rewrites POSIX-looking adb arguments, such as the push target and
# TMPDIR=/tmp, into Windows paths before adb.exe sees them.
export MSYS_NO_PATHCONV := 1
# GNU Make reads GNUmakefile before Makefile, so plain make gets the export above.
include Makefile

.PHONY: pull-logs

# Copies data/runs from the make-flashed app. An App Lab import is assumed to
# live elsewhere (unverified).
pull-logs:
	mkdir -p build
	"$(ADB)" $(ADB_DEVICE) exec-out 'tar -C "$$HOME/ArduinoApps/uno-q-robocar-cli/data" -cf - runs' | tar -xf - -C build
