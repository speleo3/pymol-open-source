appname ?= pymol
apppath ?= ~/Applications/$(appname).app
launcherbin = Contents/MacOS/$(appname)

PYTHON ?= $(abspath ../.venv/bin/python)

$(launcherbin): launcher.m
	mkdir -p $(dir $@)
	clang -fobjc-arc -framework Cocoa -DPYMOL_PYTHON_FALLBACK='"$(PYTHON)"' $< -o $@

install: $(launcherbin)
	mkdir -p $(apppath)
	cp -va Contents $(apppath)/
	sed 's/%APPNAME%/$(appname)/' Contents/Info.plist > $(apppath)/Contents/Info.plist
	$(MAKE) refresh

refresh:
	/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister -f $(apppath)

clean:
	$(RM) $(launcherbin)

# vi:noexpandtab:sw=4:ts=4
