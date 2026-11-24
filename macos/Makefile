appname ?= pymol
apppath ?= $(HOME)/Applications/$(appname).app
launcherbin = Contents/MacOS/$(appname)
venvpath = $(apppath)/Contents/venv

PYTHON ?= /opt/homebrew/bin/python3

$(launcherbin): launcher.m
	mkdir -p $(dir $@)
	clang -fobjc-arc -framework Cocoa -DPYMOL_PYTHON_FALLBACK='"$(PYTHON)"' $< -o $@

install: $(launcherbin)
	mkdir -p $(apppath)
	cp -va Contents $(apppath)/
	sed 's/%APPNAME%/$(appname)/' Contents/Info.plist > $(apppath)/Contents/Info.plist
	$(MAKE) refresh

$(apppath):
	$(MAKE) install

$(venvpath): | $(apppath)
	$(PYTHON) -m venv --system-site-packages $(venvpath)

install-venv: | $(venvpath)

install-venv-extras: | $(venvpath)
	$(venvpath)/bin/pip install \
		git+https://github.com/speleo3/pymol-psico \
		biopython rdkit mdanalysis

install-venv-extras-prolif: | $(venvpath)
	# fails with gemmi for python3.15
	$(venvpath)/bin/pip install prolif

refresh:
	/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister -f $(apppath)

clean:
	$(RM) $(launcherbin)

# vi:noexpandtab:sw=4:ts=4
