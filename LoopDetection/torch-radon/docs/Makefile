SPHINXBUILD := sphinx-build
SPHINXPROJ := torch_radon
SOURCEDIR := source
BUILDDIR := build

.PHONY: help Makefile

%: Makefile
	TORCH_RADON_DOC_BUILD=1 $(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)"
