SHELL = /bin/bash -o pipefail

# make <target> YES=1 passes -y: install missing prerequisites without asking.
define builder
  ./build.sh $(if $(YES),-y) $1
endef

.PHONY: upload uploadfs build all

upload:
	$(call builder, -u)

uploadfs:
	$(call builder, -f)

build:
	$(call builder, -b)

all:
	$(call builder, -a)

zip:
	$(call builder, -z)

%-lwm:
	$(call builder, -p $(shell echo $* | tr '[:lower:]' '[:upper:]') -g)

pico-de-coco:
	make -C pico/coco/build

clean:
	$(call builder, -c)

# ---------------------------------------------------------------------------
# Source-tree policy: building must never change the source tree.
# See tests/check_gitignore_policy.py for what is checked and how to exempt
# vendor code (.gitignore-check-exempt).
#
#   make check-gitignore         static: no .gitignore rule may cover source dirs
#   make check-tree-baseline     snapshot what is already in the tree (run BEFORE a build)
#   make check-tree              after a build: fail if it left files in source dirs
#
#   make check-tree-baseline build check-tree     (targets run in the order given)
#   make check-tree CHECK_ARGS="--exempt 'data/BUILD_*'"   (extra options, no commit needed)
# ---------------------------------------------------------------------------
PYTHON ?= python3
CHECK_GITIGNORE = $(PYTHON) tests/check_gitignore_policy.py
# Kept inside .git/ so it is per-checkout and never shows up in git status.
TREE_BASELINE ?= $(shell git rev-parse --git-path tree-baseline)

.PHONY: check-gitignore check-tree-baseline check-tree

check-gitignore:
	$(CHECK_GITIGNORE) $(CHECK_ARGS)

check-tree-baseline:
	$(CHECK_GITIGNORE) --check-files --save-baseline $(TREE_BASELINE) $(CHECK_ARGS)

check-tree:
	$(CHECK_GITIGNORE) --check-files $(if $(wildcard $(TREE_BASELINE)),--baseline $(TREE_BASELINE)) $(CHECK_ARGS)
