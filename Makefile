OS ?= $(shell uname -s | tr '[:upper:]' '[:lower:]')
ARCH ?= $(shell uname -m)
OUTPUT_NAME = orbbec-module
BIN := build-conan/build/RelWithDebInfo/orbbec-module
TAG_VERSION?=latest
APPIMAGE := packaging/appimages/deploy/$(OUTPUT_NAME)-$(TAG_VERSION)-$(ARCH).AppImage

.PHONY: build lint setup appimage

module.tar.gz: $(APPIMAGE) meta.json
	cp $(APPIMAGE) $(OUTPUT_NAME).AppImage
	tar -czvf module.tar.gz $(OUTPUT_NAME).AppImage meta.json
	rm $(OUTPUT_NAME).AppImage

build: $(BIN)

$(BIN): lint conanfile.py src/* bin/*
	bin/build.sh

clean:
	rm -rf packaging/appimages/deploy

setup:
	bin/setup.sh

lint:
	./bin/run-clang-format.sh

$(APPIMAGE): $(BIN)
	export TAG_NAME=$(TAG_VERSION); \
	cd packaging/appimages && \
	mkdir -p deploy && \
	rm -f deploy/$(OUTPUT_NAME)* && \
	appimage-builder --recipe $(OUTPUT_NAME)-$(ARCH).yml
	cp ./packaging/appimages/$(OUTPUT_NAME)-$(TAG_VERSION)-$(ARCH).AppImage ./packaging/appimages/deploy/
