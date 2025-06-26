OS ?= $(shell uname -s | tr '[:upper:]' '[:lower:]')
ARCH ?= $(shell uname -m)
OUTPUT_NAME = orbbec-module
BIN := build-conan/build/RelWithDebInfo/orbbec-module
TAG_VERSION?=latest
APPIMAGE := packaging/appimages/deploy/$(OUTPUT_NAME)-$(TAG_VERSION)-$(ARCH).AppImage
ORBBEC_SDK_VERSION=v2.4.3
ORBBEC_SDK_TIMESTAMP=202505191331
ORBBEC_SDK_COMMIT=045a0e76
ORBBEC_SDK_DIR=OrbbecSDK_$(ORBBEC_SDK_VERSION)_$(ORBBEC_SDK_TIMESTAMP)_$(ORBBEC_SDK_COMMIT)_$(OS)_$(ARCH)

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
	export ORBBEC_SDK_VERSION=$(ORBBEC_SDK_VERSION); \
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
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
