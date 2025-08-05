OS ?= $(shell uname -s | tr '[:upper:]' '[:lower:]')
ARCH ?= $(shell uname -m)
OUTPUT_NAME = orbbec-module
BIN := build-conan/build/RelWithDebInfo/orbbec-module
TAG_VERSION?=latest
APPIMAGE := packaging/appimages/deploy/$(OUTPUT_NAME)-$(TAG_VERSION)-$(ARCH).AppImage

.PHONY: build lint setup appimage

module.tar.gz: deploy meta.json
	tar -czvf module.tar.gz -C module-deploy .

build: $(BIN)

$(BIN): lint conanfile.py src/* bin/*
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
	bin/build.sh

clean:
	rm -rf packaging/appimages/deploy module.tar.gz

clean-all: clean
	rm -rf build-conan
	rm -rf module-deploy
	rm -rf tmp_cpp_sdk
	rm -rf venv
	rm -f orbbec-test-bin
	rm -f $(OUTPUT_NAME)

setup:
	export ORBBEC_SDK_VERSION=$(ORBBEC_SDK_VERSION); \
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
	export OS=$(OS); \
	export ARCH=${ARCH}; \
	bin/setup.sh

lint:
	./bin/run-clang-format.sh

orbbec-test-bin:
	cd tests && \
	go test -c -o orbbec-test-bin ./ && \
	mv orbbec-test-bin ../

deploy: $(BIN)
	conan install --requires=viam-orbbec/0.0.1 \
	--deployer-package "&" --envs-generation false \
	--deployer-folder module-deploy
