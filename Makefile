# Detect OS
OS ?= $(shell uname -s | tr '[:upper:]' '[:lower:]')

ifeq ($(OS),Windows_NT)
    ARCH := $(PROCESSOR_ARCHITECTURE)
    BIN_SUFFIX := .exe
  # Scripts for windows are written in powershell and
  #	we invoke a wrapper batch script with cmd
  # syntax, which calls powershell for us and explicitly sets the
  # exit status so we terminate make on error.
	SCRIPT_EXT := .bat
	SHELL := cmd
else
	SCRIPT_EXT = .sh
	BIN_SUFFIX :=
	ARCH ?= $(shell uname -m)
endif

OUTPUT_NAME = orbbec-module$(BIN_SUFFIX)
BIN := build-conan/build/RelWithDebInfo/$(OUTPUT_NAME)
TAG_VERSION?=latest

# Docker image
HUB_USER := viam-modules/orbbec
BASE_NAME := viam-cpp-base-orin
BASE_TAG := 0.0.1

.PHONY: build lint setup conan-pkg

build: $(BIN)

$(BIN): conanfile.py src/* bin/*
ifeq ($(OS),Windows_NT)
	cmd /C "set ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR) && bin\build$(SCRIPT_EXT)"
else
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
	bin/build$(SCRIPT_EXT)
endif

clean:
	rm -rf packaging/appimages/deploy module.tar.gz

clean-all: clean
	rm -rf build-conan
	rm -rf tmp_cpp_sdk
	rm -rf tmp_cpp_sdk
	rm -rf venv
	rm -f orbbec-test-bin
	rm -f $(OUTPUT_NAME)

setup:
ifeq ($(OS),Windows_NT)
	cmd /C "set ORBBEC_SDK_VERSION=$(ORBBEC_SDK_VERSION) && set ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR) && bin\setup$(SCRIPT_EXT)"
else
	export ORBBEC_SDK_VERSION=$(ORBBEC_SDK_VERSION); \
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
	export OS=$(OS); \
	export ARCH=${ARCH}; \
	bin/setup$(SCRIPT_EXT)
endif

lint:
ifeq ($(OS),Windows_NT)
	@echo lint unsupported on windows
else
	./bin/run-clang-format.sh
endif

orbbec-test-bin:
	cd tests && \
	go test -c -o orbbec-test-bin ./ && \
	mv orbbec-test-bin ../

# Both the commands below need to source/activate the venv in the same line as the
# conan call because every line of a Makefile runs in a subshell

conan-pkg:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan create . \
	-o:a "viam-cpp-sdk/*:shared=False" \
	-s:a build_type=Release \
	-s:a compiler.cppstd=17 \
	--build=missing

module.tar.gz: conan-pkg meta.json
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install --requires=viam-orbbec/0.0.1 \
	-o:a "viam-cpp-sdk/*:shared=False" \
	-s:a build_type=Release \
	-s:a compiler.cppstd=17 \
	--deployer-package "&" \
	--envs-generation false

image-base:
	docker build -t $(BASE_NAME):$(BASE_TAG) \
		--platform=linux/arm64 \
		--memory=16g \
		-f etc/Dockerfile.ubuntu.jammy ./

# Pushes base docker image to github packages.
# Requires docker login to ghcr.io
push-base:
	docker tag $(BASE_NAME):$(BASE_TAG) ghcr.io/$(HUB_USER)/$(BASE_NAME):$(BASE_TAG) && \
	docker push ghcr.io/$(HUB_USER)/$(BASE_NAME):$(BASE_TAG)
