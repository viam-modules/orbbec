OUTPUT_NAME = orbbec-module
BIN := build-conan/build/RelWithDebInfo/orbbec-module
TAG_VERSION?=latest

# Docker image
HUB_USER := viam-modules/orbbec
BASE_NAME := viam-cpp-base-orin
BASE_TAG := 0.0.1

.PHONY: build lint setup conan-pkg

build: $(BIN)

$(BIN): lint conanfile.py src/* bin/*
	bin/build.sh

clean:
	rm -rf packaging/appimages/deploy module.tar.gz

clean-all: clean
	rm -rf build-conan
	rm -rf tmp_cpp_sdk
	rm -rf venv
	rm -f orbbec-test-bin
	rm -f $(OUTPUT_NAME)

setup:
	bin/setup.sh

lint:
	./bin/run-clang-format.sh

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
