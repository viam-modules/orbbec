# Detect OS

OS ?= $(shell uname -s | tr '[:upper:]' '[:lower:]')
ARCH ?= $(shell uname -m)

ifeq ($(OS),Windows_NT)
    BIN_SUFFIX := .exe
	SCRIPT_EXT := .ps1
else
	SCRIPT_EXT = .sh
endif

TAR_BIN_NAME = orbbec-module
OUTPUT_NAME = orbbec-module$(BIN_SUFFIX)
BIN := build-conan/build/RelWithDebInfo/$(OUTPUT_NAME)
TAG_VERSION?=latest
APPIMAGE := packaging/appimages/deploy/$(OUTPUT_NAME)-$(TAG_VERSION)-$(TARGET_ARCH).AppImage

ifeq ($(OS),darwin)
  ORBBEC_SDK_VERSION=v2.4.3
  ORBBEC_SDK_COMMIT=045a0e76
  ORBBEC_SDK_TIMESTAMP=202505192200
  ORBBEC_SDK_DIR = OrbbecSDK_$(ORBBEC_SDK_VERSION)_$(ORBBEC_SDK_TIMESTAMP)_$(ORBBEC_SDK_COMMIT)_macOS_beta
else ifeq ($(OS),linux)
   ORBBEC_SDK_VERSION=v2.4.8
   ORBBEC_SDK_COMMIT=ec8e3469
   ORBBEC_SDK_DIR = OrbbecSDK_$(ORBBEC_SDK_VERSION)_$(ORBBEC_SDK_TIMESTAMP)_$(ORBBEC_SDK_COMMIT)_linux_$(TARGET_ARCH)
  ifeq ($(ARCH),x86_64)
    ORBBEC_SDK_TIMESTAMP = 202507031325
   else ifeq ($(ARCH), aarch64)
	ORBBEC_SDK_TIMESTAMP = 202507031330
   endif
else ifeq ($(OS),Windows_NT)
  ORBBEC_SDK_VERSION=v2.4.8
  ORBBEC_SDK_COMMIT=ec8e346
  ORBBEC_SDK_TIMESTAMP=202507032159
  ORBBEC_SDK_DIR = OrbbecSDK_$(ORBBEC_SDK_VERSION)_$(ORBBEC_SDK_TIMESTAMP)_$(ORBBEC_SDK_COMMIT)_win_x64
else
  $(error Unsupported OS: $(OS))
endif


.PHONY: build setup appimage

ifeq ($(OS),linux)
module.tar.gz: $(APPIMAGE) meta.json
else
module.tar.gz: $(BIN) meta.json
endif
ifeq ($(OS),linux)
	mv $(APPIMAGE) $(OUTPUT_NAME)
	tar -czvf module.tar.gz \
		$(OUTPUT_NAME) \
		meta.json \
		./first_run.sh \
		./install_udev_rules.sh \
		./99-obsensor-libusb.rules
else ifeq ($(OS),darwin)
 # update the rpath https://en.wikipedia.org/wiki/Rpath to look for the orbbec dynamic library
 # in the ./lib folder in the folder produced by the module.tar.gz on the computer that runs the module
 # rather than the build directory of the build machine
	if otool -l $(BIN) | grep -A2 LC_RPATH | grep -q "path @executable_path/lib"; then \
		install_name_tool -delete_rpath $(ORBBEC_SDK_DIR)/lib $(BIN); \
	fi
	install_name_tool -add_rpath @executable_path/lib $(BIN);
	tar -czvf module.tar.gz \
	meta.json \
    first_run.sh \
	-C $(ORBBEC_SDK_DIR) lib/ \
    -C ../$(dir $(BIN)) $(TAR_BIN_NAME)
else ifeq ($(OS),Windows_NT)
	cp "$(BIN)" "$(TAR_BIN_NAME)" > nul
	tar -czvf module.tar.gz \
	meta.json \
	-C .\$(ORBBEC_SDK_DIR) lib \
	-C bin OrbbecSDK.dll extensions \
    $(TAR_BIN_NAME)
endif

build: $(BIN)

$(BIN): conanfile.py src/* bin/*
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
	export TARGET_OS=$(OS); \
	bin/build$(SCRIPT_EXT)

clean:
	rm -rf packaging/appimages/deploy module.tar.gz

clean-all: clean
	rm -rf build-conan
	rm -rf tmp_cpp_sdk
	rm -rf venv
	rm -f orbbec-test-bin
	rm -f $(OUTPUT_NAME)

setup:
	export ORBBEC_SDK_VERSION=$(ORBBEC_SDK_VERSION); \
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
	export OS=$(OS); \
	export ARCH=${SOURCE_ARCH}; \
	export TARGET_OS=$(OS); \
	bin/setup$(SCRIPT_EXT)

lint:
	./bin/run-clang-format.sh

orbbec-test-bin:
	cd tests && \
	go test -c -o orbbec-test-bin ./ && \
	mv orbbec-test-bin ../

$(APPIMAGE): $(BIN)
	export TAG_NAME=$(TAG_VERSION); \
	cd packaging/appimages && \
	mkdir -p deploy && \
	rm -f deploy/$(OUTPUT_NAME)* && \
	appimage-builder --recipe $(OUTPUT_NAME)-$(TARGET_ARCH).yml
	cp ./packaging/appimages/$(OUTPUT_NAME)-$(TAG_VERSION)-$(TARGET_ARCH).AppImage ./packaging/appimages/deploy/
