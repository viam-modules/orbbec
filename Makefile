OS ?= $(shell uname -s | tr '[:upper:]' '[:lower:]')
ARCH ?= $(shell uname -m)
OUTPUT_NAME = orbbec-module
BIN := build-conan/build/RelWithDebInfo/orbbec-module
TAG_VERSION?=latest
APPIMAGE := packaging/appimages/deploy/$(OUTPUT_NAME)-$(TAG_VERSION)-$(ARCH).AppImage
ORBBEC_SDK_VERSION=v2.4.3
ORBBEC_SDK_COMMIT=045a0e76

ifeq ($(OS),darwin)
  ORBBEC_SDK_TIMESTAMP = 202505192200
  ORBBEC_SDK_DIR = OrbbecSDK_$(ORBBEC_SDK_VERSION)_$(ORBBEC_SDK_TIMESTAMP)_$(ORBBEC_SDK_COMMIT)_macOS_beta
else ifeq ($(OS),linux)
  ifeq ($(ARCH),x86_64)
    ORBBEC_SDK_TIMESTAMP = 202505191331
    ORBBEC_SDK_DIR = OrbbecSDK_$(ORBBEC_SDK_VERSION)_$(ORBBEC_SDK_TIMESTAMP)_$(ORBBEC_SDK_COMMIT)_linux_$(ARCH)
  else
    $(error Unsupported architecture: $(ARCH))
  endif
else
  $(error Unsupported OS: $(OS))
endif


.PHONY: build lint setup appimage

module.tar.gz: $(BIN) meta.json
ifeq ($(OS),linux)
	mv $(APPIMAGE) $(OUTPUT_NAME)
	tar -czvf module.tar.gz \
		$(OUTPUT_NAME).AppImage \
		meta.json \
		./first_run.sh \
		./install_udev_rules.sh \
		./99-obsensor-libusb.rules
else ifeq ($(OS),darwin)
	install_name_tool -change $(ORBBEC_SDK_DIR)/lib/libOrbbecSDK.2.dylib @executable_path/lib/libOrbbecSDK.2.dylib $(BIN)
	if ! otool -l $(BIN) | grep -A2 LC_RPATH | grep -q "@executable_path/lib"; then \
		install_name_tool -add_rpath @executable_path/lib $(BIN); \
	fi
	tar -czvf module.tar.gz \
	meta.json \
    first_run.sh \
	-C $(ORBBEC_SDK_DIR)/lib libOrbbecSDK.2.dylib \
    -C ../../$(dir $(BIN)) $(OUTPUT_NAME)
endif

build: $(BIN)

$(BIN): lint conanfile.py src/* bin/*
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
	bin/build.sh

clean:
	rm -rf packaging/appimages/deploy module.tar.gz

setup:
	export ORBBEC_SDK_VERSION=$(ORBBEC_SDK_VERSION); \
	export ORBBEC_SDK_DIR=$(ORBBEC_SDK_DIR); \
	export OS=$(OS); \
	export ARCH=${ARCH}; \
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
