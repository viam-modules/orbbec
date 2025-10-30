# Module orbbec

Use [Orbbec cameras](https://www.orbbec.com/products/) compatible with [v2 of the Orbbec CPP SDK](https://github.com/orbbec/OrbbecSDK_v2/).
This module provides access to the color and depth sensors and creates pointclouds from them.

## Supported Platforms
- **Darwin ARM64**
- **Linux x64**: Ubuntu 24.04
- **Linux ARM64**: NVIDIA Jetson AGX Orin, NVIDIA Jetson Orin NX, NVIDIA Jetson Orin Nano, NVIDIA Jetson AGX Xavier, NVIDIA Jetson Xavier NX
- **Windows AMD64**: Windows 11

## Model viam:orbbec:astra2

[Official Orbbec Astra 2 Camera Webpage](https://www.orbbec.com/products/structured-light-camera/astra-2/).

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "serial_number": "AARY14100EF",
  "sensors": {
    "depth": {
      "height": 1200,
      "width": 1600,
      "format": "Y16"
    },
    "color": {
      "width": 1920,
      "height": 1080,
      "format": "RGB"
    }
  }
}
```
#### Configuration Attributes

The following attributes are available for the Astra 2 model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `serial_number` | string | **Required** | The serial number of the specific Orbbec camera to use. This number is printed on the device. The serial number of each plugged-in and available orbbec camera will be logged on module startup.  |
|`sensors` | struct | **Optional** | The configuration of the color and depth sensors |

#### `sensor` attributes:
| Name | Type | Inclusion | Description |
|------|------|-----------|-------------|
| `height` | string | **Optional** | Native camera sensor height in pixels  |
| `width` | string | **Optional** | Native camera sensor width in pixels |
| `format` | string | **Optional** | Native camera format |

#### `sensor` formats
| Sensor | Formats |
|--------|---------|
| `color` | `MJPG`, `RGB` |
| `depth` | `Y16` |


#### `width`/`height` available combinations
| Color | Depth |
|-------|-------|
| `1920x1080` | `1600x1200`, `800X600`, `400X300` | 
| `1440X1080` | `1600x1200`, `800X600`, `400X300` | 
| `1280X720` | `1600x1200`, `800X600`, `400X300` | 
| `800X600` | `800X600`, `400X300` | 
| `640X480` | `800X600`, `400X300` | 
| `640X360` | `800X600`, `400X300` | 

## Model viam:orbbec:gemini_335le

[Official Orbbec Gemini 335Le Camera Webpage](https://www.orbbec.com/gemini-335le/).

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "serial_number": "AARY14100EF",
  "sensors": {
    "depth": {
      "height": 1200,
      "width": 1600,
      "format": "Y16"
    },
    "color": {
      "width": 1920,
      "height": 1080,
      "format": "MJPG"
    }
  }
}
```
#### Configuration Attributes

The following attributes are available for the Gemini 335Le model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `serial_number` | string | **Required** | The serial number of the specific Orbbec camera to use. This number is printed on the device. The serial number of each plugged-in and available orbbec camera will be logged on module startup.  |
|`sensors` | struct | **Optional** | The configuration of the color and depth sensors |

#### `sensor` attributes:
| Name | Type | Inclusion | Description |
|------|------|-----------|-------------|
| `height` | string | **Optional** | Native camera sensor height in pixels  |
| `width` | string | **Optional** | Native camera sensor width in pixels |
| `format` | string | **Optional** | Native camera format |

#### `sensor` formats
| Sensor | Formats |
|--------|---------|
| `color` | `MJPG`|
| `depth` | `Y16` |


#### `width`/`height` available combinations
| Color | Depth |
|-------|-------|
| `1280X800` | `1280X800`, `848X530`, `640X400`, `424X266`, `320X200` |
| `848X530` | `640X400`, `424X266`, `320X200` |
| `640X400` | `640X400`, `424X266`, `320X200` |
| `640X480` | `640X480` |


## Attributes
A call to get_attributes will return the camera attributes in [this struct](https://github.com/viamrobotics/viam-cpp-sdk/blob/43deea420f572e6b61b6fbd519e09b2520f05676/src/viam/sdk/components/camera.hpp#L58)

Bear in mind that the distortion parameters contained in that struct are not named, i.e. they are contained in a vector of doubles. So they must be parsed following the order in which they are being stored, which is as follows:

|index|parameter|
|-----|---------|
|  0  |   p1    |
|  1  |   p2    |
|  2  |   k1    |
|  3  |   k2    |
|  4  |   k3    |
|  5  |   k4    |
|  6  |   k5    |
|  7  |   k6    |

## DoCommand
You can use DoCommand to upgrade the firmware of your device to the required version.

Update the astra 2 firmware to v2.8.20 and gemini_335le to v1.5.55. If running on macOS, you must manually unplug and replug the device after this returns.
**WARNING**: Do not unplug the device while the firmware update is in progress.

{
  "update_firmware": ""
}

## Model viam:orbbec:discovery

This model is used to locate orbbec cameras connected to your machine. No configuration is needed.
Expand the test card or look at the discovery control card to obtain configurations for all connected Orbbecs.

## Troubleshooting

**Unable to connect to camera**

The `first_run.sh` script included in this module should automatically install the `udev` rules for connecting to the camera on Linux devices.
If there is an issue, try copying `99-obsensor-libusb.rules` in the root directory of this repo to `/etc/udev/rules.d/` on the Viam machine and calling the following command on the system:

```
sudo udevadm control --reload-rules && sudo udevadm trigger
```

When running on macos, `viam-server` must be run with `sudo` in order to have sufficient permissions to call `uvc_open`.


## Setup
```bash
canon make setup
```

## Build Module
```bash
canon make
```

## Build (Development)
```bash
canon make build
```

## Testing

This module includes an automated test binary to verify camera functionality.

### Running the test

1.  **Build the module and test binary:**
    ```bash
    canon make setup
    canon make build
    canon make orbbec-test-bin
    ```

2.  **Run the test:**
    The test binary requires the path to the module and the camera's serial number. Note that `make build` places the module binary in the `build-conan/build/RelWithDebInfo` directory.

    ```bash
    ./orbbec-test-bin --module <PATH_TO_LOCAL_MODULE> --serial_number <YOUR_CAMERA_SERIAL_NUMBER>
    ```

    -   `--module`: Path to the module executable.
    -   `--serial_number`: The serial number of your Orbbec camera. This is written on a sticker on the Astra 2 devices and is also logged on module startup if you are unsure what it is.
