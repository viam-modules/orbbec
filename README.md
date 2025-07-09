# Module orbbec

Use [Orbbec cameras](https://www.orbbec.com/products/) compatible with [v2 of the Orbbec CPP SDK](https://github.com/orbbec/OrbbecSDK_v2/).
This module provides access to the color and depth sensors and creates pointclouds from them.

## Model viam:orbbec:astra2

Use [Orbbec cameras](https://www.orbbec.com/products/structured-light-camera/astra-2/).

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "serial_number": "XXXXXXXXXXX"
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `serial_number` | string | **Required** | The serial number of the specific Orbbec camera to use. This number is printed on the device. The serial number of each plugged-in and available orbbec camera will be logged on module startup.  |

## Troubleshooting

**Unable to connect to camera**

The `first_run.sh` script included in this module should automatically install the `udev` rules for connecting to the camera on Linux devices.
If there is an issue, try copying `99-obsensor-libusb.rules` in the root directory of this repo to `/etc/udev/rules.d/` on the Viam machine and calling the following command on the system:

```
sudo udevadm control --reload-rules && sudo udevadm trigger
```


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
