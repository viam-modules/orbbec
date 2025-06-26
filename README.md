# Module orbbec-camera 

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
| `serial_number` | string | **Required** | The serial number of the specific Orbbec camera to use. To find your camera's serial number, the serial number of each plugged-in and available RealSense camera will be logged on module startup.  |

#### Example Configuration

```json
{
  "serial_number": "XXXXXXXXXXX"
}
```

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
