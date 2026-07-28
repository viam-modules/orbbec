package tests

import (
	"context"
	"fmt"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"
	"go.viam.com/utils/rpc"
)

const (
	componentName       = "astra2-cam"
	testTimeoutDuration = 5 * time.Second
	testTimestampBuffer = 1 * time.Second // buffer to account for driver clock skew
	testTickDuration    = 100 * time.Millisecond
)

func TestCameraServer(t *testing.T) {
	timeoutCtx, cancel := context.WithTimeout(context.Background(), time.Minute)
	defer cancel()

	t.Run("With a configured robot", func(t *testing.T) {
		configString := fmt.Sprintf(`
			{
			"network": {
				"bind_address": "0.0.0.0:90831",
				"insecure": true
			},
			"components": [
				{
				"name": "%v",
				"model": "viam:orbbec:astra2",
				"type": "camera",
				"namespace": "rdk",
				"attributes": {
					"serial_number": "%v"
				},
				"depends_on": []
				}
			],
			"modules": [
				{
				"type": "local",
				"name": "local-orbbec-mod",
				"executable_path": "%v"
				}
			]
			}
		`, componentName, *serialNumber, absModulePath)
		myRobot, err := setUpViamServer(timeoutCtx, configString, "astra2-test", t)
		if err != nil {
			t.Fatalf("Failed to set up viam server: %v", err)
		}
		defer myRobot.Close(timeoutCtx)

		cam, err := camera.FromRobot(myRobot, componentName)
		if err != nil {
			t.Fatalf("Failed to get camera from robot: %v", err)
		}
		defer cam.Close(timeoutCtx)

		// Single-source retrieval goes through Images with a filter rather than the image API:
		// viam-cpp-sdk removed Camera::get_image and the CameraServer no longer serves a GetImage
		// RPC at all, so there is nothing behind DecodeImageFromCamera for this module to answer.
		t.Run("Get images method (color only)", func(t *testing.T) {
			timeout := time.After(testTimeoutDuration)
			tick := time.Tick(testTickDuration)
			for {
				select {
				case <-timeout:
					t.Fatal("timed out waiting for Get images method (color only)")
				case <-tick:
					images, _, err := cam.Images(timeoutCtx, []string{"color"}, nil)
					if err != nil || len(images) < 1 {
						continue
					}
					test.That(t, len(images), test.ShouldEqual, 1)
					test.That(t, images[0].SourceName, test.ShouldEqual, "color")
					return
				}
			}
		})

		t.Run("Get images method (two images)", func(t *testing.T) {
			timeout := time.After(testTimeoutDuration)
			tick := time.Tick(testTickDuration)
			for {
				select {
				case <-timeout:
					t.Fatal("timed out waiting for Get images method (two images)")
				case <-tick:
					timeBeforeCall := time.Now()
					images, metadata, err := cam.Images(timeoutCtx, nil, nil)
					if err != nil || len(images) < 2 {
						continue
					}
					test.That(t, len(images), test.ShouldEqual, 2)
					test.That(t, metadata.CapturedAt, test.ShouldHappenAfter, timeBeforeCall.Add(-testTimestampBuffer))
					return
				}
			}
		})

		t.Run("Reconfigure camera", func(t *testing.T) {
			timeout := time.After(testTimeoutDuration)
			tick := time.Tick(testTickDuration)
			for {
				select {
				case <-timeout:
					t.Fatal("timed out waiting for Reconfigure camera")
				case <-tick:
					cfg := resource.Config{
						Attributes: utils.AttributeMap{
							"debug": true,
						},
					}
					builtIn, ok := cam.(resource.BuiltInResource)
					if !ok {
						t.Skip("camera does not implement BuiltInResource (Reconfigure)")
						return
					}
					err := builtIn.BuiltInReconfigure(timeoutCtx, resource.Dependencies{}, cfg)
					if err != nil {
						continue
					}
					test.That(t, err, test.ShouldBeNil)
					return
				}
			}
		})

		t.Run("Get point cloud method", func(t *testing.T) {
			timeout := time.After(testTimeoutDuration)
			tick := time.Tick(testTickDuration)
			for {
				select {
				case <-timeout:
					t.Fatal("timed out waiting for Get point cloud method")
				case <-tick:
					pc, err := cam.NextPointCloud(timeoutCtx, nil)
					if err != nil {
						continue
					}
					test.That(t, err, test.ShouldBeNil)
					test.That(t, pc, test.ShouldNotBeNil)
					test.That(t, pc.Size(), test.ShouldBeBetween, 0, rpc.MaxMessageSize)
					return
				}
			}
		})

		t.Run("Get properties method", func(t *testing.T) {
			props, err := cam.Properties(timeoutCtx)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, props, test.ShouldNotBeNil)
			test.That(t, props.SupportsPCD, test.ShouldBeTrue)
			test.That(t, props.IntrinsicParams, test.ShouldNotBeNil)
		})

		t.Run("Get geometries method", func(t *testing.T) {
			geometries, err := cam.Geometries(timeoutCtx, nil)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, len(geometries), test.ShouldEqual, 1)

			// The pose is the center of the camera body relative to the RGB sensor's optical
			// center, which is the frame the module reports its data in. See the derivation on
			// ASTRA2_CONFIG in src/module/orbbec.cpp.
			expected, err := spatialmath.NewBox(
				spatialmath.NewPoseFromPoint(r3.Vector{X: 19.00, Y: 5.47, Z: -19.32}),
				r3.Vector{X: 144.54, Y: 45.35, Z: 38.64},
				"box",
			)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, spatialmath.GeometriesAlmostEqual(geometries[0], expected), test.ShouldBeTrue)
		})

		t.Run("DoCommand get_device_properties", func(t *testing.T) {
			resp, err := cam.DoCommand(timeoutCtx, map[string]interface{}{
				"get_device_properties": "",
			})
			test.That(t, err, test.ShouldBeNil)
			test.That(t, resp, test.ShouldNotBeNil)
			_, hasError := resp["error"]
			test.That(t, hasError, test.ShouldBeFalse)
		})

		t.Run("DoCommand set_device_property and verify", func(t *testing.T) {
			// Get a writable property to test with
			resp, err := cam.DoCommand(timeoutCtx, map[string]interface{}{
				"get_device_property": "OB_PROP_COLOR_BRIGHTNESS_INT",
			})
			test.That(t, err, test.ShouldBeNil)
			test.That(t, resp, test.ShouldNotBeNil)
			originalValue, ok := resp["current"].(float64)
			test.That(t, ok, test.ShouldBeTrue)

			// Set to a new value
			newValue := originalValue + 1
			resp, err = cam.DoCommand(timeoutCtx, map[string]interface{}{
				"set_device_property": map[string]interface{}{
					"OB_PROP_COLOR_BRIGHTNESS_INT": newValue,
				},
			})
			test.That(t, err, test.ShouldBeNil)
			_, hasError := resp["error"]
			test.That(t, hasError, test.ShouldBeFalse)

			// Verify the value was set
			resp, err = cam.DoCommand(timeoutCtx, map[string]interface{}{
				"get_device_property": "OB_PROP_COLOR_BRIGHTNESS_INT",
			})
			test.That(t, err, test.ShouldBeNil)
			updatedValue, ok := resp["current"].(float64)
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, updatedValue, test.ShouldEqual, newValue)

			// Restore original value
			_, err = cam.DoCommand(timeoutCtx, map[string]interface{}{
				"set_device_property": map[string]interface{}{
					"OB_PROP_COLOR_BRIGHTNESS_INT": originalValue,
				},
			})
			test.That(t, err, test.ShouldBeNil)
		})

		t.Run("DoCommand set_device_properties and verify", func(t *testing.T) {
			// Get current values for two writable properties
			resp, err := cam.DoCommand(timeoutCtx, map[string]interface{}{
				"get_device_property": "OB_PROP_COLOR_BRIGHTNESS_INT",
			})
			test.That(t, err, test.ShouldBeNil)
			originalBrightness, ok := resp["current"].(float64)
			test.That(t, ok, test.ShouldBeTrue)

			resp, err = cam.DoCommand(timeoutCtx, map[string]interface{}{
				"get_device_property": "OB_PROP_COLOR_CONTRAST_INT",
			})
			test.That(t, err, test.ShouldBeNil)
			originalContrast, ok := resp["current"].(float64)
			test.That(t, ok, test.ShouldBeTrue)

			// Set both properties at once
			newBrightness := originalBrightness + 1
			newContrast := originalContrast + 1
			resp, err = cam.DoCommand(timeoutCtx, map[string]interface{}{
				"set_device_properties": map[string]interface{}{
					"OB_PROP_COLOR_BRIGHTNESS_INT": map[string]interface{}{"current": newBrightness},
					"OB_PROP_COLOR_CONTRAST_INT":   map[string]interface{}{"current": newContrast},
				},
			})
			test.That(t, err, test.ShouldBeNil)
			_, hasError := resp["error"]
			test.That(t, hasError, test.ShouldBeFalse)

			// Verify both values were set
			resp, err = cam.DoCommand(timeoutCtx, map[string]interface{}{
				"get_device_property": "OB_PROP_COLOR_BRIGHTNESS_INT",
			})
			test.That(t, err, test.ShouldBeNil)
			updatedBrightness, ok := resp["current"].(float64)
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, updatedBrightness, test.ShouldEqual, newBrightness)

			resp, err = cam.DoCommand(timeoutCtx, map[string]interface{}{
				"get_device_property": "OB_PROP_COLOR_CONTRAST_INT",
			})
			test.That(t, err, test.ShouldBeNil)
			updatedContrast, ok := resp["current"].(float64)
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, updatedContrast, test.ShouldEqual, newContrast)

			// Restore original values
			_, err = cam.DoCommand(timeoutCtx, map[string]interface{}{
				"set_device_properties": map[string]interface{}{
					"OB_PROP_COLOR_BRIGHTNESS_INT": map[string]interface{}{"current": originalBrightness},
					"OB_PROP_COLOR_CONTRAST_INT":   map[string]interface{}{"current": originalContrast},
				},
			})
			test.That(t, err, test.ShouldBeNil)
		})

		t.Run("DoCommand set_device_property unsupported property", func(t *testing.T) {
			resp, err := cam.DoCommand(timeoutCtx, map[string]interface{}{
				"set_device_property": map[string]interface{}{
					"NonExistentProperty": 42,
				},
			})
			test.That(t, err, test.ShouldBeNil)
			errMsg, hasError := resp["error"]
			test.That(t, hasError, test.ShouldBeTrue)
			test.That(t, errMsg, test.ShouldContainSubstring, "not supported")
		})
	})
}
