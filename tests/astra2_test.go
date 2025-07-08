package tests

import (
	"context"
	"fmt"
	"testing"
	"time"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"
	"go.viam.com/utils/rpc"
)

const (
	componentName       = "astra2-cam"
	testTimeoutDuration = 5 * time.Second
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

		t.Run("Get image method (one image)", func(t *testing.T) {
			timeout := time.After(testTimeoutDuration)
			tick := time.Tick(testTickDuration)
			for {
				select {
				case <-timeout:
					t.Fatal("timed out waiting for Get image method (one image)")
				case <-tick:
					img, err := camera.DecodeImageFromCamera(timeoutCtx, utils.MimeTypeJPEG, nil, cam)
					if err != nil {
						continue
					}
					test.That(t, img, test.ShouldNotBeNil)
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
					images, metadata, err := cam.Images(timeoutCtx)
					if err != nil || len(images) < 2 {
						continue
					}
					test.That(t, len(images), test.ShouldEqual, 2)
					test.That(t, metadata.CapturedAt, test.ShouldHappenAfter, timeBeforeCall)
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
					err := cam.Reconfigure(timeoutCtx, resource.Dependencies{}, cfg)
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
					pc, err := cam.NextPointCloud(timeoutCtx)
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
	})
}
