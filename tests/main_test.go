package tests

import (
	"bytes"
	"context"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"go.viam.com/rdk/config"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot"
	robotimpl "go.viam.com/rdk/robot/impl"
)

var (
	modulePath    = flag.String("module", "", "the path to the Orbbec local module binary to test.")
	serialNumber  = flag.String("serial_number", "", "the serial number of the Orbbec device to test.")
	absModulePath = ""
)

func TestMain(m *testing.M) {
	logger := logging.NewLogger("orbbec-tests")
	fmt.Println("Starting Orbbec module tests")
	flag.Parse()

	moduleString := strings.TrimSpace(*modulePath)
	serialNumber := strings.TrimSpace(*serialNumber)

	if moduleString == "" {
		logger.Fatal("The path to the module is a required argument.")
	}

	if serialNumber == "" {
		logger.Fatal("The serial number of the Orbbec device is a required argument.")
	}

	absPath, err := filepath.Abs(moduleString)
	if err != nil {
		logger.Fatal("Error resolving absolute path for module: %v", err)
	}
	absModulePath = absPath

	logger.Info("Checking if file exists at %q", absModulePath)
	_, err = os.Stat(absModulePath)
	if err != nil {
		logger.Fatal("Error checking if file exists at %q: %v", absModulePath, err)
	}
	logger.Info("File exists.")

	exitVal := m.Run()
	if exitVal == 0 {
		logger.Info("All tests passed.")
	} else {
		logger.Error("Tests failed.")
	}
	os.Exit(exitVal)
}

func setUpViamServer(ctx context.Context, configString string, loggerName string, _ *testing.T) (robot.Robot, error) {
	logger := logging.NewLogger(loggerName)

	cfg, err := config.FromReader(ctx, "default.json", bytes.NewReader([]byte(configString)), logger, nil)
	if err != nil {
		return nil, err
	}

	r, err := robotimpl.RobotFromConfig(ctx, cfg, nil, logger)
	if err != nil {
		return nil, err
	}

	return r, nil
}
