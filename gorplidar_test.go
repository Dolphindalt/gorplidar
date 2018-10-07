package gorplidar

import (
	"testing"
)

func TestRplidarInit(t *testing.T) {
	lidar := NewRPLidar("/dev/ttyUSB0", 115200)
	lidar.Connect()
	lidar.StartMotor()
	lidar.StopMotor()
	lidar.Disconnect()
}
