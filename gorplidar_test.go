package gorplidar

import "testing"

func TestRplidarInit(t *testing.T) {
	lidar := NewRPLidar("/dev/ttyUSB0", 11520)
	lidar.Connect()
	lidar.StartMotor()
	lidar.StopMotor()
	lidar.Disconnect()
}
