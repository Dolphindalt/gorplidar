package gorplidar

import (
	"testing"
)

func TestRplidarInit(t *testing.T) {
	lidar := NewRPLidar("/dev/ttyUSB0", 115200)
	err := lidar.Connect()
	if err != nil {
		t.Fatal(err)
	}
	status, errcode, err := lidar.Health()
	if status != "Good" {
		t.Fatalf("%v: %v\n", status, errcode)
	}
	if err != nil {
		t.Fatal(err)
	}
	lidar.StartMotor()
	lidar.StopMotor()
	lidar.Disconnect()
}
