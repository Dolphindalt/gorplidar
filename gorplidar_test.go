package gorplidar

import (
	"testing"

	"github.com/mikepb/go-serial"
)

func TestRplidarInit(t *testing.T) {
	info, err := serial.ListPorts()
	if err != nil || len(info) <= 0 {
		t.Fatal("no serial ports detected")
	}
	port := info[0].Name()
	lidar := NewRPLidar(port, 11520)
	lidar.Connect()
	lidar.StartMotor()
	lidar.StopMotor()
	lidar.Disconnect()
}
