package golidar

/**
package main

import (
	"time"

	"github.com/golidar"
)

func main() {
	lidar := golidar.NewRPLidar("/dev/ttyUSB0", 115200, time.Millisecond*500)
	lidar.Connect()
	lidar.StartMotor()
	time.Sleep(time.Second * 10)
	lidar.StopMotor()
	time.Sleep(time.Second * 10)
	lidar.Disconnect()
}
*/
import (
	"encoding/binary"
	"log"
	"time"

	"github.com/mikepb/go-serial"
)

const (
	syncByte         byte   = 0xA5
	syncByte2        byte   = 0x5A
	getInfoByte      byte   = 0x50
	getHealthByte    byte   = 0x52
	stopByte         byte   = 0x25
	resetrByte       byte   = 0x40
	scanByte         byte   = 0x20
	forceScanByte    byte   = 0x21
	descriptorLength        = 7
	infoLength              = 20
	healthLength            = 3
	infoType                = 4
	healthType              = 6
	scanType                = 129
	maxMotorPWM      uint16 = 1023
	defaultMotorPWM  uint16 = 600
	setPWMByte       byte   = 0xF0
)

var healthStatus = map[int]string{
	0: "Good",
	1: "Warning",
	2: "Error",
}

// RPLidar holds information used to communicate through serial to the lidar
type RPLidar struct {
	serialPort  *serial.Port
	portName    string
	baudrate    int
	timeout     time.Duration
	motorActive bool
}

// NewRPLidar creates an instance of RPLidar
func NewRPLidar(portName string, baudrate int, timeout time.Duration) *RPLidar {
	return &RPLidar{nil, portName, baudrate, timeout, false}
}

// Connect establishes a serial communication channel with the lidar using the information provided form RPLidar
func (rpl *RPLidar) Connect() {
	if rpl.serialPort != nil {
		err := rpl.serialPort.Close()
		if err != nil {
			log.Fatal(err)
		}
	}
	options := serial.RawOptions
	options.Mode = serial.MODE_READ_WRITE
	options.BitRate = rpl.baudrate
	serialPort, err := options.Open(rpl.portName)
	if err != nil {
		log.Fatal(err)
	}
	rpl.serialPort = serialPort
}

// Disconnect closes serial communication
func (rpl *RPLidar) Disconnect() {
	if rpl.serialPort == nil {
		return
	}
	err := rpl.serialPort.Close()
	if err != nil {
		log.Fatal(err)
	}
}

// PWM stands for Pulse Width Modulation
func (rpl *RPLidar) PWM(pwm uint16) {
	if !(0 <= pwm && pwm <= maxMotorPWM) {
		log.Fatal("specified PWM was in an invalid range")
	}
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:2], pwm)
	rpl.sendPayloadCmd(setPWMByte, payload)
}

// StartMotor toggles the motor into an active spinning state
func (rpl *RPLidar) StartMotor() {
	log.Printf("Starting motor...\n")
	dtr, err := rpl.serialPort.DTR()
	if err != nil {
		log.Fatal(err)
	}
	err = rpl.serialPort.SetDTR(dtr)
	if err != nil {
		log.Fatal(err)
	}
	rpl.PWM(defaultMotorPWM)
	rpl.motorActive = true
}

// StopMotor brings the motor to zero velocity
func (rpl *RPLidar) StopMotor() {
	log.Printf("Stopping motor...\n")
	rpl.PWM(0)
	time.Sleep(time.Millisecond * 500)
	err := rpl.serialPort.SetDTR(0)
	if err != nil {
		log.Fatal(err)
	}
	rpl.motorActive = false
}

func (rpl *RPLidar) sendPayloadCmd(cmd byte, payload []byte) {
	req := []byte{}
	size := byte(len(payload))
	req = append(req, syncByte, cmd, size)
	for _, b := range payload {
		req = append(req, b)
	}
	checksum := byte(0)
	for _, b := range req {
		checksum ^= b
	}
	req = append(req, checksum)
	count, err := rpl.serialPort.Write(req)
	if err != nil {
		log.Fatal(err)
	}
	if count != len(req) {
		log.Fatal("Failed to write all bytes")
	}
	log.Printf("Sent command: %v\n", req)
}

func (rpl *RPLidar) sendCmd(cmd byte) {
	req := []byte{}
	req = append(req, syncByte, cmd)
	rpl.serialPort.Write(req)
	log.Printf("Sent command: %v\n", req)
}
