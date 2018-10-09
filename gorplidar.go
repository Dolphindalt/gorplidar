/*

Package gorplidar provides a library to control the Slamtec RPLidar.

Protocol: https://www.robotshop.com/media/files/pdf2/rpk-02-communication-protocol.pdf

This package aims to satisfy the communication protocol specified in the document linked above.

Currently, not all protocols are supported. Configurability could also be improved,
as many options are set by default on instantiation of the RPLidar structure.

Example usage:

	package main

	import (
		"fmt"
		"log"

		"github.com/gorplidar"
	)

	func main() {
		lidar := gorplidar.NewRPLidar("/dev/ttyUSB0", 115200)
		lidar.Connect()
		status, errcode, err := lidar.Health()
		if err != nil {
			log.Fatal(err)
		} else if status == "Warning" {
			log.Printf("Lidar status: %v Error Code: %v\n", status, errcode)
		} else if status == "Error" {
			log.Fatalf("Lidar status: %v Error Code: %v\n", status, errcode)
		}
		lidar.StartMotor()
		scanResults, err := lidar.StartScan(3)
		if err != nil {
			log.Fatal(err)
		}
		for _, p := range scanResults {
			fmt.Printf("Quality: %v\tAngle: %.2f\tDistance: %.2f\n", p.Quality, p.Angle, p.Distance)
		}
		lidar.StopMotor()
		lidar.Disconnect()
	}

*/
package gorplidar

import (
	"encoding/hex"
	"errors"
	"fmt"
	"log"
	"math"
	"time"

	"github.com/mikepb/go-serial"
)

const (
	syncByte         byte   = 0xA5
	syncByte2        byte   = 0x5A
	getInfoByte      byte   = 0x50
	getHealthByte    byte   = 0x52
	stopByte         byte   = 0x25
	resetByte        byte   = 0x40
	scanByte         byte   = 0x20
	forceScanByte    byte   = 0x21
	sampleRateByte   byte   = 0x59
	expressScanByte  byte   = 0x82
	descriptorLength int    = 7
	infoLength       int    = 20
	healthLength     int    = 3
	infoType         int    = 4
	healthType       int    = 6
	scanType         int    = 129
	maxMotorPWM      uint16 = 1023
	defaultMotorPWM  uint16 = 600
	setPWMByte       byte   = 0xF0
	maxBufferSize    int    = 500
)

// healthStatus used in GetHealth function string return
var healthStatus = map[int]string{
	0: "Good",
	1: "Warning",
	2: "Error",
}

// RPLidar holds information used to communicate through serial to the lidar.
type RPLidar struct {
	serialPort  *serial.Port
	portName    string
	baudrate    int
	MotorActive bool
	options     *serial.Options
	Connected   bool
	Scanning    bool
}

// RPLidarInfo is returned by GetDeviceInfo.
type RPLidarInfo struct {
	Model        int
	Firmware     [2]int
	Hardware     int
	SerialNumber string
}

// RPLidarPoint represents a single point of data from a lidar scan.
type RPLidarPoint struct {
	Quality  int
	Angle    float32
	Distance float32
}

type expressData struct {
	newScan    bool
	startAngle float32
	distances  []float32
	angles     []float32
}

// NewRPLidar creates an instance of RPLidar.
// The portName refers to the name of the serial port.
// Baudrate is the rate at which data is transfered through the serial.
func NewRPLidar(portName string, baudrate int) *RPLidar {
	return &RPLidar{nil, portName, baudrate, false, nil, false, false}
}

// Connect establishes a serial communication channel with the lidar using the information provided form RPLidar.
func (rpl *RPLidar) Connect() error {
	if rpl.serialPort != nil && rpl.Connected {
		err := rpl.serialPort.Close()
		if err != nil {
			return err
		}
	}
	options := serial.RawOptions
	options.Mode = serial.MODE_READ_WRITE
	options.BitRate = rpl.baudrate

	serialPort, err := options.Open(rpl.portName)
	if err != nil {
		return err
	}
	rpl.options = &options
	rpl.serialPort = serialPort
	rpl.Connected = true
	return nil
}

// Disconnect closes serial communication.
func (rpl *RPLidar) Disconnect() error {
	if rpl.serialPort == nil || !rpl.Connected {
		return nil
	}
	rpl.Reset()
	rpl.serialPort.Reset()
	err := rpl.serialPort.Close()
	if err != nil {
		return err
	}
	rpl.Connected = false
	return nil
}

// PWM stands for Pulse Width Modulation.
// pwm can be zero or less than or equal to 1023.
func (rpl *RPLidar) PWM(pwm uint16) error {
	if !(0 <= pwm && pwm <= maxMotorPWM) {
		return errors.New("specified PWM was in an invalid range")
	}
	payload := make([]byte, 2)
	payload[0] = byte(pwm & 0xFF)
	payload[1] = byte(pwm >> 8)
	rpl.sendPayloadCmd(setPWMByte, payload)
	return nil
}

// StartMotor toggles the motor into an active spinning state.
func (rpl *RPLidar) StartMotor() error {
	if rpl.serialPort == nil {
		return errors.New("serial port not detected")
	}
	rpl.options.DTR = serial.DTR_OFF // start A1 motor
	rpl.serialPort.Apply(rpl.options)
	rpl.PWM(defaultMotorPWM) // start A2 motor
	rpl.MotorActive = true
	return nil
}

// StopMotor brings the motor to zero velocity.
func (rpl *RPLidar) StopMotor() error {
	if rpl.serialPort == nil {
		return errors.New("serial port not detected")
	}
	rpl.PWM(0) // stop A2 motor
	time.Sleep(time.Millisecond * 2)
	rpl.options.DTR = serial.DTR_ON // stop A1 motor
	rpl.serialPort.Apply(rpl.options)
	rpl.MotorActive = false
	return nil
}

// StopScan forces the lidar to exit the current scan.
func (rpl *RPLidar) StopScan() {
	rpl.sendCmd(stopByte)
	rpl.Scanning = false
	time.Sleep(time.Millisecond * 20)
}

// Reset forces the lidar to reset into a state similar to after powering up.
func (rpl *RPLidar) Reset() {
	rpl.sendCmd(resetByte)
	time.Sleep(time.Millisecond * 20)
}

// StartScan begins a lidar scan for the amount of scan cycles desired.
// The scans parameter refers to how many scan cycles will occur.
func (rpl *RPLidar) StartScan(scanCycles int) ([]*RPLidarPoint, error) {
	if !rpl.Connected {
		return nil, errors.New("The device is not connected")
	}
	if !rpl.MotorActive {
		rpl.StartMotor()
	}
	rpl.StopScan()
	totalScanCycles := 0
	scan, asize, err := rpl.startScanCmd(scanByte)
	if err != nil {
		status, errcode, err := rpl.Health()
		if err != nil {
			log.Fatalf("Health check failed after scan failed: %v\n", err)
		} else if status == "Warning" {
			log.Printf("Warning status on health check after failed scan: %v\n", errcode)
		} else if status == "Error" {
			log.Fatalf("Error status on health check after failed scan: %v\n", errcode)
		}
	}
	rpl.readResponse(asize) // disregard first measurment as the results may be incomplete
	for {
		data := rpl.readResponse(asize)
		newScan, quality, angle, distance := rpl.parseRawScanData(data)
		if newScan && rpl.Scanning {
			if totalScanCycles == scanCycles {
				break
			}
			totalScanCycles++
		}
		rpl.Scanning = true
		if quality > 0 && distance > 0 {
			scan = append(scan, &RPLidarPoint{quality, angle, distance})
		}
	}
	rpl.StopScan()
	v, err := rpl.serialPort.InputWaiting()
	if err != nil {
		log.Fatal(err)
	}
	v /= asize
	for i := 0; i < v; i++ {
		data := rpl.readResponse(asize)
		_, quality, angle, distance := rpl.parseRawScanData(data)
		if quality > 0 && distance > 0 {
			scan = append(scan, &RPLidarPoint{quality, angle, distance})
		}
	}

	return scan, nil
}

// ExpressScan performs a scan as fast as possible (A2 4khz).
// The A1 device should just use the StartScan function because of the same sampling rate (2khz).
func (rpl *RPLidar) ExpressScan(scanCycles int) ([]*RPLidarPoint, error) {
	if !rpl.Connected {
		return nil, errors.New("The device is not connected")
	}
	if !rpl.MotorActive {
		rpl.StartMotor()
	}
	rpl.StopScan()
	requestPacket := []byte{0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22}
	rpl.sendPayloadCmd(expressScanByte, requestPacket)
	asize, single, _, err := rpl.readDescriptor()
	if err != nil {
		return nil, err
	}
	if asize != 84 {
		return nil, errors.New("Express scan size not 84 bytes")
	}
	if !single {
		return nil, errors.New("Expected mutliple response mode for express scan")
	}
	frame := 32
	var oldExpressData, currentExpressData *expressData
	currentExpressData = nil
	oldExpressData = nil
	totalCycles := 0
	scan := []*RPLidarPoint{}
	rpl.Scanning = true
	for {
		if frame == 32 {
			if scanCycles == totalCycles {
				break
			}
			totalCycles++
			frame = 0
			if currentExpressData == nil {
				raw := rpl.readResponse(asize)
				currentExpressData = rpl.parseRawExpressScanData(raw)
			}
			oldExpressData = currentExpressData
			raw := rpl.readResponse(asize)
			currentExpressData = rpl.parseRawExpressScanData(raw)
		}
		frame++
		scan = append(scan, rpl.transformExpressScanData(oldExpressData, currentExpressData.startAngle, frame))
	}
	rpl.StopScan()
	v, err := rpl.serialPort.InputWaiting()
	if err != nil {
		log.Fatal(err)
	}
	v /= asize
	for i := 0; i < v; i++ {
		data := rpl.readResponse(asize)
		_, quality, angle, distance := rpl.parseRawScanData(data)
		if quality > 0 && distance > 0 {
			scan = append(scan, &RPLidarPoint{quality, angle, distance})
		}
	}
	return scan, nil
}

// SampleRate returns a single measurement duration for standard and express scanning modes.
// This function seems to only be supported by the A2 model.
// The time is measured in micro seconds.
// The first int returned is the standard, the second is the express.
func (rpl *RPLidar) SampleRate() (int, int, error) {
	rpl.sendCmd(sampleRateByte)
	asize, single, _, err := rpl.readDescriptor()
	if err != nil {
		return 0, 0, err
	}
	if asize != 4 {
		return 0, 0, fmt.Errorf("Sample rate length %v, expected 4", asize)
	}
	if !single {
		return 0, 0, errors.New("Expected single response mode for sample rate")
	}
	data := rpl.readResponse(asize)
	startScanSampleRate := int(data[0]) + int(data[1])
	expressScanSampleRate := int(data[2]) + int(data[3])
	return startScanSampleRate, expressScanSampleRate, nil
}

// DeviceInfo returns a struct containing information about the lidar.
func (rpl *RPLidar) DeviceInfo() (*RPLidarInfo, error) {
	rpl.sendCmd(getInfoByte)
	asize, single, dtype, err := rpl.readDescriptor()
	if err != nil {
		return nil, err
	}
	if asize != infoLength {
		return nil, errors.New("Unexpected size of get health response")
	}
	if !single {
		return nil, errors.New("Expected single reponse mode")
	}
	if dtype != infoType {
		return nil, errors.New("Expected response of get health type")
	}
	data := rpl.readResponse(asize)
	serialHex := fmt.Sprintf("%x", data[4:])
	serialNumber, err := hex.DecodeString(serialHex)
	if err != nil {
		return nil, err
	}
	return &RPLidarInfo{
		int(data[0]),
		[2]int{int(data[2]), int(data[1])},
		int(data[3]),
		string(serialNumber),
	}, nil
}

// Health returns a string representing the status and an error code representing the health of the lidar.
func (rpl *RPLidar) Health() (string, int, error) {
	rpl.sendCmd(getHealthByte)
	asize, single, dtype, err := rpl.readDescriptor()
	if err != nil {
		return "", 0, err
	}
	if asize != healthLength {
		return "", 0, errors.New("Unexpected size of get health response")
	}
	if !single {
		return "", 0, errors.New("Expected single reponse mode")
	}
	if dtype != healthType {
		return "", 0, errors.New("Expected response of get health type")
	}
	data := rpl.readResponse(asize)
	status := healthStatus[int(data[0])]
	errcode := int(data[1])<<8 + int(data[2])
	return status, errcode, nil
}

// Processes the command to start a scan
func (rpl *RPLidar) startScanCmd(cmd byte) ([]*RPLidarPoint, int, error) {
	scan := []*RPLidarPoint{}
	rpl.serialPort.ResetInput()
	time.Sleep(time.Millisecond * 100) // this works, trust me
	rpl.sendCmd(cmd)
	asize, single, dtype, err := rpl.readDescriptor()
	if err != nil {
		return nil, 0, err
	}
	if asize != 5 {
		return nil, 0, fmt.Errorf("Scan length %v, expected 5", asize)
	}
	if single {
		return nil, 0, errors.New("Expected multiple response mode for start scan")
	}
	if dtype != scanType {
		return nil, 0, errors.New("Expected response of start scan type")
	}
	return scan, asize, nil
}

// parseRawScanData converts data from a scan into a slice of *RPLidarPoints.
// Format of the Data Response Packets:
// 0 : quality 6 bits, |S| 1 bit, S 1 bit
// 1 : angle 7 bits, C 1 bit
// 2 : angle 8 bits
// 3 : distance 8 bits
// 4 : distance 8 bits
func (rpl *RPLidar) parseRawScanData(data []byte) (bool, int, float32, float32) {
	s := data[0]&0x01 > 0
	abS := (data[0]>>1)&0x01 > 0
	quality := int(data[0]) >> 2
	if s == abS {
		log.Fatal("Data check bit failed while parsing raw scan data")
	}
	c := data[1] & 0x01
	if c != 1 {
		log.Fatal("Check bit was not 1")
	}
	angle := float32(int(data[1])>>1+int(data[2])<<7) / 64.0
	distance := float32(int(data[3])+(int(data[4])<<8)) / 4.0
	return s, quality, angle, distance
}

// transformExpressScanData takes a reference angle, frame, data, and transforms it according to the slamtec specification.
func (rpl *RPLidar) transformExpressScanData(od *expressData, newAngle float32, frame int) *RPLidarPoint {
	angle := math.Mod(float64(float64(od.startAngle)+(math.Mod(float64(newAngle-od.startAngle), 360))/32.0*float64(frame)-float64(od.angles[frame-1])), 360)
	distance := od.distances[frame-1]
	return &RPLidarPoint{100, float32(angle), distance}
}

// Abandon hope, all who enter here
// Or refer to the slamtec rplidar documentation
func (rpl *RPLidar) parseRawExpressScanData(data []byte) *expressData {
	sync1 := (data[0] >> 4)
	sync2 := (data[1] >> 4)
	sign := []float32{1, -1}
	if sync1 != 0xA || sync2 != 0x5 {
		log.Fatal("Sync bit check failed while parsing raw scan data")
	}
	newScan := (data[3] >> 7) > 1
	startAngle := float32(int(data[2])+int((data[3]&0xFF))<<8) / 64.0
	distances := []float32{}
	angles := []float32{}
	for i := 0; i < 80; i += 5 {
		d1 := float32((int(data[i+4]) >> 2) + (int(data[i+5]) << 6))
		a1 := float32(int(data[i+8]&0x0F)+(int(data[i+4]&0x01)<<4)) / 8.0 * sign[int(data[i+6]&0x02)>>1]
		d2 := float32((int(data[i+6]) >> 2) + int(data[i+7]<<6))
		a2 := float32((int(data[i+8])>>4)+(int(data[i+6]&0x01)<<4)) / 8.0 * sign[int(data[i+6]&0x02)>>1]
		distances = append(distances, d1, d2)
		angles = append(angles, a1, a2)
	}
	return &expressData{newScan, startAngle, distances, angles}
}

// sendPayloadCmd sends a command bearing extra data to the lidar.
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
}

// sendCmd sends a one byte command to the lidar.
func (rpl *RPLidar) sendCmd(cmd byte) {
	req := []byte{}
	req = append(req, syncByte, cmd)
	rpl.serialPort.Write(req)
}

// readResonse returns a byte slice of a specified size of bytes recieved from the lidar.
func (rpl *RPLidar) readResponse(size int) []byte {
	res := make([]byte, size)
	asize, err := rpl.serialPort.Read(res)
	if err != nil {
		log.Fatal(err)
	}
	if asize != size {
		log.Fatalf("Expected to read %v bytes, read %v instead\n", size, asize)
	}
	return res
}

// readDescriptor returns information about data ready to be recieved by the client.
// The first int is the size of the response.
// The bool is true if the lidar is in single response mode, else multi response mode.
// The last int is the type of response.
func (rpl *RPLidar) readDescriptor() (int, bool, int, error) {
	descriptor := make([]byte, descriptorLength)
	asize, err := rpl.serialPort.Read(descriptor)
	if err != nil {
		return 0, false, 0, err
	}
	if asize != descriptorLength {
		return 0, false, 0, fmt.Errorf("expected to read %v bytes, read %v instead", descriptorLength, asize)
	}
	if descriptor[0] != syncByte || descriptor[1] != syncByte2 {
		return 0, false, 0, errors.New("expected descriptor starting bytes")
	}
	single := (descriptor[descriptorLength-2] == 0)
	return int(descriptor[2]), single, int(descriptor[descriptorLength-1]), nil
}
