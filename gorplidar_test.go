package gorplidar

import (
	"errors"
	"testing"
	"time"

	serial "go.bug.st/serial"
)

// NewRPLidarWithPort creates a new instance of RPLidar (for testing).
func NewRPLidarWithPort(port serial.Port) *RPLidar {
	return &RPLidar{port, "test_port",
		115200,
		false,
		true,
		false,
	}
}

// MockSerialPort implements SerialPort for testing
type MockSerialPort struct {
	ReadBuffer      []byte
	ReadIndex       int
	WriteBuffer     []byte
	Closed          bool
	DTRState        bool
	RTSState        bool
	ReadTimeout     time.Duration
	Mode            *serial.Mode
	ModemStatusBits *serial.ModemStatusBits

	// Error injection
	ReadError               error
	WriteError              error
	SetModeError            error
	DrainError              error
	ResetInputBufferError   error
	ResetOutputBufferError  error
	SetDTRError             error
	SetRTSError             error
	GetModemStatusBitsError error
	SetReadTimeoutError     error
	BreakError              error
}

func NewMockSerialPort() *MockSerialPort {
	return &MockSerialPort{
		ReadBuffer:  []byte{},
		WriteBuffer: []byte{},
	}
}

func (m *MockSerialPort) SetMode(mode *serial.Mode) error {
	if m.SetModeError != nil {
		return m.SetModeError
	}
	m.Mode = mode
	return nil
}

func (m *MockSerialPort) Read(p []byte) (n int, err error) {
	if m.ReadError != nil {
		return 0, m.ReadError
	}

	if m.ReadIndex >= len(m.ReadBuffer) {
		return 0, errors.New("serial: timout")
	}

	n = copy(p, m.ReadBuffer[m.ReadIndex:])
	m.ReadIndex += n
	return n, nil
}

func (m *MockSerialPort) Write(p []byte) (n int, err error) {
	if m.WriteError != nil {
		return 0, m.WriteError
	}

	m.WriteBuffer = append(m.WriteBuffer, p...)
	return len(p), nil
}

func (m *MockSerialPort) Drain() error {
	if m.DrainError != nil {
		return m.DrainError
	}
	// This would wait for write buffer to flush
	return nil
}

func (m *MockSerialPort) ResetInputBuffer() error {
	return nil
}

func (m *MockSerialPort) ResetOutputBuffer() error {
	return nil
}

func (m *MockSerialPort) SetDTR(dtr bool) error {
	if m.SetDTRError != nil {
		return m.SetDTRError
	}
	m.DTRState = dtr
	return nil
}

func (m *MockSerialPort) SetRTS(rts bool) error {
	if m.SetRTSError != nil {
		return m.SetRTSError
	}
	m.RTSState = rts
	return nil
}

func (m *MockSerialPort) GetModemStatusBits() (*serial.ModemStatusBits, error) {
	if m.GetModemStatusBitsError != nil {
		return nil, m.GetModemStatusBitsError
	}
	return m.ModemStatusBits, nil
}

func (m *MockSerialPort) SetReadTimeout(t time.Duration) error {
	if m.SetReadTimeoutError != nil {
		return m.SetReadTimeoutError
	}
	m.ReadTimeout = t
	return nil
}

func (m *MockSerialPort) Close() error {
	m.Closed = true
	return nil
}

func (m *MockSerialPort) Break(d time.Duration) error {
	if m.BreakError != nil {
		return m.BreakError
	}
	// Simulate break signal for duration d
	return nil
}

// Helper method to queue response data for testing
func (m *MockSerialPort) QueueResponse(data []byte) {
	m.ReadBuffer = append(m.ReadBuffer, data...)
}

// Helper method to get what was written
func (m *MockSerialPort) GetWritten() []byte {
	return m.WriteBuffer
}

// Helper method to clear buffers
func (m *MockSerialPort) Reset() {
	m.ReadBuffer = []byte{}
	m.ReadIndex = 0
	m.WriteBuffer = []byte{}
}

func TestStartMotor(t *testing.T) {
	mock := NewMockSerialPort()
	rpl := NewRPLidarWithPort(mock)

	err := rpl.StartMotor()
	if err != nil {
		t.Fatalf("StartMotor failed: %v", err)
	}

	if !rpl.MotorActive {
		t.Error("Motor should be active")
	}

	if mock.DTRState != false {
		t.Error("DTR should be set to false")
	}
}

func TestStartMotor_NoSerialPort(t *testing.T) {
	rpl := &RPLidar{serialPort: nil}

	err := rpl.StartMotor()
	if err == nil {
		t.Error("Expected error when serial port is nil")
	}
}

func TestStartMotor_DTRError(t *testing.T) {
	mock := NewMockSerialPort()
	mock.SetDTRError = errors.New("DTR error")
	rpl := NewRPLidarWithPort(mock)

	err := rpl.StartMotor()
	if err == nil {
		t.Error("Expected error when SetDTR fails")
	}
}

func TestStartScan(t *testing.T) {
	mock := NewMockSerialPort()
	rpl := NewRPLidarWithPort(mock)

	// Descriptor
	descriptor := []byte{0xA5, 0x5A, 0x05, 0x00, 0x00, 0x01, 0x81}
	mock.QueueResponse(descriptor)

	// First measurement (discarded)
	// S=1, |S|=0, quality=1: (1 << 2) | (0 << 1) | 1 = 0x05
	mock.QueueResponse([]byte{0x05, 0x81, 0x00, 0x10, 0x00})

	// Generate scan points for 1 complete cycle
	for i := range 10 {
		var s byte
		if i == 0 {
			// S=1, |S|=0, quality=1: (1 << 2) | (0 << 1) | 1 = 0x05
			s = 0x05
		} else {
			// S=0, |S|=1, quality=1: (1 << 2) | (1 << 1) | 0 = 0x06
			s = 0x06
		}
		// C bit must be 1
		angleByte := byte(0x01 | (i << 1)) // C=1, angle low bits
		mock.QueueResponse([]byte{s, angleByte, 0x00, 0x20, 0x00})
	}

	// Add one more point with S=1 to trigger end of cycle
	mock.QueueResponse([]byte{0x05, 0x81, 0x00, 0x20, 0x00})

	results, err := rpl.StartScan(1)
	if err != nil {
		t.Fatalf("StartScan failed: %v", err)
	}

	if len(results) == 0 {
		t.Error("Expected scan results")
	}
}

func TestStartScan_NotConnected(t *testing.T) {
	mock := NewMockSerialPort()
	rpl := NewRPLidarWithPort(mock)
	rpl.Connected = false

	_, err := rpl.StartScan(1)
	if err == nil {
		t.Error("Expected error when not connected")
	}
}

func TestStartScan_DescriptorError(t *testing.T) {
	mock := NewMockSerialPort()
	mock.ReadError = errors.New("read error")
	rpl := NewRPLidarWithPort(mock)

	_, err := rpl.StartScan(1)
	if err == nil {
		t.Error("Expected error when descriptor read fails")
	}
}

func TestStartScan_WrongDescriptorSize(t *testing.T) {
	mock := NewMockSerialPort()
	rpl := NewRPLidarWithPort(mock)

	// Wrong size descriptor (size=10 instead of 5)
	descriptor := []byte{0xA5, 0x5A, 0x0A, 0x00, 0x00, 0x01, 0x81}
	mock.QueueResponse(descriptor)

	_, err := rpl.StartScan(1)
	if err == nil {
		t.Error("Expected error for wrong descriptor size")
	}
}
