# Go RPLidar

[![GoDoc](https://godoc.org/github.com/Dolphindalt/gorplidar?status.svg)](https://godoc.org/github.com/Dolphindalt/gorplidar)
[![TravisCI](https://travis-ci.org/Dolphindalt/gorplidar.svg?branch=master)](https://travis-ci.org/Dolphindalt/gorplidar)

Package gorplidar provides a library to control the Slamtec RPLidar.

Protocol: https://www.robotshop.com/media/files/pdf2/rpk-02-communication-protocol.pdf

This package aims to satisfy the communication protocol specified in the document linked above.

Currently, not all protocols are supported. Configurability could also be improved,
as many options are set by default on instantiation of the RPLidar structure.

## Usage

```go
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
```

## Documentation

https://godoc.org/github.com/Dolphindalt/gorplidar
