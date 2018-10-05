# Go RPLidar

[![GoDoc](https://godoc.org/github.com/dolphindalt/gorplidar?status.svg)](https://godoc.org/gituhb.com/dolphindalt/gorplidar)

Package gorplidar provides a go library for communicating with the SLAMTEC RPLIDAR Low Cost 360 Degree Laser Range Scanner.

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

https://godoc.org/github.com/dolphindalt/gorplidar

## License

    MIT License

    Copyright (c) 2018 Dalton Caron

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

Files from the go-serial library are licensed under the Apache License, Version 2.

    Copyright 2014 Michael Phan-Ba

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

Files from the libserialport library are licensed under the GNU Lesset General Public License. 

    Copyright (C) 2010-2012 Bert Vermeulen <bert@biot.com>
    Copyright (C) 2010-2012 Uwe Hermann <uwe@hermann-uwe.de>
    Copyright (C) 2013-2014 Martin Ling <martin-libserialport@earth.li>
    Copyright (C) 2013 Matthias Heidbrink <m-sigrok@heidbrink.biz>
    Copyright (C) 2014 Aurelien Jacobs <aurel@gnuage.org>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.