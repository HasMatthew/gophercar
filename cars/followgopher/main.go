package main

import (
	"fmt"
	"image"
	"image/color"
	"io"
	"math"
	"os"
	"os/exec"
	"strconv"
	"sync/atomic"
	"time"
	"gobot.io/x/gobot/drivers/i2c"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gobot.io/x/gobot/platforms/joystick"
	"gocv.io/x/gocv"
	"gobot.io/x/gobot/platforms/raspi"
	"github.com/fogleman/gg"
)

type pair struct {
	x float64
	y float64
}

const (
	frameX    = 400
	frameY    = 300
	frameSize = frameX * frameY * 3
	offset    = 32767.0
)



var (

	pca9685 *i2c.PCA9685Driver
	oled    *i2c.SSD1306Driver
	mpu6050 *i2c.MPU6050Driver

	// ffmpeg command to decode video stream from drone
	ffmpeg = exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
	ffmpegIn, _  = ffmpeg.StdinPipe()
	ffmpegOut, _ = ffmpeg.StdoutPipe()

	// gocv
	window = gocv.NewWindow("Tello")
	net    *gocv.Net
	green  = color.RGBA{0, 255, 0, 0}

	// tracking
	tracking                 = false
	detected                 = false
	detectSize               = false
	distTolerance            = 0.05 * dist(0, 0, frameX, frameY)
	refDistance              float64
	left, top, right, bottom float64

	// drone
	drone      = tello.NewDriver("8890")
	flightData *tello.FlightData

	// joystick
	joyAdaptor                   = joystick.NewAdaptor()
	stick                        = joystick.NewDriver(joyAdaptor, "dualshock4")
	leftX, leftY, rightX, rightY atomic.Value
)

func init() {
	leftX.Store(float64(0.0))
	leftY.Store(float64(0.0))
	rightX.Store(float64(0.0))
	rightY.Store(float64(0.0))

	// process drone events in separate goroutine for concurrency
	go func() {
		// process joystick events
		handleJoystick()

		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		drone.On(tello.FlightDataEvent, func(data interface{}) {
			// TODO: protect flight data from race condition
			flightData = data.(*tello.FlightData)
		})

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
			drone.StartVideo()
			drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
			drone.SetExposure(0)
			gobot.Every(100*time.Millisecond, func() {
				drone.StartVideo()
			})
		})

		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})

		robot := gobot.NewRobot("tello",
			[]gobot.Connection{joyAdaptor},
			[]gobot.Device{drone, stick},
		)

		robot.Start()
	}()
}

func main() {
	if len(os.Args) < 5 {
		fmt.Println("How to run:\ngo run facetracker.go [model] [config] ([backend] [device])")
		return
	}

	model := os.Args[1]
	config := os.Args[2]
	backend := gocv.NetBackendDefault
	if len(os.Args) > 3 {
		backend = gocv.ParseNetBackend(os.Args[3])
	}

	target := gocv.NetTargetCPU
	if len(os.Args) > 4 {
		target = gocv.ParseNetTarget(os.Args[4])
	}

	n := gocv.ReadNet(model, config)
	if n.Empty() {
		fmt.Printf("Error reading network model from : %v %v\n", model, config)
		return
	}
	net = &n
	defer net.Close()
	net.SetPreferableBackend(gocv.NetBackendType(backend))
	net.SetPreferableTarget(gocv.NetTargetType(target))

	for {
		// get next frame from stream
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}
		img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		if img.Empty() {
			continue
		}

		trackFace(&img)

		window.IMShow(img)
		if window.WaitKey(10) >= 0 {
			break
		}
	}
}

func trackFace(frame *gocv.Mat) {
	W := float64(frame.Cols())
	H := float64(frame.Rows())

	blob := gocv.BlobFromImage(*frame, 1.0, image.Pt(300, 300), gocv.NewScalar(104, 177, 123, 0), false, false)
	defer blob.Close()

	net.SetInput(blob, "data")

	detBlob := net.Forward("detection_out")
	defer detBlob.Close()

	detections := gocv.GetBlobChannel(detBlob, 0, 0)
	defer detections.Close()


	for r := 0; r < detections.Rows(); r++ {

		confidence := detections.GetFloatAt(r, 2)
		if confidence < 0.5 {
			continue
		}

		left = float64(detections.GetFloatAt(r, 3)) * W
		top = float64(detections.GetFloatAt(r, 4)) * H
		right = float64(detections.GetFloatAt(r, 5)) * W
		bottom = float64(detections.GetFloatAt(r, 6)) * H

		left = math.Min(math.Max(0.0, left), W-1.0)
		right = math.Min(math.Max(0.0, right), W-1.0)
		bottom = math.Min(math.Max(0.0, bottom), H-1.0)
		top = math.Min(math.Max(0.0, top), H-1.0)

		detected = true
		rect := image.Rect(int(left), int(top), int(right), int(bottom))
		gocv.Rectangle(frame, rect, green, 3)

	}

	if !tracking {
		return
	}

	if !detected {
		setThrottle(0)
	} else {
		setThrottle(-0.25)
	}

	if !detected {
		return
	}

	// Turn the car steering left or right based on whether the face image is to the left or right of center
	switch {
	case right < W/2:
		setSteering(1.0)  // TODO: use an offset based on the current steering
	case left > W/2:
		setSteering(-1.0)  // TODO: use an offset based on the current steering
	default:
		setSteering(0.0)
	}


}

func dist(x1, y1, x2, y2 float64) float64 {
	return math.Sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
}

func handleJoystick() {
	stick.On(joystick.CirclePress, func(data interface{}) {
		drone.Forward(0)
		drone.Up(0)
		drone.Clockwise(0)
		tracking = !tracking
		if tracking {
			detectSize = true
			println("tracking")
		} else {
			detectSize = false
			println("not tracking")
		}
	})
	stick.On(joystick.SquarePress, func(data interface{}) {
		fmt.Println("battery:", flightData.BatteryPercentage)
	})
	stick.On(joystick.TrianglePress, func(data interface{}) {
		drone.ThrowTakeOff()
		println("Takeoff")
	})
	stick.On(joystick.XPress, func(data interface{}) {
		drone.PalmLand()
		println("Land")
	})
	stick.On(joystick.LeftX, func(data interface{}) {
		val := float64(data.(int16))
		leftX.Store(val)
	})

	stick.On(joystick.LeftY, func(data interface{}) {
		val := float64(data.(int16))
		leftY.Store(val)
	})

	stick.On(joystick.RightX, func(data interface{}) {
		val := float64(data.(int16))
		rightX.Store(val)
	})

	stick.On(joystick.RightY, func(data interface{}) {
		val := float64(data.(int16))
		rightY.Store(val)
	})
	gobot.Every(50*time.Millisecond, func() {
		rightStick := getRightStick()

		switch {
		case rightStick.y < -10:
			drone.Forward(tello.ValidatePitch(rightStick.y, offset))
		case rightStick.y > 10:
			drone.Backward(tello.ValidatePitch(rightStick.y, offset))
		default:
			drone.Forward(0)
		}

		switch {
		case rightStick.x > 10:
			drone.Right(tello.ValidatePitch(rightStick.x, offset))
		case rightStick.x < -10:
			drone.Left(tello.ValidatePitch(rightStick.x, offset))
		default:
			drone.Right(0)
		}
	})

	gobot.Every(50*time.Millisecond, func() {
		leftStick := getLeftStick()
		switch {
		case leftStick.y < -10:
			drone.Up(tello.ValidatePitch(leftStick.y, offset))
		case leftStick.y > 10:
			drone.Down(tello.ValidatePitch(leftStick.y, offset))
		default:
			drone.Up(0)
		}

		switch {
		case leftStick.x > 20:
			drone.Clockwise(tello.ValidatePitch(leftStick.x, offset))
		case leftStick.x < -20:
			drone.CounterClockwise(tello.ValidatePitch(leftStick.x, offset))
		default:
			drone.Clockwise(0)
		}
	})
}

func getLeftStick() pair {
	s := pair{x: 0, y: 0}
	s.x = leftX.Load().(float64)
	s.y = leftY.Load().(float64)
	return s
}

func getRightStick() pair {
	s := pair{x: 0, y: 0}
	s.x = rightX.Load().(float64)
	s.y = rightY.Load().(float64)
	return s
}

func setSteering(steering float64) {
	steeringVal := getSteeringPulse(steering)
	pca9685.SetPWM(1, 0, uint16(steeringVal))
	fmt.Printf("setSteering to %v", steering)
}

func setThrottle(throttle float64) {
	throttleVal := getThrottlePulse(throttle)
	pca9685.SetPWM(0, 0, uint16(throttleVal))
	fmt.Printf("setThrottle to %v", throttle)
}

// adjusts the steering from -1.0 (hard left) <-> 1.0 (hardright) to the correct
// pwm pulse values.
func getSteeringPulse(val float64) float64 {
	return gobot.Rescale(val, -1, 1, 290, 490)
}

// adjusts the throttle from -1.0 (hard back) <-> 1.0 (hard forward) to the correct
// pwm pulse values.
func getThrottlePulse(val float64) int {
	if val > 0 {
		return int(gobot.Rescale(val, 0, 1, 350, 300))
	}
	return int(gobot.Rescale(val, -1, 0, 490, 350))
}

