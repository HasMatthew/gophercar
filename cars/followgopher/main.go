package main

import (
	"fmt"
	"image"
	"image/color"
	"log"
	"math"
	"net/http"
	"os"
	"strconv"
	"sync/atomic"

	"github.com/hybridgroup/mjpeg"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/i2c"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gobot.io/x/gobot/platforms/joystick"
	"gobot.io/x/gobot/platforms/raspi"
	"gocv.io/x/gocv"
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
	r       *raspi.Adaptor
	pca9685 *i2c.PCA9685Driver
	oled    *i2c.SSD1306Driver
	mpu6050 *i2c.MPU6050Driver

	// gocv
	green = color.RGBA{0, 255, 0, 0}

	// tracking
	tracking                 = false
	detected                 = false
	detectSize               = false
	distTolerance            = 0.05 * dist(0, 0, frameX, frameY)
	refDistance              float64
	left, top, right, bottom float64

	// drone
	flightData *tello.FlightData

	// joystick
	joyAdaptor                   = joystick.NewAdaptor()
	stick                        = joystick.NewDriver(joyAdaptor, "dualshock4")
	leftX, leftY, rightX, rightY atomic.Value

	classifier gocv.CascadeClassifier
)

func init() {
	leftX.Store(float64(0.0))
	leftY.Store(float64(0.0))
	rightX.Store(float64(0.0))
	rightY.Store(float64(0.0))

}

func main() {
	deviceID, _ := strconv.Atoi(os.Args[1])
	xmlFile := os.Args[2]

	r = raspi.NewAdaptor()
	pca9685 = i2c.NewPCA9685Driver(r)
	oled = i2c.NewSSD1306Driver(r)
	mpu6050 = i2c.NewMPU6050Driver(r)

	robot := gobot.NewRobot("gophercar",
		[]gobot.Connection{r},
		[]gobot.Device{pca9685, oled, mpu6050},
	)

	go robot.Start()

	// open webcam
	fmt.Printf("OpenVideoCapture()")
	webcam, err := gocv.OpenVideoCapture(deviceID)
	if err != nil {
		fmt.Printf("Error opening capture device: %v\n", 0)
		return
	}
	defer webcam.Close()

	img := gocv.NewMat()
	defer img.Close()

	// load classifier to recognize faces
	fmt.Printf("NewCascadeClassifier()")
	classifier = gocv.NewCascadeClassifier()
	defer classifier.Close()

	fmt.Printf("CascadeClassifier.Load()")
	if !classifier.Load(xmlFile) {
		fmt.Printf("Error reading cascade file: %v\n", xmlFile)
		return
	}

	host := ":8080"

	// create the mjpeg stream
	stream := mjpeg.NewStream()

	// start capturing
	go mjpegCapture(webcam, stream)

	fmt.Printf("Capturing. Point your browser to %s\n", host)

	// start http server
	http.Handle("/", stream)
	log.Fatal(http.ListenAndServe(host, nil))
}

func trackFace(frame gocv.Mat) {
	W := float64(frame.Cols())
	// H := float64(frame.Rows())
	rects := classifier.DetectMultiScale(frame)
	fmt.Printf("found %d faces\n", len(rects))

	maxRect := struct {
		left  float64
		right float64
		area  int
	}{}

	detected = false

	for _, r := range rects {
		left = float64(r.Min.X)
		// top = float64(detections.GetFloatAt(r, 4)) * H
		right = float64(r.Max.X)
		// bottom = float64(detections.GetFloatAt(r, 6)) * H

		left = math.Min(math.Max(0.0, left), W-1.0)
		right = math.Min(math.Max(0.0, right), W-1.0)
		// bottom = math.Min(math.Max(0.0, bottom), H-1.0)
		// top = math.Min(math.Max(0.0, top), H-1.0)

		detected = true

		rect := image.Rect(int(left), int(top), int(right), int(bottom))
		gocv.Rectangle(&frame, rect, green, 3)
		s := rect.Size().X * rect.Size().Y
		gocv.Rectangle(&frame, rect, green, 3)
		if s > maxRect.area {
			fmt.Printf("s > maxRect.area")
			maxRect.area = s
			maxRect.left = left
			maxRect.right = right
		}
	}

	//left = maxRect.left
	//right = maxRect.right

	center := (right + left) / 2.0


	//if !detected {
	//	setThrottle(0)
	//} else {
	//	setThrottle(-0.25)
	//}

	setThrottle(-0.1990014343699454)

	if !detected {
		return
	}

	// Turn the car steering left or right based on whether the face image is to the left or right of center

	steering := 0.0

	if center < W/2 {
		steering = 0.5

	} else {
		steering = -0.5

	}
	setSteering(steering)

	fmt.Printf("steering: %v, face center: %v, frame center: %v, left: %v, right: %v", steering, center, W/2, left, right)


}

func dist(x1, y1, x2, y2 float64) float64 {
	return math.Sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
}

func handleJoystick() {

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
	// fmt.Printf("setSteering to %v\n", steering)
}

func setThrottle(throttle float64) {
	throttleVal := getThrottlePulse(throttle)
	// throttleVal := getThrottlePulseSlow()
	pca9685.SetPWM(0, 0, uint16(throttleVal))
	// fmt.Printf("setThrottle to %v\n", throttle)
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

func getThrottlePulseSlow() int {
	// -0.1590014343699454
	return int(420) // 380
}

func mjpegCapture(webcam *gocv.VideoCapture, stream *mjpeg.Stream) {
	img := gocv.NewMat()
	defer img.Close()

	for {
		if ok := webcam.Read(&img); !ok {
			fmt.Println("Device closed.")
			return
		}
		if img.Empty() {
			continue
		}

		trackFace(img)

		buf, _ := gocv.IMEncode(".jpg", img)
		stream.UpdateJPEG(buf)
	}
}
