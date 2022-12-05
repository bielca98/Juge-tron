from controller import Robot
from cmath import rect, polar
import numpy as np
import struct
import time
from visio.obstacle_detection import ObstacleDetection
from visio.custom_detection import ObjectDetection
from visio.yolo import Yolo
from pathplanner import PathPlanner, State

SPHERE_R = 0.5
WHEEL_R = 0.03


class Sphero(Robot):
    VELOCITY_MSG = 0
    velocity = [0, 0, 0]
    polarVel = [0, 0]
    cameraPosition = 0
    speed = 5
    rotationSpeed = 1 * np.pi / 180
    maxVelocity = 0
    lastMessage = None
    imageShape = None

    def __init__(self, yolo=False):
        super(Sphero, self).__init__()

        # Get devices and enable
        self.timeStep = int(self.getBasicTimeStep())
        self.camera = self.getDevice('camera')
        h, w = self.camera.getHeight(), self.camera.getWidth()
        self.imageShape = h, w
        self.emitter = self.getDevice('emitter')
        self.receiver = self.getDevice('receiver')
        self.accelerometer = self.getDevice('accelerometer')
        self.pen = self.getDevice('pen')
        self.gyro = self.getDevice('gyro')
        self.cameraMotor = self.getDevice('cameraMotor')

        self.gyro.enable(self.timeStep)
        self.accelerometer.enable(self.timeStep)
        self.camera.enable(self.timeStep)
        self.receiver.enable(self.timeStep)
        self.keyboard.enable(32)
        self.keyboard = self.getKeyboard()

        self.motors = []
        for i in range(3):
            m = self.getDevice(f"wheel{i}")
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            self.motors.append(m)
        self.maxVelocity = int(self.motors[0].getMaxVelocity())

        self.obstacle_detector = ObstacleDetection()
        if yolo:
            self.cat_detector = Yolo()
        else:
            self.cat_detector = ObjectDetection()
        self.pathplanner = PathPlanner(*self.imageShape, debug=True)

    def sleep(self, ms):
        """
        Async sleep

        Parameter
        ---------
        ms: int
            Time to sleep in miliseconds
        """
        a = int(ms / 1000 * 1024)
        self.step(a)

    def setCamera(self, rad=None):
        """
        Set the direccion of the camera.
        If not given any ang, it will point towards the forward direcction.

        Parameter
        ---------
        rad: float, optional
            Angle in radians.
        """
        if rad is not None:
            self.cameraMotor.setPosition(rad)
            return
        v1 = rect(self.velocity[0], 0)
        v2 = rect(self.velocity[1], -2/3 * np.pi)
        v3 = rect(self.velocity[2], 2/3 * np.pi)
        s = np.array([v1, v2, v3]).sum(axis=0)
        s = polar(s)
        if s[0] != 0:
            self.cameraMotor.setPosition(s[1])

    def setVelocity(self):
        """
        Set the velocity of the motors based on `velocity`

        Notes
        -----
        Caps the velocity to `maxVelocity`
        """
        self.velocity = [self.maxVelocity if v >
                         self.maxVelocity else v for v in self.velocity]
        self.velocity = [-self.maxVelocity if v < -
                         self.maxVelocity else v for v in self.velocity]

        self.sendData()
        for i, j in zip(self.velocity, self.motors):
            j.setVelocity(i)

    def findMotors(self, rad):
        """
        Find which two motors needs to be turned on so that robot
        moves in the direccion of `rad`

        Parameter
        ---------
        rad: float
            Angle in radians

        Returns
        -------
        out: ndarray
            index of the motors
        """
        # p1 is the direction of velocity.
        # p2 contains the directions of the motors and their inverse
        # first 3 are for positive velocity and the other for negative velocity.
        # Computes the distance from p1 to each point in p2 and
        # return the index in mod 3 of the two smallest distances.

        p1 = np.array([(1, rad)], dtype=np.float32).reshape(-1, 2)
        p2 = np.array([(1, 0),
                       (1, -2/3 * np.pi),
                       (1, 2/3 * np.pi),
                       (1, np.pi),
                       (1, 1/3 * np.pi),
                       (1, -1/3 * np.pi)], dtype=np.float32).reshape(-1, 2)

        ang1, ang2 = p1[:, 1], p2[:, 1]
        d = np.sqrt(2 * (1 - np.cos(ang2 - ang1)))
        m = np.argpartition(d, axis=0, kth=2)[:2]
        m[m > 2] -= 6
        m2 = m.copy()
        m2[m2 < 0] += 3
        m3 = 0
        if (m2[0] == 0 and m2[1] == 1) or (m2[0] == 1 and m2[1] == 0):
            m3 = 2
        elif (m2[0] == 0 and m2[1] == 2) or (m2[0] == 2 and m2[1] == 0):
            m3 = 1
        elif (m2[0] == 1 and m2[1] == 2) or (m2[0] == 2 and m2[1] == 1):
            m3 = 0
        # print(m2, m3)
        return m, m3

    def movePolar(self):
        """
        Convert the velocity `r` and direction `rad` to motor velocities
        """
        r, rad = self.polarVel
        if r > self.maxVelocity:
            r = self.maxVelocity

        if r < -self.maxVelocity:
            r = -self.maxVelocity

        self.polarVel[0] = r

        v = rect(r, rad)
        v = np.abs([v.real, v.imag])
        v1, v2 = v.max(), v.min()
        (m1, m2), m3 = self.findMotors(rad)

        v1 = -v1 if m1 < 0 else v1
        v2 = -v2 if m2 < 0 else v2

        v1 = -v1 if r < 0 else v1
        v2 = -v2 if r < 0 else v2

        vel = [0, 0, 0]
        vel[int(m1)] = v1
        vel[int(m2)] = v2

        self.velocity = vel

        self.setVelocity()
        self.motors[int(m3)].setVelocity(-r)
        # print([m1, m2, m3], r, rad * 180 / np.pi)

    def substract_polar(self, v1, v2):
        v1 = rect(*v1)
        v2 = rect(*v2)
        v3 = v1 - v2
        # print(v1, v2, v3, polar(v3))
        return polar(v3)

    def stabilize(self):
        self.stop()

    def forward(self, speed=None, cameraLock=False):
        '''
        Move the robot in the forward direction.
        If `cameraLock` is set then the camera will be fixed in place.

        Parameter
        ---------
        Speed: int, optional
            Defualt is `self.speed`, can be positive or negative (backwards)

        cameraLock: bool, optional
            Camera-free movement.
        '''
        self.polarVel[0] += self.speed if speed is None else speed
        self.movePolar()
        if not cameraLock:
            self.setCamera(self.cameraPosition)

    def rotate(self, rotation=None, cameraLock=False):
        '''
        Rotate the robot by `rotationSpeed` radians.
        If `cameraLock` is set then the camera will be fixed in place.

        Parameter
        ---------
        rotationSpeed: int, optional
            Defualt is `self.rotationSpeed`, can be positive (right) or negative (left)

        cameraLock: bool, optional
            Camera-free rotation.
        '''
        self.polarVel[1] += self.rotationSpeed if rotation is None else rotation
        if cameraLock:
            self.movePolar()
            return
        self.cameraPosition += self.rotationSpeed if rotation is None else rotation
        self.movePolar()
        self.setCamera(self.cameraPosition)

    def changeDirection(self, cameraLock=False):
        '''
        Reverse the direcction of movement.
        If `cameraLock` is set then the camera will be fixed in place.

        Parameter
        ---------
        cameraLock: bool, optional
            Camera-free rotation.
        '''
        if not cameraLock:
            self.polarVel[1] -= np.pi
            self.setCamera(self.cameraPosition)
        self.polarVel[0] = -self.polarVel[0]
        self.movePolar()

    def stop(self):
        '''
        Stop the robot
        '''
        self.polarVel[0] = 0
        self.cameraPosition = self.polarVel[1]
        self.movePolar()
        self.setCamera(self.cameraPosition)

    def sendData(self):
        '''
        Send data to the plugin
        '''
        s = struct.pack('ifff', self.VELOCITY_MSG, *self.velocity)
        if s != self.lastMessage:
            # print(s)
            self.lastMessage = s
            self.emitter.send(s)

    def getGyroValues(self):
        '''
        Get raw gyroscope values.

        Returns
        -------
        out: list
            angular velocity along the x, y, z axes in rad/s
        '''
        return self.gyro.getValues()

    def getAccValues(self):
        '''
        Get raw accelerometer values.

        Returns
        -------
        out: list
            acceleration along the x, y, z axes in m/s
        '''
        return self.accelerometer.getValues()

    def penWrite(self, write):
        self.pen.write(write)

    def controlPolar(self):
        '''
        Control the robot using the keyboard.

        W, S, A, D

        To move the robot without moving the camera use.

        I, K, J, L

        Stop the robot by pressing 

        SPACE
        '''
        key = self.keyboard.getKey()

        if key == ord('W'):
            self.forward(self.speed)

        if key == ord('S'):
            self.forward(-self.speed)

        if key == ord('A'):
            self.rotate(-self.rotationSpeed)

        if key == ord('D'):
            self.rotate(self.rotationSpeed)

        if key == ord('Q'):
            self.changeDirection()

        if key == ord('I'):
            self.forward(-self.speed, cameraLock=True)

        if key == ord('K'):
            self.forward(-self.speed, cameraLock=True)

        if key == ord('J'):
            self.rotate(-self.rotationSpeed, cameraLock=True)

        if key == ord('L'):
            self.rotate(-self.rotationSpeed, cameraLock=True)

        if key == ord('U'):
            self.changeDirection(cameraLock=True)

        if key == ord(' '):
            self.stop()

    def getImage(self):
        buffer = self.camera.getImage()
        h, w = self.imageShape
        img = np.frombuffer(buffer, np.uint8).reshape(
            (h, w, 4))[:, :, :3]
        return img

    def next_move(self, next_direction):
        v, drad = next_direction[0], next_direction[1]
        self.polarVel[0] = v
        self.polarVel[1] += drad
        self.cameraPosition += drad
        self.movePolar()
        self.setCamera(self.cameraPosition)

    def run(self):
        fps = 1/10
        last_frame_time = 0
        next_direction = None

        while self.step(self.timeStep) != -1:
            frame_time = time.time()

            if last_frame_time + fps < frame_time:
                gyro = self.getGyroValues()
                image = self.getImage()
                state = self.pathplanner.current_state.state
                vision_image = None

                if state == State.SEARCH_CAT:
                    # fps = 1
                    vision_image = self.cat_detector.show_inference_and_return(
                        image)
                else:
                    # fps = 1 / 10
                    vision_image = self.obstacle_detector.detect(
                        image, draw=True)

                next_direction = self.pathplanner.next_direction(
                    gyro, vision_image)
                state = self.pathplanner.current_state.state

                last_frame_time = frame_time

                print("State:", state)
                d = self.polarVel[1] * 180 / np.pi
                s = f"Velocity: {self.polarVel[0]: 0.2f} m/s Direction {d: 0.2f} °"
                print(s)
            self.next_move(next_direction)


controller = Sphero(yolo=True)
controller.run()
