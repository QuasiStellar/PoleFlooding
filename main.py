from time import time
from random import uniform, random
from math import sqrt, cos, sin, pi, hypot
from vpython import ellipsoid, sphere, arrow, vector, color, rate, button, scene, mag

ARROWS = True
ARROW_V_MULTIPLIER = 1000
ARROW_A_MULTIPLIER = 10000000
SCALE = 100000
EQUATORIAL_R = 6378137  # m
POLAR_R = 6356752.3  # m
WATER_V = 146 * 10**16  # m^3
PARTICLES_COUNT = 1000
PARTICLE_V = WATER_V / PARTICLES_COUNT
PARTICLE_R = (PARTICLE_V * 3 / 4 / pi)**(1/3)  # m
EARTH_M = 5.9722 * 10**24  # kg
GRAVITATIONAL_CONSTANT = 6.67408 * 10**-11  # m^3 / kg s^2
BOOST = 10
DECELERATION = 0.0001  # m / s^2
FPS = 60

running = False
del scene.lights[1]
scene.width = 1000
scene.height = 1000


def pause(_button):
    global running
    if running:
        _button.text = "Run"
    else:
        _button.text = "Pause"
    running = not running
    return


button(text="Run", pos=scene.title_anchor, bind=pause)


def random_surface_location():
    z = uniform(-POLAR_R, POLAR_R)
    n = sqrt(EQUATORIAL_R ** 2 * (1 - z ** 2 / POLAR_R ** 2))
    angle = random() * 2 * pi
    x, y = cos(angle) * n, sin(angle) * n
    return vector(x / SCALE, y / SCALE, z / SCALE)


class WaterParticle:
    def __init__(self):
        self.model = sphere(pos=random_surface_location(), radius=PARTICLE_R / SCALE, color=color.blue)
        v = 2 * pi * hypot(self.model.pos.x, self.model.pos.z) / 24 / 3600
        self.velocity = vector(self.model.pos.z * v / mag(self.model.pos),
                               0,
                               -self.model.pos.x * v / mag(self.model.pos))
        self.acceleration = vector(0, 0, 0)
        self.velocity_arrow = arrow(pos=self.model.pos,
                                    axis=self.velocity,
                                    length=mag(self.velocity) * ARROW_V_MULTIPLIER,
                                    color=color.blue,
                                    round=True)
        self.velocity_arrow.shaftwidth = 0.2
        self.acceleration_arrow = arrow(pos=self.model.pos,
                                        axis=self.acceleration,
                                        length=0,
                                        color=color.red,
                                        round=True)

    def inside_earth(self):
        return hypot(self.model.pos.x * SCALE / EQUATORIAL_R,
                     self.model.pos.y * SCALE / POLAR_R,
                     self.model.pos.z * SCALE / EQUATORIAL_R) < 1

    def update_velocity_arrows(self):
        self.velocity_arrow.pos = self.model.pos
        self.velocity_arrow.axis = self.velocity
        self.velocity_arrow.length = mag(self.velocity) * ARROW_V_MULTIPLIER
        self.velocity_arrow.shaftwidth = 0.2

    def update_acceleration_arrows(self):
        self.acceleration_arrow.pos = self.model.pos
        self.acceleration_arrow.axis = self.acceleration
        self.acceleration_arrow.length = mag(self.acceleration) * ARROW_A_MULTIPLIER
        self.acceleration_arrow.shaftwidth = 0.2


earth = ellipsoid(length=EQUATORIAL_R * 2 / SCALE, height=POLAR_R * 2 / SCALE, width=EQUATORIAL_R * 2 / SCALE)
particles = [WaterParticle() for i in range(PARTICLES_COUNT)]


start_time = time()
while "my guitar gently weeps":
    if running:
        rate(FPS)
        cur_time = time()
        start_time = cur_time
        for particle in particles:
            center_distance = hypot(particle.model.pos.x * SCALE,
                                    particle.model.pos.y * SCALE,
                                    particle.model.pos.z * SCALE)
            a = GRAVITATIONAL_CONSTANT * EARTH_M / center_distance**2 / SCALE
            particle.acceleration = -a * particle.model.pos / mag(particle.model.pos)
            if particle.inside_earth():
                vec = vector(-POLAR_R**2 / EQUATORIAL_R**2 * particle.model.pos.x / particle.model.pos.y,
                             -1,
                             -POLAR_R**2 / EQUATORIAL_R**2 * particle.model.pos.z / particle.model.pos.y)
                particle.acceleration -= particle.acceleration.proj(vec)
                particle.velocity -= particle.velocity.proj(vec)
                if ARROWS:
                    particle.update_acceleration_arrows()
            particle.velocity += particle.acceleration * BOOST
            particle.velocity *= 1 - DECELERATION * BOOST
            particle.model.pos += particle.velocity * BOOST
            if ARROWS:
                particle.update_velocity_arrows()
