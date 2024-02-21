import arcade
import random
import math
from threading import Thread

WIDTH = 1700
HEIGHT = 500
TITLE = "Cannon Simulator"

# Gravity constant [m/s^2]
G = 9.8

# Cannon class
class Cannon():
    
    def __init__(self, resource_path):
        self.wheel = arcade.Sprite(resource_path + "/resource/wheel.png", scale = 0.7)
        self.arm = arcade.Sprite(resource_path + "/resource/cannon.png", scale = 0.7)

        # Cannon fire speed [spaces/s]
        self.fire_speed = 10000
        
        # Base coordinates for placing the cannon
        self.x_0 = 50
        self.y_0 = 30
        self.arm_offset = 48
        
        self.wheel.center_x = self.x_0
        self.wheel.center_y = self.y_0
        self.arm.center_x = self.x_0 + self.arm_offset
        self.arm.center_y = self.y_0
        
    # Method to render cannon
    def render(self):
        self.arm.draw()
        self.wheel.draw()

    # Method to set cannon's angle (input params is in rad)
    def set_angle(self, angle):
        self.arm.angle = angle
        self.arm.center_x = self.x_0 + self.arm_offset * math.cos(math.radians(angle))
        self.arm.center_y = self.y_0 + self.arm_offset * math.sin(math.radians(angle))
        
        
class CannonBall(arcade.Sprite):
    
    def __init__(self, resource, spawn_x, spawn_y, speed, firing_angle):
        super().__init__(resource + "/resource/cannonball.png", scale = 0.5)
        self.center_x = spawn_x
        self.center_y = spawn_y
        self.vel_x = speed * math.cos(firing_angle)
        self.vel_y = speed * math.sin(firing_angle)
        
    def render(self):
        self.draw()
        
    def update(self, delta_time):
        # Update horizontal position
        self.center_x += self.vel_x * delta_time
        # Update vertical speed and position accordingly
        self.vel_y -= G
        self.center_y += self.vel_y * delta_time
        if self.center_y <= 10:
            self.center_y = 10
            self.vel_y = 0.0
            self.vel_x = 0.0
            
    def stop(self):
        self.vel_x = 0.0
        self.vel_y = 0.0
            
                    
class Target(arcade.Sprite):
    def __init__(self, resource, spawn_x):
        super().__init__(resource + "/resource/target.png", scale = 0.5)
        self.center_x = spawn_x
        self.center_y = 40
        
    def render(self):
        self.draw()
        
        
# Thread to run simulation in background
class SimThread(Thread):
   def __init__(self):
      Thread.__init__(self)
   
   def run(self):
      arcade.run()
            
# Main class representing simulation environment
class CannonSim(arcade.Window):

    def __init__(self, resources_path):
        super().__init__(WIDTH, HEIGHT, TITLE)
        arcade.set_background_color(arcade.color.LIGHT_GRAY)

        self.resources_path = resources_path
        self.thread = SimThread()

    def setup(self):
        # Instantiate cannon and variables
        self.cannon = Cannon(self.resources_path)
        self.firing_angle = 0.0
        # Create arrays for cannonballs and targets
        self.cannonballs = arcade.SpriteList(use_spatial_hash=True)
        self.targets = arcade.SpriteList(use_spatial_hash=True)
        self.cannonball_idx = 0
        self.target_idx = 0
        # Scene object for collisions
        self.scene = arcade.Scene()
        self.to_hit = {0,1,2}
        # Make targets in random positions (with some constraints)
        targets_spawned = 0
        targets_positions = []
        while targets_spawned < 3:
            random_x = random.randint(WIDTH/2, WIDTH-20)
            # If some target is already placed, check vicinity constraints during placement
            if targets_positions:
                distances = []
                for pos in targets_positions:
                    distances.append(abs(pos - random_x))
                if all(d > 200 for d in distances):
                    self.spawn_target(random_x)
                    targets_positions.append(random_x)
                    targets_spawned += 1
            # Else, place first target
            else:
                self.spawn_target(random_x)
                targets_positions.append(random_x)
                targets_spawned += 1
        # Set score to 0 and start simulation thread
        self.score = 0
        self.thread.start()
        
    # Internal routine to spawn target sprite
    def spawn_target(self, spawn_x):
        target = Target(self.resources_path, spawn_x)
        self.scene.add_sprite("Target {0}".format(self.target_idx), target)
        self.targets.append(target)
        self.target_idx += 1
        
    # Method to increase or decrease target fire speed
    def set_cannon_fire_speed(self, flag):
        # if True, increase speed by fixed amount
        if flag:
            self.cannon.fire_speed += 500
        # else, decrease
        else:
            self.cannon.fire_speed -= 500
    
    # Method to retrieve cannon's angle for control purposes
    def get_cannon_angle(self):
        return math.radians(self.firing_angle)
        
    # Method to rotate cannon
    def rotate_cannon(self, rotation_speed):
        self.firing_angle += rotation_speed
        self.cannon.set_angle(self.firing_angle)
        
    # Method to shoot new cannonball
    def shoot_cannon(self):
        cannonball = CannonBall(
            self.resources_path,
            int(self.cannon.x_0 + 2 * self.cannon.arm_offset * math.cos(math.radians(self.firing_angle))),
            int(self.cannon.y_0 + 2 * self.cannon.arm_offset * math.sin(math.radians(self.firing_angle))),
            self.cannon.fire_speed * 0.078,
            math.radians(self.firing_angle)
        )
        self.scene.add_sprite("CannonBall {0}".format(self.cannonball_idx), cannonball)
        self.cannonballs.append(cannonball)
        self.cannonball_idx += 1
        
    # Method to compute whether target is reachable with current firing speed and given target location
    def find_target_angle(self, target_x):
        value = (target_x - self.cannon.x_0 - 2 * self.cannon.arm_offset) * G / ((self.cannon.fire_speed * 0.01)**2)
        if value > 0 and value <= 1:
            return 0.5 * math.asin(value)
        else:
            return -1
                
    def draw_score_message(self):
        arcade.draw_text("SCORE: {0}".format(self.score),
            WIDTH - 200,
            HEIGHT - 50,
            arcade.color.BLACK,
            25)

    def on_draw(self):
        self.clear()
        self.cannon.render()
        self.cannonballs.draw()
        self.targets.draw()
        self.draw_score_message()
        
    def on_update(self, delta_time: float):
        for cannonball in self.cannonballs:
            cannonball.update(delta_time)
            for idx in self.to_hit:
                is_colliding_with_target = arcade.check_for_collision(cannonball, self.targets[idx])
                if is_colliding_with_target:
                    self.score += 1
                    self.to_hit.remove(idx)
                    cannonball.stop()
                    break


    

