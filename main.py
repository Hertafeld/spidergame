# Example file showing a circle moving on screen
import pygame
import math

# pygame setup
pygame.init()
SIZE_X = 1280
SIZE_Y = 720
SCALE = 4

screen = pygame.display.set_mode((SIZE_X, SIZE_Y))
clock = pygame.time.Clock()
running = True

dt = 0.0
speed = 100
edge_buffer = 15

player = pygame.image.load('player2.png').convert()
player = pygame.transform.scale_by(player, 1.0/SCALE)
player.set_colorkey((255, 255, 255))
ray_miss = pygame.image.load('ray1.png').convert();
ray_miss = pygame.transform.scale_by(ray_miss, 1.0/SCALE)
ray_miss.set_colorkey((255, 255, 255))
ray_hit = pygame.image.load('ray2.png').convert();
ray_hit = pygame.transform.scale_by(ray_hit, 1.0/SCALE)
ray_hit.set_colorkey((255, 255, 255))
ray_closest = pygame.image.load('ray3.png').convert();
ray_closest = pygame.transform.scale_by(ray_closest, 1.0/SCALE)
ray_closest.set_colorkey((255, 255, 255))

leg_base = pygame.image.load('leg.png').convert();
leg_base = pygame.transform.scale_by(leg_base, 1.3/SCALE)
leg_base.set_colorkey((255, 255, 255))
leg_end = pygame.image.load('leg.png').convert();
leg_end = pygame.transform.scale_by(leg_end, (1.5/SCALE, 1.0/SCALE))
leg_end.set_colorkey((255, 255, 255))

background = pygame.image.load('level3.png').convert()

position = player.get_rect()
position.y = 300
position.x = 200

screen.blit(background, (0, 0))

rays = 12
ray_delta = 1
ray_range = 200/SCALE
ray_ticks = int(ray_range / ray_delta)

can_jump = True

ray_spots = []
ray_rects = []
ray_vectors = []
ray_collisions = []
ray_collisions_prev = []
closest_ray_index = -1
closest_ray_distance = -1

force = pygame.math.Vector2(0, 0)
velocity = pygame.math.Vector2(0, 0)

MASS = 1
G = 1.0
GRAVITY = pygame.math.Vector2(0, MASS*G)
AIR_RESISTANCE = 0.10

MUSCLE_STAND_FORCE = 2.0
MUSCLE_CLING_FORCE = 0.5
MUSCLE_DAMP_FORCE = 1.0
AUX_MUSCLE_FORCE = 0.0
MUSCLE_WALK_FORCE = 3.0
JUMP_FORCE = 1.0
TARGET_LEG_DISTANCE = 80/SCALE

leg_end_test = leg_end.copy()
leg_end_rect = leg_end_test.get_rect()
leg_end_rect.x = 600
leg_end_rect.y = 350

class Leg:
    def __init__(self, screen, background):
        self.screen = screen
        self.background = background

        self.visible = False
        self.base_surface = pygame.transform.rotate(leg_base, 0)
        self.base_rect = self.base_surface.get_rect()
        self.end_surface = pygame.transform.rotate(leg_end, 0)
        self.end_rect = self.end_surface.get_rect()

        self.base_length = self.base_rect.width
        self.end_length = self.end_rect.width

        # These are computed from the terminal point
        self.base_angle = 0
        self.joint_angle = 0

        # This determines the angles above
        self.terminal_point = pygame.math.Vector2(0, 0)

        #self.updateAnglesFromTerminalPoint()
        #self.updateSurfacesFromAngles()

    def erase(self):
        self.screen.blit(self.background, self.base_rect, self.base_rect)
        self.screen.blit(self.background, self.end_rect, self.end_rect)

    def draw(self):
        if not self.visible:
            return
        self.screen.blit(self.base_surface, self.base_rect)
        self.screen.blit(self.end_surface, self.end_rect)

    def updateAnglesFromTerminalPoint(self):
        inversion = 1
        body_to_terminal = self.terminal_point - pygame.math.Vector2(position.centerx, position.centery)

        # Decide which way to "fold" the legs based on closest surface
        if closest_ray_index != -1 and (body_to_terminal.angle_to(ray_vectors[closest_ray_index]) + 360) % 360 > 180:
            inversion = -1
        a = self.base_length
        b = self.end_length
        c = body_to_terminal.magnitude()

        cos_base_angle = (c*c+a*a-b*b)/(2*a*c)
        if cos_base_angle > 1 or cos_base_angle < -1:
            self.visible = False
            return False
        
        body_to_terminal_angle = math.degrees(math.atan2(-body_to_terminal.y, body_to_terminal.x))
        print("Terminal vector: (%.1f, %.1f) BtT angle: %.1f" % (body_to_terminal.x, body_to_terminal.y, body_to_terminal_angle))
        self.base_angle = body_to_terminal_angle + inversion*math.degrees(math.acos(cos_base_angle))

        cos_joint_angle = (a*a+b*b-c*c) / (2*a*b)
        print(cos_joint_angle)

        if cos_joint_angle > 1 or cos_joint_angle < -1:
            self.visible = False
            return False
        self.joint_angle = (self.base_angle-180) + inversion* math.degrees(math.acos(cos_joint_angle))
        self.visible = True
        return True

    def updateSurfacesFromAngles(self):
        self.base_surface = pygame.transform.rotate(leg_base, self.base_angle)
        self.end_surface = pygame.transform.rotate(leg_end, self.joint_angle)
        
        self.base_rect = self.base_surface.get_rect()
        self.base_rect.x = position.centerx
        self.base_rect.y = position.centery
        self.base_angle = self.base_angle % 360
        self.joint_angle = self.joint_angle % 360
        if self.base_angle > 90 and self.base_angle < 270:
            self.base_rect.x -= self.base_rect.width
        if self.base_angle < 180:
            self.base_rect.y -= self.base_rect.height

        self.end_rect = self.end_surface.get_rect()
        self.end_rect.x = position.centerx + math.cos(math.radians(self.base_angle)) * self.base_length
        self.end_rect.y = position.centery - math.sin(math.radians(self.base_angle)) * self.base_length
        if self.joint_angle > 90 and self.joint_angle < 270:
            self.end_rect.x -= self.end_rect.width
        if self.joint_angle < 180:
            self.end_rect.y -= self.end_rect.height


def in_bounds(x, y):
    return x >= edge_buffer and x <= SIZE_X - edge_buffer and y >= edge_buffer and y <= SIZE_Y - edge_buffer

def shine_rays():
    ray_spots = []
    ray_vectors = []
    ray_rects = []
    ray_collisions = []
    closest_ray = -1
    closest_ray_distance = ray_ticks+1

    for ray_index in range(rays):
        collision = False
        x = 0
        y = 0
        for ray_distance in range(ray_ticks):
            x = int(position.centerx + ray_delta * ray_distance * math.sin(3.14150*2*(ray_index/rays)))
            y = int(position.centery + ray_delta * ray_distance * math.cos(3.14150*2*(ray_index/rays)))
            if (not in_bounds(x, y) or background.get_at((x, y)) != pygame.Color(255, 255, 255)):
                ray_spots.append((x, y))
                ray_vectors.append(pygame.math.Vector2(x - position.centerx, y - position.centery))
                ray_rects.append(pygame.Rect((x-ray_hit.get_size()[0]/2, y-ray_hit.get_size()[1]/2), ray_hit.get_size()))
                ray_collisions.append(True)
                collision = True
                if closest_ray == -1 or ray_distance < closest_ray_distance:
                    closest_ray_distance = ray_distance
                    closest_ray = ray_index
                break
        if not collision:
            ray_spots.append(pygame.Vector2(x, y))
            ray_vectors.append(pygame.math.Vector2(x - position.centerx, y - position.centery))
            ray_rects.append(pygame.Rect((x-ray_miss.get_size()[0]/2, y-ray_miss.get_size()[1]/2), ray_miss.get_size()))
            ray_collisions.append(False)
    return ray_spots, ray_rects, ray_vectors, ray_collisions, closest_ray, closest_ray_distance

MAX_LEGS = 6
LEG_RATE = 12
leg_spawn = 0
legs = []

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.blit(background, position, position)

    for rect in ray_rects:
        screen.blit(background, rect, rect)

    for leg in legs:
        leg.erase()

    # Poll keys
    keys = pygame.key.get_pressed()

    # Cast rays to see where the wall is
    ray_collisions_prev = ray_collisions
    ray_spots, ray_rects, ray_vectors, ray_collisions, closest_ray_index, closest_ray_distance = shine_rays()

    # Force is computed each frame
    force = pygame.math.Vector2(0, 0)

    # Gravity 
    force += GRAVITY

    # Air resistance
    force -= velocity * AIR_RESISTANCE

    if closest_ray_index != -1:
        # Leg standing force
        if ray_vectors[closest_ray_index].magnitude() != 0:
            displacement = ray_vectors[closest_ray_index] - ray_vectors[closest_ray_index].normalize() * TARGET_LEG_DISTANCE
        else:
            displacement = pygame.math.Vector2(0, 0)
        if closest_ray_distance > TARGET_LEG_DISTANCE:
            force += displacement * MUSCLE_CLING_FORCE
        else:
            force += displacement * MUSCLE_STAND_FORCE

        # Leg damping force
        force -= velocity * MUSCLE_DAMP_FORCE

        left_leg = (closest_ray_index - 1 + rays) % rays
        right_leg = (closest_ray_index + 1) % rays

        # Stabilizing force from adjacent legs
        for leg in [left_leg, right_leg]:
            if ray_collisions[leg]:
                if ray_vectors[leg].magnitude() != 0:
                    displacement = ray_vectors[leg] - ray_vectors[leg].normalize() * TARGET_LEG_DISTANCE
                else:
                    displacement = pygame.math.Vector2(0, 0)
                if closest_ray_distance > TARGET_LEG_DISTANCE:
                    force += displacement * MUSCLE_CLING_FORCE * AUX_MUSCLE_FORCE
                else:
                    force += displacement * MUSCLE_STAND_FORCE * AUX_MUSCLE_FORCE

        if keys[pygame.K_d]:
            if ray_collisions[right_leg]:
                walk_vector = ray_vectors[right_leg] - ray_vectors[closest_ray_index]
            else:
                walk_vector = ray_vectors[closest_ray_index].rotate(-90).normalize()
            if walk_vector.magnitude() > 0:
                force += MUSCLE_WALK_FORCE * walk_vector.normalize()
        if keys[pygame.K_a]:
            if ray_collisions[left_leg]:
                walk_vector = ray_vectors[left_leg] - ray_vectors[closest_ray_index]
            else:
                walk_vector = ray_vectors[closest_ray_index].rotate(90).normalize()
            if walk_vector.magnitude() > 0:
                force += MUSCLE_WALK_FORCE * walk_vector.normalize()
        if keys[pygame.K_SPACE] and can_jump:
            force -= ray_vectors[closest_ray_index] * JUMP_FORCE

        if keys[pygame.K_q]:
            position.y = 200
            velocity.y = 0
            velocity.x = 0
            position.x = 300

    else: # In the air
        if keys[pygame.K_SPACE]:
            can_jump = False
    
    if not keys[pygame.K_SPACE]:
        can_jump = True

    velocity += force/MASS
    position = position.move(velocity.x, velocity.y)

    leg_index = 0
    while leg_index < len(legs):
        if not legs[leg_index].updateAnglesFromTerminalPoint():
            del(legs[leg_index])
        else:
            legs[leg_index].updateSurfacesFromAngles()
            legs[leg_index].draw()
            leg_index += 1

    # Find leading ray
    leading_ray_index = -1
    max_dot = -1
    v_norm = velocity.normalize()
    for i in range(rays):
        if ray_collisions[i]:
            if leading_ray_index == -1 or v_norm.dot(ray_vectors[i].normalize()) > max_dot:
                leading_ray_index = i
                max_dot = v_norm.dot(ray_vectors[i].normalize())

    # Spawn new legs
    # First, on a new ray contact
    for i in range(rays):
        if ray_collisions[i] and not ray_collisions_prev[i] and len(legs) < MAX_LEGS:
            legs.append(Leg(screen, background))
            legs[-1].terminal_point = pygame.math.Vector2(ray_spots[i])
    # Second, by walking
    leg_spawn += velocity.magnitude()
    if leg_spawn > LEG_RATE and leading_ray_index != -1 and len(legs) < MAX_LEGS:
        legs.append(Leg(screen, background))
        legs[-1].terminal_point = pygame.math.Vector2(ray_spots[leading_ray_index])
        leg_spawn = 0

    screen.blit(player, position)
    for ray_index in range(len(ray_spots)):
        if closest_ray_index == ray_index:
            #screen.blit(ray_closest, ray_rects[ray_index])
            pass
        elif ray_collisions[ray_index]:
            #screen.blit(ray_hit, ray_rects[ray_index])
            pass
        else:
            #screen.blit(ray_miss, ray_rects[ray_index])
            pass

    #test_leg.base_angle = pygame.time.get_ticks() / 10
    #test_leg.joint_angle = pygame.time.get_ticks() / 21
    

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000


pygame.quit()