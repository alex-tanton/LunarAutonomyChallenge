#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import datetime
import logging
import math
import random
import re
import weakref

try:
    import pygame
    from pygame.locals import K_UP
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_F1
    from pygame.locals import K_TAB
    from pygame.locals import K_1
    from pygame.locals import K_2
    from pygame.locals import K_3
    from pygame.locals import K_4
    from pygame.locals import K_5
    from pygame.locals import K_6
    from pygame.locals import K_7
    from pygame.locals import K_8
    from pygame.locals import K_9
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_j
    from pygame.locals import K_c
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):

        self.world = carla_world
        self.sync = args.sync
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.camera_manager = None
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self):
        self.player_max_drum_speed = 1.589
        self.player_max_drum_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = self.world.get_blueprint_library().find("vehicle.ipex.ipex")
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_drum_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_drum_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        self.world.set_presetid(0, False)

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        else:
            spawn_point = carla.Transform(carla.Location(8.0, -4.0, 0.0),carla.Rotation(yaw=40))
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)

        if self.player is None:
            raise ValueError("Couldn't spawn the vehicle")

        # Set up the sensors.
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        sensors = [
            self.camera_manager.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self):

        self.linear_target_speed = 0
        self.angular_target_speed = 0
        self.front_drum_target_speed = 0
        self.front_arm_target_angle = 0.79
        self.back_arm_target_angle = 0.79
        self.back_drum_target_speed = 0

        self._linear_speed_increase = 0.01
        self._angular_speed_increase = 0.01
        self._drum_speed_increase = 0.017
        self._arm_angle_increase = 0.017

        self._max_linear_speed = 0.5
        self._max_angular_speed = 0.5
        self._max_arm_angle = 2.36
        self._max_drum_speed = 0.17

        self._lights_increase = 10
        self.lights_value = {
            carla.SensorPosition.Front: 0.0,
            carla.SensorPosition.FrontLeft: 0.0,
            carla.SensorPosition.FrontRight: 0.0,
            carla.SensorPosition.Left: 0.0,
            carla.SensorPosition.Right: 0.0,
            carla.SensorPosition.BackLeft: 0.0,
            carla.SensorPosition.BackRight: 0.0,
            carla.SensorPosition.Back: 0.0
        }

    def parse_events(self, client, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_TAB and pygame.key.get_mods() & KMOD_SHIFT:
                    world.camera_manager.toggle_camera(reverse=True)
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                elif event.key == K_o:
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p:
                    client.stop_recorder()
                    world.recording_enabled = False
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)

                if event.key == K_x:
                    world.player.set_radiator_cover_state(carla.RadiatorCoverState.Open)
                    world.hud.notification("Opening the radiator cover")
                elif event.key == K_c:
                    world.player.set_radiator_cover_state(carla.RadiatorCoverState.Close)
                    world.hud.notification("Closing the radiator cover")

                sensor = None
                if event.key == K_1:
                    sensor = carla.SensorPosition.Front
                elif event.key == K_2:
                    sensor = carla.SensorPosition.FrontLeft
                elif event.key == K_3:
                    sensor = carla.SensorPosition.FrontRight
                elif event.key == K_4:
                    sensor = carla.SensorPosition.Left
                elif event.key == K_5:
                    sensor = carla.SensorPosition.Right
                elif event.key == K_6:
                    sensor = carla.SensorPosition.BackLeft
                elif event.key == K_7:
                    sensor = carla.SensorPosition.BackRight
                elif event.key == K_8:
                    sensor = carla.SensorPosition.Back
                if sensor is not None:
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        light_value = max(min(self.lights_value[sensor] - self._lights_increase, 100.0), 0.0)
                    else:
                        light_value = max(min(self.lights_value[sensor] + self._lights_increase, 100.0), 0.0)
                    self.lights_value[sensor] = light_value
                    world.player.set_light_state(sensor, light_value/100)
                    world.hud.notification(f"Turning {sensor} light to {light_value}%")

        keys = pygame.key.get_pressed()

        if keys[K_UP] or keys[K_w]:
            self.linear_target_speed = min(self.linear_target_speed + self._linear_speed_increase, self._max_linear_speed)
        elif keys[K_DOWN] or keys[K_s]:
            self.linear_target_speed = max(self.linear_target_speed - self._linear_speed_increase, -self._max_linear_speed)
        else:
            self.linear_target_speed = max(self.linear_target_speed - 2* self._linear_speed_increase, 0.0)

        if keys[K_LEFT] or keys[K_a]:
            self.angular_target_speed = max(self.angular_target_speed - self._angular_speed_increase, -self._max_angular_speed)
        elif keys[K_RIGHT] or keys[K_d]:
            self.angular_target_speed = min(self.angular_target_speed + self._angular_speed_increase, self._max_angular_speed)
        else:
            self.angular_target_speed = max(self.angular_target_speed - 2* self._angular_speed_increase, 0.0)

        world.player.apply_velocity_control(carla.VehicleVelocityControl(self.linear_target_speed, self.angular_target_speed))

        if keys[K_f]:
            self.front_drum_target_speed = min(self.front_drum_target_speed + self._drum_speed_increase, self._max_drum_speed)
            world.player.set_front_drums_target_speed(self.front_drum_target_speed)
        elif keys[K_v]:
            self.front_drum_target_speed = max(self.front_drum_target_speed - self._drum_speed_increase, -self._max_drum_speed)
            world.player.set_front_drums_target_speed(self.front_drum_target_speed)
        if keys[K_g]:
            self.front_arm_target_angle = min(self.front_arm_target_angle + self._arm_angle_increase, self._max_arm_angle)
            world.player.set_front_arm_angle(self.front_arm_target_angle)
        elif keys[K_b]:
            self.front_arm_target_angle = max(self.front_arm_target_angle - self._arm_angle_increase, -self._max_arm_angle)
            world.player.set_front_arm_angle(self.front_arm_target_angle)
        if keys[K_h]:
            self.back_arm_target_angle = min(self.back_arm_target_angle + self._arm_angle_increase, self._max_arm_angle)
            world.player.set_back_arm_angle(self.back_arm_target_angle)
        elif keys[K_n]:
            self.back_arm_target_angle = max(self.back_arm_target_angle - self._arm_angle_increase, -self._max_arm_angle)
            world.player.set_back_arm_angle(self.back_arm_target_angle)
        if keys[K_j]:
            self.back_drum_target_speed = min(self.back_drum_target_speed + self._drum_speed_increase, self._max_drum_speed)
            world.player.set_back_drums_target_speed(self.back_drum_target_speed)
        elif keys[K_m]:
            self.back_drum_target_speed = max(self.back_drum_target_speed - self._drum_speed_increase, -self._max_drum_speed)
            world.player.set_back_drums_target_speed(self.back_drum_target_speed)

    @staticmethod
    def _is_quit_shortcut(key):
        return key == K_ESCAPE or key == K_q

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height, controller):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self._controller = controller

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            '',
            'Target values:',
            '',
            ' Linear speed:     %5.2f   m/s' % (self._controller.linear_target_speed),
            ' Angular speed:    %5.2f rad/s' % (self._controller.angular_target_speed),
            ' Front Drum speed: %5.2f rad/s' % (self._controller.front_drum_target_speed),
            ' Front Arm angle:  %5.2f   rad' % (self._controller.front_arm_target_angle),
            ' Back Arm angle:   %5.2f   rad' % (self._controller.back_arm_target_angle),
            ' Back Drum speed:  %5.2f rad/s' % (self._controller.back_drum_target_speed),
            '',
            'Current power        %3.0f Wh' % (world.player.get_current_power()),
            'Consumed power       %3.0f Wh' % (world.player.get_consumed_power()),
            '',
            'Lights values:',
            '',
            ' Front:           %3.0d%%' % (self._controller.lights_value[carla.SensorPosition.Front]),
            ' FrontLeft:       %3.0d%%' % (self._controller.lights_value[carla.SensorPosition.FrontLeft]),
            ' FrontRight:      %3.0d%%' % (self._controller.lights_value[carla.SensorPosition.FrontRight]),
            ' Left:            %3.0d%%' % (self._controller.lights_value[carla.SensorPosition.Left]),
            ' Right:           %3.0d%%' % (self._controller.lights_value[carla.SensorPosition.Right]),
            ' BackLeft:        %3.0d%%' % (self._controller.lights_value[carla.SensorPosition.BackLeft]),
            ' BackRight:       %3.0d%%' % (self._controller.lights_value[carla.SensorPosition.BackRight]),
            ' Back:            %3.0d%%' % (self._controller.lights_value[carla.SensorPosition.Back]),
            '']
    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        Attachment = carla.AttachmentType

        self._camera_transforms = [
            (carla.SensorPosition.FrontLeft, carla.Transform(), Attachment.Rigid, 'FrontLeftCamera'), # Front Left
            (carla.SensorPosition.FrontRight, carla.Transform(), Attachment.Rigid, 'FrontRightCamera'), # Front Right
            (carla.SensorPosition.Left, carla.Transform(), Attachment.Rigid, 'LeftCamera'), # Left
            (carla.SensorPosition.Right, carla.Transform(), Attachment.Rigid, 'RightCamera'), # Right
            (carla.SensorPosition.BackLeft, carla.Transform(), Attachment.Rigid, 'BackLeftCamera'), # Back Left
            (carla.SensorPosition.BackRight, carla.Transform(), Attachment.Rigid, 'BackRightCamera'), # Back Right
            (carla.SensorPosition.Back, carla.Transform(), Attachment.Rigid, 'BackArmCamera'), # Drum Back
            (carla.SensorPosition.Front, carla.Transform(), Attachment.Rigid, 'FrontArmCamera') # Drum Front
        ]

        self.transform_index = 1
        self.sensors = [['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}]]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                bp.set_attribute('fov', "70")
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', "2.2")
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self, reverse=False):
        if not reverse:
            self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        else:
            self.transform_index = (self.transform_index - 1) % len(self._camera_transforms)
        self.set_sensor(self.index, force_respawn=True)

    def set_sensor(self, index, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
                self._parent.set_camera_state(carla.SensorPosition.All, False)
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][1],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][2],
                bone=self._camera_transforms[self.transform_index][3])

            self._parent.set_camera_state(self._camera_transforms[self.transform_index][0], True)
            self.hud.notification(f"Changing to {self._camera_transforms[self.transform_index][0].name} camera")
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        self.index = index

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        controller = KeyboardControl()
        hud = HUD(args.width, args.height, controller)
        world = World(sim_world, hud, args)

        # Initialize the vehicle
        world.player.set_front_arm_angle(controller.front_arm_target_angle)
        world.player.set_back_arm_angle(controller.back_arm_target_angle)
        world.player.set_light_state(carla.SensorPosition.All, 0.0)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            world.tick(clock)
            world.render(display)

            pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1080x720',
        help='window resolution (default: 1080x720)')
    argparser.add_argument(
        '--sync',
        default=True,
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
