#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Change lane scenario:

The scenario realizes a driving behavior, in which the user-controlled ego vehicle
follows a fast driving car on the highway. There's a slow car driving in great distance to the fast vehicle.
At one point the fast vehicle is changing the lane to overtake a slow car, which is driving on the same lane.

The ego vehicle doesn't "see" the slow car before the lane change of the fast car, therefore it hast to react
fast to avoid an collision. There are two options to avoid an accident:
The ego vehicle adjusts its velocity or changes the lane as well.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      StopVehicle,
                                                                      LaneChange,
                                                                      WaypointFollower,
                                                                      Idle, AtomicBehavior, ChangeAutoPilot)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle, StandStill, DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

from srunner.scenariomanager.timer import TimeOut

class VehicleLightsControls(AtomicBehavior):
    """
    A class to set lights status on a vehicle

    Important parameters:
    - name: Name of the atomic behavior
    - actor: the vehicle that we are changing the light_status for
    - light_status: the new light state to be set
    """

    def __init__(self, actor, light_status=carla.VehicleLightState(carla.VehicleLightState.NONE), name="VehicleLights"):
        """
        Default init. Has to be called via super from derived class
        """
        super(VehicleLightsControls, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.name = name
        self._actor = actor
        self.light_status = light_status

    def update(self):
        self._actor.set_light_state(self.light_status)
        new_status = py_trees.common.Status.SUCCESS
        return new_status
    
    def initialise(self):
        return


class DebugCheckPoint(AtomicBehavior):
    """
    A class to print out a message that scanario proceeded to a next step

    Important parameters:
    - name: Name of the atomic behavior
    - actor: the vehicle that we are changing the light_status for
    """

    is_enabled = True
    
    def __init__(self, actor, name="CheckPoint"):
        super(DebugCheckPoint, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.name = name
        self._actor = actor

    def update(self):
        if DebugCheckPoint.is_enabled:
            print(f'CHECKPOINT [{self._actor.type_id}] {self.name}')
        return py_trees.common.Status.SUCCESS
    
    def initialise(self):
        return

class ChangeLane(BasicScenario):

    """
    This class holds everything required for a "change lane" scenario involving three vehicles.
    There are two vehicles driving in the same direction on the highway: A fast car and a slow car in front.
    The fast car will change the lane, when it is close to the slow car.

    The ego vehicle is driving right behind the fast car.

    This is a single ego vehicle scenario
    """

    TESLA_VELOCITY = 25.0           # m/s
    TESLA_OFFSET_FROM_REFERENCE = 5                     # meters
    SAFE_DISTANCE_BETWEEN_CARS_ON_LANE_CHANGE = 23      # meters
    UNSAFE_DISTANCE_BETWEEN_CARS_ON_LANE_CHANGE = 10    # meters
    EGO_CAR_APPROACH_VELOCITY = 2   # m/s

    # Lane change distance coefficient applied to Tesla's velocity:
    #   5.8 for normal
    #   3 for imminent collision
    LANE_CHANGE_DISTANCE_K = 3      # Oleg: weird behaiour for various K
    
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, params=''):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self.timeout = timeout
        self._params = params
        
        self._map = CarlaDataProvider.get_map()
        
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._tesla_offset_from_reference = ChangeLane.TESLA_OFFSET_FROM_REFERENCE
        self._tesla_velocity = ChangeLane.TESLA_VELOCITY
        self._egocar_velocity = ChangeLane.TESLA_VELOCITY + ChangeLane.EGO_CAR_APPROACH_VELOCITY
        self._distance_between_cars_on_lane_change = ChangeLane.UNSAFE_DISTANCE_BETWEEN_CARS_ON_LANE_CHANGE

        if randomize:
            self._tesla_offset_from_reference = 1 + random.random() * 4
            self._tesla_velocity = random.randint(20, 30)
        
        super(ChangeLane, self).__init__("ChangeLane",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)


    def _initialize_actors(self, config):

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        # get Tesla's starting position
        tesla_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._tesla_offset_from_reference)
        self._tesla_transform = carla.Transform(
            carla.Location(tesla_waypoint.transform.location.x - 20,    # a bit further from the egocar
                           tesla_waypoint.transform.location.y - 3,     # right lane
                           tesla_waypoint.transform.location.z + 1),    # a bit above the road
            tesla_waypoint.transform.rotation)


    def _create_behavior(self):

        if self._params == '1':     # just driver
            self._egocar_velocity = ChangeLane.TESLA_VELOCITY
            ego_car_sequence = self._create_ego_car_behaviour_drive_straight()
            tesla_sequence = self._create_tesla_behaviour_drive_straight()
        elif self._params == '2':   # safe
            self._distance_between_cars_on_lane_change = ChangeLane.SAFE_DISTANCE_BETWEEN_CARS_ON_LANE_CHANGE
            ego_car_sequence = self._create_ego_car_behaviour_drive_straight()
            tesla_sequence = self._create_tesla_behaviour_change_lane()
        elif self._params == '3':   # unsafe
            ego_car_sequence = self._create_ego_car_behaviour_approach_tesla()
            tesla_sequence = self._create_tesla_behaviour_change_lane()
        else:
            return None
        
        root = py_trees.composites.Parallel("Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(ego_car_sequence)
        root.add_child(tesla_sequence)
        
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        #collision_criterion = CollisionTest(self.ego_vehicles[0])
        #criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

    def _create_ego_car_behaviour_approach_tesla(self):
        
        ego_car = self.ego_vehicles[0]
        tesla = self.other_actors[0]

        ego_car_sequence = py_trees.composites.Sequence("Ego")

        ego_car_sequence.add_child(DebugCheckPoint(ego_car, "start"))

        # Enable auto-pilot
        # Note that autopilot may overtake the control with some delay
        ego_car_autopilot = ChangeAutoPilot(ego_car, activate=True, name="EgoAutoPilot1")
        ego_car_sequence.add_child(ego_car_autopilot)
        ego_car_sequence.add_child(DebugCheckPoint(ego_car, "auto-pilot enabled"))

        drive_until_tesla_reached = py_trees.composites.Parallel("EgoAutonomousDriverUntilReachingTesla", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Follow the middle lane ... 
        p1 = py_trees.composites.Sequence("p1")
        lane_follower = WaypointFollower(ego_car, self._egocar_velocity, name="EgoLaneDriver1")
        p1.add_child(lane_follower)
        p1.add_child(DebugCheckPoint(ego_car, "lane-follower enabled"))
        drive_until_tesla_reached.add_child(p1)

        # ...until tesla is close enough
        p2 = py_trees.composites.Sequence("p2")
        distance_to_tesla = InTriggerDistanceToVehicle(tesla, ego_car, distance=self._distance_between_cars_on_lane_change, name="EgoToTeslaDistance1")
        p2.add_child(distance_to_tesla)
        p2.add_child(DebugCheckPoint(ego_car, "waiting for reaching Tesla..."))
        drive_until_tesla_reached.add_child(p2)
        
        ego_car_sequence.add_child(drive_until_tesla_reached)
        ego_car_sequence.add_child(DebugCheckPoint(ego_car, "Tesla reached"))

        # ...then disable autopilot
        ego_car_manual = ChangeAutoPilot(ego_car, activate=False, name="EgoManualPilot1")
        ego_car_sequence.add_child(ego_car_manual)
        ego_car_sequence.add_child(DebugCheckPoint(ego_car, "manual driving enabled"))

        # Contunue running as long as Tesla exists
        ego_car_sequence.add_child(Idle(name="EgoIndefiniteDriving1"))
        
        return ego_car_sequence
    
    def _create_ego_car_behaviour_drive_straight(self):
        
        ego_car = self.ego_vehicles[0]

        ego_car_sequence = py_trees.composites.Sequence("Ego")

        ego_car_sequence.add_child(DebugCheckPoint(ego_car, "start"))

        # Enable auto-pilot
        # Note that autopilot may overtake the control with some delay
        ego_car_autopilot = ChangeAutoPilot(ego_car, activate=True, name="EgoAutoPilot1")
        ego_car_sequence.add_child(ego_car_autopilot)
        ego_car_sequence.add_child(DebugCheckPoint(ego_car, "auto-pilot enabled"))

        # Follow the middle lane
        lane_follower = WaypointFollower(ego_car, self._egocar_velocity, name="EgoLaneDriver1")
        ego_car_sequence.add_child(lane_follower)
        ego_car_sequence.add_child(DebugCheckPoint(ego_car, "lane-follower enabled"))

        # Contunue running as long as Tesla exists
        ego_car_sequence.add_child(Idle(name="EgoIndefiniteDriving1"))
        
        return ego_car_sequence
    
    def _create_tesla_behaviour_change_lane(self):
        
        ego_car = self.ego_vehicles[0]
        tesla = self.other_actors[0]
        
        # Sequence of Tesla actions, behaviours, triggers
        
        tesla_sequence = py_trees.composites.Sequence("Tesla")
        
        # Show Tesla in the starting location
        tesla_visible = ActorTransformSetter(tesla, self._tesla_transform, name="TeslaPlacement")
        tesla_sequence.add_child(tesla_visible)

        # Move Tesla on the same lane ...
        driving_straight = py_trees.composites.Parallel("TeslaDrivingUntilEgoIsClose1", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        lane_follower = WaypointFollower(tesla, self._tesla_velocity, name="TeslaLaneDriver1")
        driving_straight.add_child(lane_follower)

        # ... for a certain distance before switching the left blinker on
        # distance_driven_before_lane_change = DriveDistance(tesla, ChangeLane.DISTANCE_TO_DRIVE_BEFORE_LANE_CHANGE, name="TeslaDrivingDisance")
        # driving_straight.add_child(distance_driven_before_lane_change)

        # ... until the ego car appears close enough
        distance_to_ego = InTriggerDistanceToVehicle(ego_car, tesla, distance=self._distance_between_cars_on_lane_change, name="TeslaToEgoDistance1")
        driving_straight.add_child(distance_to_ego)
        
        tesla_sequence.add_child(driving_straight)

        # Alternatively, this could be a time, not a distance
        # tesla_sequence.add_child(TimeOut(40))

        # Even more, the left blinker could be enabled when the egocar reaches Tesla:
        # driving_straight.add_child(InTriggerDistanceToVehicle(ego_car, tesla, 10))

        # Tesla switched left turn blinking on
        light_left_on = carla.VehicleLightState(carla.VehicleLightState.LeftBlinker)
        tesla_sequence.add_child(VehicleLightsControls(tesla, light_status = light_left_on, name="TeslaLightsLeftOn1"))
        
        # Tesla blinks the left turn for few second before changing the lane
        tesla_sequence.add_child(TimeOut(1.5))

        # Tesla changes 2 lanes at once
        # Note how the lane changing distance is dependent on the Tesla's speed
        lane_change = LaneChange(tesla,
                                 speed = self._tesla_velocity,
                                 distance_other_lane = self._tesla_velocity * 1.5,
                                 distance_lane_change = self._tesla_velocity * ChangeLane.LANE_CHANGE_DISTANCE_K,
                                 lane_changes = 2,
                                 name="TeslaLaneChange1")
        tesla_sequence.add_child(lane_change)

        # Tesla switches all lights off
        light_off = carla.VehicleLightState(carla.VehicleLightState.NONE)
        tesla_sequence.add_child(VehicleLightsControls(tesla, light_status=light_off, name="TeslaLaneDriver1"))

        # And continues driving on the current (new) lane
        driving_straight = py_trees.composites.Parallel("TeslaDrivingUntilFinished1", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        lane_follower = WaypointFollower(tesla, self._tesla_velocity, name="TeslaLaneDriver2")
        driving_straight.add_child(lane_follower)

        # The trial is over after 10 seconds
        driving_straight.add_child(TimeOut(5))

        tesla_sequence.add_child(driving_straight)
        
        return tesla_sequence
    
    def _create_tesla_behaviour_drive_straight(self):
        
        tesla = self.other_actors[0]
        
        # Sequence of Tesla actions, behaviours, triggers
        tesla_sequence = py_trees.composites.Sequence("Tesla")
        
        # Show Tesla in the starting location
        tesla_visible = ActorTransformSetter(tesla, self._tesla_transform, name="TeslaPlacement")
        tesla_sequence.add_child(tesla_visible)

        # Move Tesla on the same lane ...
        driving_straight = py_trees.composites.Parallel("TeslaDrivingUntilEgoIsClose1", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        lane_follower = WaypointFollower(tesla, self._tesla_velocity, name="TeslaLaneDriver1")
        driving_straight.add_child(lane_follower)

        # ... for a certain time
        driving_straight.add_child(TimeOut(60))
        
        tesla_sequence.add_child(driving_straight)

        return tesla_sequence