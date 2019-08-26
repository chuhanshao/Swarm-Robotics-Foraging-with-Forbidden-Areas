
-- robotics states
WALK = "WALK"                         -- Initial state, just randomly walk
AVOID = "AVOID"                       -- Avoid state, to avoid obstacles.
STOP = "STOP"                         -- Stop state, stop to indicate the forbidden areas and the nest
FORAGE = "FORAGE"                     -- Forage state, find the food
BACK = "BACK"                         -- Back state, take the food to the nest

-- global variables
current_state = WALK                  -- the current state of the robot
is_obstacle_sensed = false            -- is the robot sensing an obstacle?
is_black_sensed = false               -- is the robot sensing black? which means the food source
is_white_sensed = false               -- is the robot sensing white? which means the nest
is_light_sensed = false               -- is the robot sensing the light? whihch indicates the direction of the food
nest_found = false                    -- did the robots find the nest?
carrying_food = false                 -- is the robot carring food?
mark_of_first_forb = false            -- is the first forbidden area sensed by otehr robots?
mark_of_second_forb = false           -- is the second forbidden area sensed by otehr robots?
is_first_forbidden_sensed = false     -- is the robot in the first forbidden area?
is_second_forbidden_sensed = false    -- is the robot in the second forbidden area?

WHEEL_SPEED = 15                      -- the wheel speed is required to be not greater than 15
TARGET_DIST = 300                     -- the target distance between robots, in cm
EPSILON = 50                          -- a coefficient to increase the force of the repulsion/attraction function

-- variables for obstacle avoidance
MAX_TURN_STEPS = 5
current_turn_steps = 0

function init()
   -- enable the cameras of the robots
   robot.colored_blob_omnidirectional_camera.enable()
end

-- This function is executed at each time step
function step()
  ProcessProx()                       -- check for obstacles
  DetectLight()                       -- check for light (food source)
  ProcessBlackGround()                -- check for black ground
  ProcessWhiteGround()                -- check for white ground
  CheckNestFound()                    -- check if the nest is found
  DetectFirstForbidden()              -- check if a robot is in the first forbidden area
  DetectSecondForbidden()             -- check if a robot is in the second forbidden area
  CheckFirstForbiddenFound()          -- check if other robot is in the first forbidden area
  CheckSecondForbiddenFound()         -- check if other robot is in the second forbidden area

  ---------- State WALK ----------
  -- explore the environment
  -- state changes once the transition condition is met
  if current_state == WALK then
    robot.wheels.set_velocity(15, 15)
    if is_white_sensed and nest_found == false then
      -- sensed the white area for the first time
      -- the nest is found
      current_state = STOP
      nest_found = true
    elseif is_first_forbidden_sensed and mark_of_first_forb == false then
      -- sensed the first forbidden area and no other robots nearby sensed the area
      -- lose the food and stop
      current_state = STOP
      carrying_food = false
    elseif is_second_forbidden_sensed and mark_of_second_forb == false then
      -- sensed the second forbidden area and no other robots nearby sensed the area
      -- lose the food and stop
      current_state = STOP
      carrying_food = false
    elseif is_obstacle_sensed then
      -- sensed the obstacle
      current_state = AVOID
      -- set the number of steps to turn
      current_turn_steps = robot.random.uniform_int(2,MAX_TURN_STEPS)
    elseif is_light_sensed and carrying_food == false and nest_found then
      -- if there is light (in fact there is alway a light)
      -- not carry food and the nest is found
      -- the robot go towards the light to get food
      current_state = FORAGE
    elseif carrying_food and nest_found then
      -- after carrying food and the nest is found
      -- the rotbot should go back to the nest
      current_state = BACK
    end

  ---------- State AVOID ----------
  -- a robot turns around by setting a random number of steps to avoid collisions
  elseif current_state == AVOID then
		robot.wheels.set_velocity(15,-15)
		current_turn_steps = current_turn_steps - 1
		-- if turning steps have been completed
    -- change its state back to "WALK"
		if current_turn_steps <= 0 then
			current_state = WALK
		end

  ---------- State STOP ----------
  -- a robot stops when it:
  -- 1. find the nest, and sets its leds to BLUE = 255
  -- 2. find the first forbidden area, and sets its leds to RED = 200
  -- 3. find the second forbidden area, and sets its leds to GREEN = 200
  elseif current_state == STOP then
    if is_white_sensed then
      -- set leds colour to blue to attract other robot back to the nest
      robot.leds.set_all_colors("0,0,255,0")
    elseif is_first_forbidden_sensed then
      -- the robot go into the first forbidden area and no other robots in the area nearby
      robot.leds.set_all_colors("200, 0, 0,0")
      carrying_food = false
    elseif is_second_forbidden_sensed then
      -- the robot go into the second forbidden area and no other robots in the area nearby
      robot.leds.set_all_colors("0,200,0,0")
      carrying_food = false
    end
    -- set the speed of wheels to 0 to stop the robot
    robot.wheels.set_velocity(0,0)

    -- in case the robot move out of the areas
    -- the robot is free to WALK
    if (not is_white_sensed) and (not is_first_forbidden_sensed) and (not is_second_forbidden_sensed) then
      current_state = WALK
    end

  ---------- State FORAGE ----------
  -- a robot chases the light and get the food
  elseif current_state == FORAGE then
    -- avoid obstacles
    if is_obstacle_sensed then
      current_state = AVOID
    end
    -- detect the light and go towards the light
    direction = DirectionToLight()
    speeds = ComputeSpeedFromAngle(direction)
    robot.wheels.set_velocity(speeds[1],speeds[2])
    -- if the balck area is sensed
    -- the robot has carried food and the state is back to WALK
    ProcessBlackGround()
    if is_black_sensed then
      carrying_food = true
      current_state = WALK
    end

  ---------- State BACK ----------
  -- the carrying food robots go back to the nest
  elseif current_state == BACK then
    -- if the robot goes into the forbidden areas
    -- it loses the food and its state is back to WALK
    DetectFirstForbidden()
    DetectSecondForbidden()
    if is_first_forbidden_sensed or is_second_forbidden_sensed then
      carrying_food = false
      current_state = WALK
    end

    -- detect the RED=200, GREEN=200 and BLUE=255 light
    DetectRedLight()
    DetectGreenLight()
    DetectBlueLights()
    -- if red or green light is sensed
    -- keep away from it
    if sensed_red then
      KeepAway()
    elseif sensed_green then
      KeepAway()
    -- if the blue light is sensed
    elseif sensed_blue then
      -- get the direction of the blue light
      direction = BlueDirection()
      speeds = ComputeSpeedFromAngle(direction)
      -- go towards the blue light
      robot.wheels.set_velocity(speeds[1],speeds[2])
      -- set its leds to BLUE=245
      robot.leds.set_all_colors("0,0,245,0")
    -- if the BLUE=245 light is sensed
    elseif (not sensed_blue) and sensed_blue_245 then
      -- get the direction of the BLUE=245 light
      direction = Blue245Direction()
      speeds = ComputeSpeedFromAngle(direction)
      -- go towards the BLUE=245 light
      robot.wheels.set_velocity(speeds[1],speeds[2])
      -- set its leds to BLUE=235
      robot.leds.set_all_colors("0,0,235,0")
    -- if the BLUE=235 light is sensed
    -- do the same operations as above
    elseif (not sensed_blue) and (not sensed_blue_245) and sensed_blue_235 then
      direction = Blue235Direction()
      speeds = ComputeSpeedFromAngle(direction)
      robot.wheels.set_velocity(speeds[1],speeds[2])
      robot.leds.set_all_colors("0,0,225,0")
    elseif (not sensed_blue) and (not sensed_blue_245) and (not sensed_blue_235) and sensed_blue_225 then
      direction = Blue225Direction()
      speeds = ComputeSpeedFromAngle(direction)
      robot.wheels.set_velocity(speeds[1],speeds[2])
      robot.leds.set_all_colors("0,0,215,0")
    elseif (not sensed_blue) and (not sensed_blue_245) and (not sensed_blue_235) and (not sensed_blue_225) and sensed_blue_215 then
      direction = Blue215Direction()
      speeds = ComputeSpeedFromAngle(direction)
      robot.wheels.set_velocity(speeds[1],speeds[2])
      robot.leds.set_all_colors("0,0,205,0")
    elseif (not sensed_blue) and (not sensed_blue_245) and (not sensed_blue_235) and (not sensed_blue_225) and (not sensed_blue_215) and sensed_blue_205 then
      direction = Blue205Direction()
      speeds = ComputeSpeedFromAngle(direction)
      robot.wheels.set_velocity(speeds[1],speeds[2])
      robot.leds.set_all_colors("0,0,195,0")
    elseif (not sensed_blue) and (not sensed_blue_245) and (not sensed_blue_235) and (not sensed_blue_225) and (not sensed_blue_215) and (not sensed_blue_205) and sensed_blue_195 then
      direction = Blue195Direction()
      speeds = ComputeSpeedFromAngle(direction)
      robot.wheels.set_velocity(speeds[1],speeds[2])
      robot.leds.set_all_colors("0,0,185,0")
    elseif (not sensed_blue) and (not sensed_blue_245) and (not sensed_blue_235) and (not sensed_blue_225) and (not sensed_blue_215) and (not sensed_blue_205) and (not sensed_blue_195) and sensed_blue_185 then
      direction = Blue185Direction()
      speeds = ComputeSpeedFromAngle(direction)
      robot.wheels.set_velocity(speeds[1],speeds[2])
      robot.leds.set_all_colors("0,0,175,0")
    elseif (not sensed_blue) and (not sensed_blue_245) and (not sensed_blue_235) and (not sensed_blue_225) and (not sensed_blue_215) and (not sensed_blue_205) and (not sensed_blue_195) and (not sensed_blue_185) and sensed_blue_175 then
      direction = Blue175Direction()
      speeds = ComputeSpeedFromAngle(direction)
      robot.wheels.set_velocity(speeds[1],speeds[2])
      robot.leds.set_all_colors("0,0,165,0")
    elseif (not sensed_blue) and (not sensed_blue_245) and (not sensed_blue_235) and (not sensed_blue_225) and (not sensed_blue_215) and (not sensed_blue_205) and (not sensed_blue_195) and (not sensed_blue_185) and (not sensed_blue_175) and sensed_blue_165 then
      direction = Blue165Direction()
      speeds = ComputeSpeedFromAngle(direction)
      robot.wheels.set_velocity(speeds[1],speeds[2])
      robot.leds.set_all_colors("0,0,155,0")
    -- if no blue light is sensed
    -- set the leds to BLUE=200 to indicate the nest is found
    -- (it is reasonable due to the robot is BACK when the nest is found)
    else
      robot.leds.set_all_colors("0,0,200,0")
      robot.wheels.set_velocity(15,15)
    end

    -- avoid obstacles
    if is_obstacle_sensed then
      current_state = AVOID
    end

    -- reset the variables
    through = false
    sensed_red = false
    sensed_green = false
    sensed_blue = false
    sensed_blue_245 = false
    sensed_blue_235 = false
    sensed_blue_225 = false
    sensed_blue_215 = false
    sensed_blue_205 = false
    sensed_blue_195 = false
    sensed_blue_185 = false
    sensed_blue_175 = false
    sensed_blue_165 = false
    robot.leds.set_all_colors("0,0,200,0")

    -- if robots enter the white area (nest)
    -- go back to WALK and its leds is set to BLUE=200
    if is_white_sensed then
      carrying_food = false
      current_state = WALK
      robot.leds.set_all_colors("0,0,200,0")
    end
  end
end

-- fucntion used to copy two tables
function table.copy(t)
	local t2 = {}
	for k,v in pairs(t) do
		t2[k] = v
	end
	return t2
end

-- Detect the light above the food.
function DetectLight()
  is_light_sensed = false
  sort_light = table.copy(robot.light)
  table.sort(sort_light, function(a,b) return a.value>b.value end)
  if sort_light[1].value > 0 then
    is_light_sensed = true
  end
end

-- Calculate the direction to the light
function DirectionToLight()
  direction = 0
  direction_x = 0
  direction_y = 0
  sort_light = table.copy(robot.light)
  table.sort(sort_light, function(a,b) return a.value>b.value end)
  direction_x = direction_x + math.cos(sort_light[1].angle)
  direction_y = direction_y + math.sin(sort_light[1].angle)
  direction = math.atan2(direction_y,direction_x)
  return direction
end

-- sense obstacles by sorting the proximity sensor values and checking the biggest.
-- if it is bigger than a threshold, then there is an obstacle.
-- ignore sensors on the back of the robot (abs(angle) < pi/2)
function ProcessProx()
	is_obstacle_sensed = false
	sort_prox = table.copy(robot.proximity)
	table.sort(sort_prox, function(a,b) return a.value > b.value end)
	if sort_prox[1].value > 0.05 and math.abs(sort_prox[1].angle) < math.pi/2
		then is_obstacle_sensed = true
	end
end

-- this function computes the necessary wheel speed to go in the direction of the desired angle.
function ComputeSpeedFromAngle(angle)
	dotProduct = 0.0;
	KProp = 20;
	wheelsDistance = 0.14;
	-- if the target angle is behind the robot, we just rotate, no forward motion
	if angle > math.pi/2 or angle < -math.pi/2 then
		dotProduct = 0.0;
	else
		-- else, we compute the projection of the forward motion vector with the desired angle
		forwardVector = {math.cos(0), math.sin(0)}
		targetVector = {math.cos(angle), math.sin(angle)}
		dotProduct = forwardVector[1]*targetVector[1]+forwardVector[2]*targetVector[2]
	end
	-- the angular velocity component is the desired angle scaled linearly
	angularVelocity = KProp * angle;
	-- the final wheel speeds are compute combining the forward and angular velocities, with different signs for the left and right wheel.
	speeds = {dotProduct * WHEEL_SPEED - angularVelocity * wheelsDistance, dotProduct * WHEEL_SPEED + angularVelocity * wheelsDistance}
	return speeds
end

-- Sense the black/white spot.
-- ff all sensors are sensing black/white, is_sensed_black/white = true
function ProcessBlackGround()
	is_black_sensed = false
	sort_ground = table.copy(robot.motor_ground)
	table.sort(sort_ground, function(a,b) return a.value > b.value end)
	-- make sure that the robot enter the black zone entirely
	if sort_ground[1].value == 0 then
		is_black_sensed = true
	end
end

function ProcessWhiteGround()
	is_white_sensed = false
	sort_ground = table.copy(robot.motor_ground)
	table.sort(sort_ground, function(a,b) return a.value < b.value end)
	-- make sure that the robot enter the white zone entirely
	if sort_ground[1].value == 1 then
		is_white_sensed = true
	end
end

-- check if the nest is found. if the robot's light is blue, then the nest is found
-- this congifguration is because of the contradition for the light for food, which is yellow light
-- yellow light = red (255) light + green (255) light, which mislead the robot
-- if there is blue (255) light sensed, the rest robots set the leds to BLUE=200 to show that the nest is found
function CheckNestFound()
	nest_found = false
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 255 then
				nest_found = true
        robot.leds.set_all_colors("0,0,200,0")
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 200 then
        nest_found = true
        robot.leds.set_all_colors("0,0,200,0")
			end
		end
	end
end

-- check if the first forbidden area is found
-- if the red light 200 is sensed
-- and the distance is lower than 70 cm
-- this area is marked by other robots
function CheckFirstForbiddenFound()
	mark_of_first_forb = false
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.red == 200 and robot.colored_blob_omnidirectional_camera[i].distance < 70 then
				mark_of_first_forb = true
			end
		end
	end
end

-- check if the second forbidden area is found
-- if the green light 200 is sensed
-- and the distance is lower than 70 cm
-- this area is marked by other robots
function CheckSecondForbiddenFound()
	mark_of_second_forb = false
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.green == 200 and robot.colored_blob_omnidirectional_camera[i].distance < 70 then
				mark_of_second_forb = true
			end
		end
	end
end

-- check if the area is the first forbidden area
-- if the colour of the ground is 0.4
-- then a robot senses thr first forbidden area
function DetectFirstForbidden()
  is_first_forbidden_sensed = false
	sort_ground = table.copy(robot.motor_ground)
	table.sort(sort_ground, function(a,b) return a.value > b.value end)
	-- make sure that the robot enter the white zone entirely
	if sort_ground[1].value == 0.4 then
		is_first_forbidden_sensed = true
	end
end

-- check if the area is the second forbidden area
-- if the colour of the ground is 0.2
-- then a robot senses thr second forbidden area
function DetectSecondForbidden()
  is_second_forbidden_sensed = false
	sort_ground = table.copy(robot.motor_ground)
	table.sort(sort_ground, function(a,b) return a.value > b.value end)
	-- make sure that the robot enter the white zone entirely
	if sort_ground[1].value == 0.2 then
		is_second_forbidden_sensed = true
	end
end

-- check if the robot detect the family of blue lights
function DetectBlueLights()
  if (#robot.colored_blob_omnidirectional_camera > 0) then
    for i=1, #robot.colored_blob_omnidirectional_camera do
      if robot.colored_blob_omnidirectional_camera[i].color.blue == 255 then
        sensed_blue = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 245 then
        sensed_blue_245 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 235 then
        sensed_blue_235 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 225 then
        sensed_blue_225 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 215 then
        sensed_blue_215 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 205 then
        sensed_blue_205 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 195 then
        sensed_blue_195 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 185 then
        sensed_blue_185 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 175 then
        sensed_blue_175 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 165 then
        sensed_blue_165 = true
      elseif robot.colored_blob_omnidirectional_camera[i].color.blue == 155 then
        sensed_blue_155 = true
      end
    end
  end
end

-- get the direction towards the light with BLUE=255
function BlueDirection()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 255 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=245
function Blue245Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 245 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=235
function Blue235Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 235 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=225
function Blue225Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 225 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=215
function Blue215Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 215 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=205
function Blue205Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 205 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=195
function Blue195Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 195 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=185
function Blue185Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 185 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=175
function Blue175Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 175 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- get the direction towards the light with BLUE=165
function Blue165Direction()
	direction = 0
	direction_x = 0
	direction_y = 0
	if (#robot.colored_blob_omnidirectional_camera > 0) then
		for i=1,#robot.colored_blob_omnidirectional_camera do
			if robot.colored_blob_omnidirectional_camera[i].color.blue == 165 then
				direction_x = direction_x + math.cos(robot.colored_blob_omnidirectional_camera[i].angle)
				direction_y = direction_y + math.sin(robot.colored_blob_omnidirectional_camera[i].angle)
			end
		end
	end
	direction = math.atan2(direction_y,direction_x)
	return direction
end

-- detect if there is a red light located in 60 cm (RED=200)
function DetectRedLight()
  if (#robot.colored_blob_omnidirectional_camera > 0) then
    for i=1, #robot.colored_blob_omnidirectional_camera do
      if robot.colored_blob_omnidirectional_camera[i].color.red == 200 and robot.colored_blob_omnidirectional_camera[i].distance < 60 then
        sensed_red = true
      end
    end
  end
end

-- detect if there is a green light located in 60 cm (GREEN=200)
function DetectGreenLight()
  if (#robot.colored_blob_omnidirectional_camera > 0) then
    for i=1, #robot.colored_blob_omnidirectional_camera do
      if robot.colored_blob_omnidirectional_camera[i].color.green == 200 and robot.colored_blob_omnidirectional_camera[i].distance < 60 then
        sensed_green = true
      end
    end
  end
end

-- keep a distance between two robots
function KeepAway()
    robot.range_and_bearing.set_data(1,1)           -- first we send something, to make sure the other robots see us
    target_angle = ProcessRAB_LJ()                  -- then we compute the angle to follow, using the other robots as input, see function code for details
    speeds = ComputeSpeedFromAngle(target_angle)    -- we now compute the wheel speed necessary to go in the direction of the target angle
    robot.wheels.set_velocity(speeds[1],speeds[2])  -- actuate wheels to move
	  robot.range_and_bearing.clear_data()            -- forget about all received messages for next step
end

-- In this function, we take all distances of the other robots and apply the lennard-jones potential.
-- We then sum all these vectors to obtain the final angle to follow in order to go to the place with the minimal potential
function ProcessRAB_LJ()
  sum_vector = {0,0}
   for i = 1,#robot.range_and_bearing do                                                                  -- for each robot seen
      lj_value = ComputeLennardJones(robot.range_and_bearing[i].range)                                    -- compute the lennard-jones value
      sum_vector[1] = sum_vector[1] + math.cos(robot.range_and_bearing[i].horizontal_bearing)*lj_value    -- sum the x components of the vectors
      sum_vector[2] = sum_vector[2] + math.sin(robot.range_and_bearing[i].horizontal_bearing)*lj_value    -- sum the y components of the vectors
   end
   desired_angle = math.atan2(sum_vector[2],sum_vector[1])                                                -- compute the angle from the vector
   --log( "angle: "..desired_angle.." length^2: "..(math.pow(sum_vector[1],2)+math.pow(sum_vector[2],2)) )
   return desired_angle
end

-- This function take the distance and compute the lennard-jones potential.
function ComputeLennardJones(distance)
   return -(4*EPSILON/distance * (math.pow(TARGET_DIST/distance,4) - math.pow(TARGET_DIST/distance,2)));
end

function reset()
  current_state = WALK
  is_obstacle_sensed = false
  is_black_sensed = false
  is_light_sensed = false
  is_white_sensed = false
  source_found = false
  nest_found = false
  carrying_food = false
  current_turn_steps = 0
  current_fwd_steps = 0
  mark_of_first_forb = false
  mark_of_second_forb = false
  is_first_forbidden_sensed = false
  is_second_forbidden_sensed = false
end

function destroy()
end
