--- Contains the parts of the setup which have to be expressed using lua.
-- @author Stefan Huber
local M = {}

local helper = require('scenario/scenariohelper')
local logTag = 'BasicScenario'

--- The final waypoint where every AI heads to.
local finalWayPointName = 'waypoint3'

--- Ends the scenario with a fail message.
-- @param reason The reason why the scenario failed.
local function fail(reason)
    log('I', logTag, 'fail called')
    scenario_scenarios.finish({failed = reason})
end

--- Ends the scenario with a success message.
-- @param reason The success message.
local function success(reason)
    log('I', logTag, 'success called')
    scenario_scenarios.finish({msg = reason})
end

--- Executed when the scenario is restarted.
local function onScenarioRestarted()
    log('I', logTag, 'onScenarioRestarted called')
end

--- When is this function called?
local function onScenarioChange()
    log('I', logTag, 'onScenarioChange called')
end

--- Executed after the countdown starts.
local function onScenarioLoaded()
    log('I', logTag, 'onScenarioLoaded called')
end

--- Sets an AI path.
-- @param vehicleName The name of the AI vehicle in the prefabs.
-- @param waypoints A list of all waypoints the AI has to pass. The final waypoint finalWayPointName is getting added.
local function setupAIVehicle(vehicleName, waypoints)
    log('I', logTag, 'setupAIVehicle called')
    local arg = {
        vehicleName = vehicleName,
        waypoints = waypoints,
        driveInLane = "on",
        resetLearning = true
    }
    helper.setAiPath(arg)
    -- Other possibly useful arguments: aggression, routeSpeed, routeSpeedMode, lapCount
end

--- Executed when the scenario starts.
local function onRaceStart()
    log('I', logTag, 'onRaceStart called')
    
    -- Try 1
    -- setupAIVehicle("mainAI", {"waypoint0", "waypoint1", "waypoint2", finalWayPointName})
    
    -- Try 2
    -- helper.setAiMode('mainAI', 'manual')
    -- helper.setAiAggression('mainAI', 8.0)
    -- helper.setAiTarget("mainAI", "waypoint3")
    
    -- Try 3
    helper.setAiRoute("mainAI", {"waypoint0", "waypoint1", "waypoint2", finalWayPointName})
    
    helper.trackVehicle("mainAI", "The ai")
    helper.trackVehicle("fakePlayer", "The faked player")
end

--- Executed when reaching a waypoint.
-- Called after @see onRaceWaypointReached.
local function onRaceWaypoint(data, goal)
    log('I', logTag, 'onRaceWaypoint called')
    -- Possibly available information: data.waypointName, data.vehicleName
    
-- The following is copied from another found lua file:

-- this (WP 0) seems to indicate that the scenario is fully loaded and we're at the scenario start dialog	
--	if data.cur == 0 then
--		setup()
--	end	  
end

--- Executed when reaching a waypoint.
-- Called before @see onRaceWaypoint.
local function onRaceWaypointReached(data, goal)
    log('I', logTag, 'onRaceWaypointReached called')
end

--- When is this function called?
local function onRaceTick(raceTickTime)
    -- log('I', logTag, 'onRaceTick called')
end

local function onRaceResult()
    log('I', logTag, 'onRaceResult called')
    -- Determine when to call function fail or success.
    success("Currently the scenario is always successful.")
end

-- Attach the functions to be called at the approriate points in time to the module.
M.onRaceStart = onRaceStart
M.onScenarioLoaded = onScenarioLoaded
M.onRaceWaypoint = onRaceWaypoint
M.onRaceWaypointReached = onRaceWaypointReached
M.onRaceResult = onRaceResult
M.onRaceTick = onRaceTick
M.onScenarioChange = onScenarioChange

return M