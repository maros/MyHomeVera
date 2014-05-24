--
-- MyHome plugin
-- 
-- This program is free software; you can redistribute it and/or
-- modify it under the terms of the GNU General Public License
-- as published by the Free Software Foundation; either version 2
-- of the License, or (at your option) any later version.
-- 
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
-- 
-- You should have received a copy of the GNU General Public License
-- along with this program; if not, write to the Free Software
-- Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
--

local Device 					= nil
local SERVICE_ID 				= "urn:upnp-k1-com:serviceId:MyHome"

local STATUS					= {}
STATUS.CHANGING 				= -1
STATUS.HOME 					= 0
STATUS.AWAY 					= 10
STATUS.VACATION					= 20
STATUS.INTRUSION				= 90
STATUS.ALARM					= 100

local TIMER_BLINDS 				= 17 	-- timer for blinds to move fully up/down
local TIMER_AWAY				= 300 	-- timer for alarm to be armed after leaving
local TIMER_INTRUSION			= 60	-- timer for alarm to be disarmed after coming

local SENSOR_ALL				= { 20,35 }
local SENSOR_IMMEDIATE			= { 35 }

local SID_WINDOWCOVERING		= "urn:upnp-org:serviceId:WindowCovering1"
local SID_SWITCHPOWER			= "urn:upnp-org:serviceId:SwitchPower1"
local SID_DIMMING				= "urn:upnp-org:serviceId:Dimming1"
local SID_SECURITYSENSOR		= "urn:micasaverde-com:serviceId:SecuritySensor1"

local SENSOR_VARIABLE_TRIPPED	= "Tripped"
local SENSOR_VARIABLE_ARMED 	= "Armed"
local SENSOR_ACTION_ARM 		= "SetArmed"

-- 
-- FUNCTION
-- Init Device
--
function initialize(lul_device)
	Device = lul_device
	
	luup.log("[MyHome] Initialize MyHome");
	
	--local data = {}
  	--data = read_config(Device)
  	
  	for i in pairs(SENSOR_ALL) do
    	local id = SENSOR_ALL[i];
		luup.variable_watch("watch_callback", SID_SECURITYSENSOR, SENSOR_VARIABLE_ARMED, tonumber(id))
		luup.variable_watch("watch_callback", SID_SECURITYSENSOR, SENSOR_VARIABLE_TRIPPED, tonumber(id))
  	end
	
 	return true
end

function set_status_home()
	if (set_status(STATUS.HOME)) then
		set_if_changed(SERVICE_ID, "StatusLabel", "Somebody is home", Device)
		
		-- Cancel timer
		cancel_timer()
		
		-- Disarm sensors
	  	for i in pairs(SENSOR_ALL) do
	    	local id = SENSOR_ALL[i];
			luup.call_action(SID_SecuritySensor1, SENSOR_ACTION_ARM, {newArmedValue = 0}, tonumber(id))
	  	end
	  	
	  	-- TODO: turn up thermostat
  	end
end

function set_status_away()
	if (set_status(STATUS.AWAY)) then
		set_if_changed(SERVICE_ID, "StatusLabel", "Nobody is home", Device)
		init_away();
	end
end

function set_status_vacation()
	if (set_status(STATUS.VACATION)) then
		set_if_changed(SERVICE_ID, "StatusLabel", "Gone on vacation", Device)
		init_away();
	end
end

function init_away()
	-- TODO: Turn off all lights
	start_timer(TIMER_AWAY,'run_away')
end

function run_intrusion()
	luup.log("[MyHome] Intrusion was detected");
	start_timer(TIMER_INTRUSION,'run_alarm')
end

function run_alarm()
	luup.log("[MyHome] ALARM!!!")
	-- TODO: Send messages
	-- TODO: Turn on lights
	-- TODO: Run sirens
	-- TODO: Open all blinds
	-- TODO: Play sound
end

function run_away()
	luup.log("[MyHome] Perform away")
  	for i in pairs(SENSOR_ALL) do
    	local id = SENSOR_ALL[i];
		luup.call_action(SID_SecuritySensor1, SENSOR_ACTION_ARM, {newArmedValue = 1}, tonumber(id))
  	end
  	
	-- TODO: Start away logic (blinds, random lights, temperature, windows)
end

function set_status(new_status)
	local current_status = luup.variable_get(SERVICE_ID, "Status", Device)
  	if (tonumber(new_status) ~= tonumber(current_status)) then
  		luup.log("[MyHome] Set status to " .. new_status)
   		luup.variable_set(SERVICE_ID, "Status", new_status, Device)
   		return true
   	end
   	return false
end

function watch_callback(lul_device, lul_service, lul_variable, lul_value_old, lul_value_new)
	luup.log("[MyHome] Watched variable changed: " .. lul_device .. " " .. lul_service .. " " .. lul_variable .. " from " .. lul_value_old .. " to " .. lul_value_new)
	
	local data = {}
    data = read_config(Device)
	
	if (tonumber(data.status) >= 0 and tonumber(data.status) < STATUS.INTRUSION) then
	  	luup.log("[MyHome] Callback data.status=" .. data.status , 100)
	 	check_tripped()
	end
end

function check_tripped()
  	local status = tonumber(read_or_init(Device,SERVICE_ID, "Status",0))
  	local armed_tripped = 0
  
  	if (status > 0 and status < STATUS.INTRUSION) then
    	local data = {}
    	data = read_config(Device)
  
    	-- Check Sensors Tripped Status
		for i in pairs(SENSOR_ALL) do
	  		local id = SENSOR_ALL[i];
	  		local sensor_tripped = luup.variable_get(SID_SECURITYSENSOR,SENSOR_VARIABLE_TRIPPED, tonumber(id))
		  	if sensor_tripped == "1" then
		    	local sensor_armed = luup.variable_get(SID_SECURITYSENSOR,SENSOR_VARIABLE_ARMED, tonumber(id))
		    	if sensor_armed == "1" then
		    		-- Check Immediate Alarm 
		    		for j in pairs(SENSOR_IMMEDIATE) do
		    			local compare = SENSOR_IMMEDIATE[i];
		    			if compare == id then
		    				run_alarm(Device, lul_settings)
		    				return
		    			end
		    		end
			  		armed_tripped = armed_tripped + 1
				end
		  	end
		end
		if armed_tripped > 0 then
		  	run_intrusion(Device, lul_settings)
		end
  	end
end

-- 
-- HELPER
--

function read_config(lul_device)
	local data = {}
  	data.status			= read_or_init(lul_device,SERVICE_ID, "Status", 0)
  	data.statuslabel	= read_or_init(lul_device,SERVICE_ID, "StatusLabel", "Disarmed")
	
	return data
end

function read_or_init(lul_device, devicetype, name, default)
	local value = luup.variable_get(devicetype,name, lul_device)
 	if (value == nil) then
   		value = default
   		luup.variable_set(devicetype,name,value,lul_device)
 	end
 	return value
end

function set_if_changed(service_id, name, value, device)
    local curValue = luup.variable_get(service_id, name, device)
    if ((value ~= curValue) or (curValue == nil)) then
        luup.variable_set(service_id, name, value, device)
        return true
    else
        return false
    end
end

-- 
-- TIMER
-- Various actors functions
--

function tick()
  	-- Timer may have been cancelled or forced.
  	-- If so, break out.
  	local counting = read_or_init(Device,SERVICE_ID, "Counting", 0)
  	if (counting == "0") then
    	return false
  	end
  	if (update_remaining()) then
    	-- Timer is still underway.
    	return luup.call_delay("tick", 1, "") == 0
  	end
  	
  	-- Timer has completed.
  	set_if_changed(SERVICE_ID, "DueTimestamp", "", Device)
  	set_if_changed(SERVICE_ID, "Remaining", 0, Device)
  	set_if_changed(SERVICE_ID, "Counting", 0, Device)
  
  	local timer_action = read_or_init(Device,SERVICE_ID, "TimerAction", "")
  	if timer_action == "run_away" then
    	run_away()
  	elseif timer_action == "run_alarm" then
    	run_alarm()
  	end
  
  	return true
end

function start_timer(duration,action)
  	local counting = read_or_init(Device,SERVICE_ID, "Counting", 0)
  	if (counting == "1") then
    	return false
  	end
  	set_if_changed(SERVICE_ID, "TimerDuration", duration, Device)
  	return start_timer_always(action)
end

function start_timer_always(action)
	set_if_changed(SERVICE_ID, "TimerAction", action, Device)
	
  	local counting = read_or_init(Device,SERVICE_ID, "Counting", 0)
  	local duration = read_or_init(Device,SERVICE_ID, "TimerDuration", 30)
  	local due_timestamp = os.time() + duration
  	set_if_changed(SERVICE_ID, "DueTimestamp", due_timestamp, Device)
  	update_remaining()
  	set_if_changed(SERVICE_ID, "Counting", 1, Device)
  	return luup.call_delay("tick", 1, "") == 0
end

function update_remaining()
  	local due_timestamp = read_or_init(Device,SERVICE_ID, "DueTimestamp", "")
  	local remaining = tonumber(due_timestamp) - os.time()
  	if (remaining < 0) then remaining = 0 end
  	set_if_changed(SERVICE_ID, "Remaining", remaining, Device)
  	return remaining > 0
end

function cancel_timer()
  	local counting = read_or_init(Device,SERVICE_ID, "Counting", 0)
  	if (counting == "0") then
    	return false
  	end
  	set_if_changed(SERVICE_ID, "Counting", 0, Device)
  	set_if_changed(SERVICE_ID, "DueTimestamp", "", Device)
  	set_if_changed(SERVICE_ID, "Remaining", 0, Device)
  	return true
end

-- 
-- ACTORS
-- Various actors functions
--

_G["blind_stop"] = function(device) 
    luup.log("[MyHome] Stoping blind " .. device)
    luup.call_action(SID_BLINDS, "Stop", {}, tonumber(device))
end

_G["light_turn_off"] = function(device)
	log("[MyHome] Turn light off " .. device)
	luup.call_action(SID_POWER, "SetTarget", { newTargetValue = 0 }, tonumber(device))
end

_G["blind_partial"] = function (device,percentage)
	device = tonumber(device)
	percentage = tonumber(percentage)

    luup.log("[MyHome] Moving blind " .. device .. " to " .. percentage .. "%")
    
    local blind_position = luup.variable_get(SID_DIMMING,"LoadLevelStatus",device);
    local blind_time = math.floor(( blind_position - percentage ) / 100 * TIMER_BLINDS);
	percentage = math.floor(( blind_time / TIMER_BLINDS) * 100)

    luup.call_action(SID_DIMMING, "SetLoadLevelTarget", { newLoadlevelTarget = percentage }, device)

    blind_time = math.floor(blind_time)
    if blind_time < 0 then
		blind_time = blind_time * -1
        luup.call_action(SID_BLINDS, "Up", {}, device)
    elseif blind_time > 0 then
        luup.call_action(SID_BLINDS, "Down", {}, device)
    end

	luup.call_delay("blind_stop", blind_time, device)
end


--function light_random(devices)
--    local presence_status = luup.variable_get("urn:schemas-upnp-org:device:VSwitch:1","Status",presence_device)
--    local time = os.date('*t');
--    
--	if presence_status == 0 && luup.is_night() && time.hour > 17 && time.hour < 23 then
--		device_index 	= math.ceil(math.random(0,table.getn(devices)))
--		device_on		= math.floor(math.random(10))
--		device_time 	= math.floor(math.random(60,900))
--		if (device_on == 1) then
--			local device = devices[device_index]
--			luup.call_action(SID_POWER, "SetTarget", { newTargetValue = 1 }, device)
--			luup.call_delay("light_turn_off", device_time, device)
--		end
--	end
--end
