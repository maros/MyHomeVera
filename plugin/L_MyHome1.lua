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

function Set(list)
	local set = {}
  	for _, l in ipairs(list) do set[l] = true end
  	return set
end

local SELF	 					= nil
local SERVICE_ID 				= "urn:upnp-k1-com:serviceId:MyHome1"

-- luup.call_action("urn:upnp-k1-com:serviceId:MyHome1", "lights_set", { taget = 1}, 62)
-- luup.call_action("urn:upnp-k1-com:serviceId:MyHome1", "lights_random", 62)
--

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
local TIMER_ALARM				= 600	-- timer for alarm

local WIND_SPEED_LIMIT			= 5
local BLINDS_PARTIAL			= 20

local DEVICES					= {}
DEVICES.SEC_SENSOR				= { 20,35,63 }
DEVICES.SEC_SENSOR_IMMEDIATE	= { 35 }
DEVICES.TEMP_SENSOR_INSIDE		= 64
DEVICES.TEMP_SENSOR_OUTSIDE		= 68
DEVICES.WEATHER					= 67
DEVICES.BLINDS_SOUTH			= { 21,22,23,24,28,29 }
DEVICES.BLINDS_NORTH			= { 32,33,26,30,25,31 }
-- DEVICES.WINDOWS					= { 111,111 }
-- DEVICES.RAIN_SENSOR				= 111

local SID_WINDOWCOVERING		= "urn:upnp-org:serviceId:WindowCovering1"
local SID_SWITCHPOWER			= "urn:upnp-org:serviceId:SwitchPower1"
local SID_DIMMING				= "urn:upnp-org:serviceId:Dimming1"
local SID_TEMPERATURE			= "urn:upnp-org:serviceId:TemperatureSensor1"
local SID_SECURITYSENSOR		= "urn:micasaverde-com:serviceId:SecuritySensor1"
local SID_WEATHER				= "urn:upnp-micasaverde-com:serviceId:Weather1"

local DID_LIGHT					= "urn:schemas-upnp-org:device:BinaryLight:1"
local DID_DOORSENSOR			= "urn:schemas-micasaverde-com:device:DoorSensor:1"
local DID_WINDOWCOVERING		= "urn:schemas-micasaverde-com:device:WindowCovering:1"


local SENSOR_VARIABLE_TRIPPED	= "Tripped"
local SENSOR_VARIABLE_ARMED 	= "Armed"
local SENSOR_ACTION_ARM 		= "SetArmed"

local TEMPERATURE				= {}
TEMPERATURE.MAX					= 25
TEMPERATURE.VACATION			= 15
TEMPERATURE.LOW					= 18
TEMPERATURE.DEFAULT				= 20

local WEATHER					= {}
WEATHER.POOR					= Set { "chanceflurries", "chancerain", "chancesleet", "chancesnow", "chancetstorms", "flurries","sleet","rain","snow","tstorms","unknown" }
WEATHER.NEUTRAL					= Set { "cloudy", "fog", "mostlycloudy", "partlycloudy", "partlysunny"  }
WEATHER.FAIR					= Set { "clear", "hazy", "mostlysunny" }

-- 
-- FUNCTION
-- Init Device
--
function initialize(lul_device)
	SELF = lul_device
	
	luup.log("[MyHome] Initialize MyHome")
	
	--local data = {}
  	--data = read_config(SELF)
  	
  	for index,device in pairs(DEVICES.SEC_SENSOR) do
		luup.variable_watch("watch_callback", SID_SECURITYSENSOR, SENSOR_VARIABLE_ARMED, device)
		luup.variable_watch("watch_callback", SID_SECURITYSENSOR, SENSOR_VARIABLE_TRIPPED, device)
  	end
	
	-- TODO: Close windows on rain
	-- luup.variable_watch("windows_close", SID_SECURITYSENSOR, SENSOR_VARIABLE_TRIPPED, DEVICES.RAIN_SENSOR)
	
 	return true
end

-- Manually set status = Home
function set_status_home()
	if (set_status(STATUS.HOME)) then
		set_if_changed(SERVICE_ID, "StatusLabel", "Somebody is home", SELF)
		
		-- Cancel timer
		cancel_timer()
		
		-- Disarm sensors immediately
	  	for index,device in pairs(DEVICES.SEC_SENSOR) do
			luup.call_action(SID_SecuritySensor1, SENSOR_ACTION_ARM, {newArmedValue = 0}, device)
	  	end
	  	
	  	--luup.variable_set(SERVICE_ID,"BlindStatusAuto",0,SELF)
	  	--luup.variable_set(SERVICE_ID,"WindowsStatusAuto",0,SELF)
	  	
	  	-- Run initial automator
	  	run_automator()
  	end
end

-- Manually set status = Away
function set_status_away()
	if (set_status(STATUS.AWAY)) then
		set_if_changed(SERVICE_ID, "StatusLabel", "Nobody is home", SELF)
		
		-- Set thermostats to low setting
		thermostats_set(TEMPERATURE.LOW)
		
		-- Turn all lights off
		lights_set(0)
		
		-- TODO: Disarm remote alarm manager
		
		start_timer(TIMER_AWAY,'run_away')
	end
end

-- Manually set status = Vacation
function set_status_vacation()
	if (set_status(STATUS.VACATION)) then
		set_if_changed(SERVICE_ID, "StatusLabel", "Gone on vacation", SELF)

		-- Set thermostats to vacation setting
		thermostats_set(TEMPERATURE.VACATION)
		
		-- Close all windows
		windows_close()
		
		-- Turn all lights off
		lights_set(0)

		start_timer(TIMER_AWAY,'run_away')
	end
end

-- Intrusion detected
function run_intrusion()
	luup.log("[MyHome] Intrusion was detected")
	start_timer(TIMER_INTRUSION,'run_alarm')
	
	-- TODO: Arm remote alarm manager
end

-- Run delayed alarm
function run_alarm()
	luup.log("[MyHome] ALARM!!!")
	-- TODO: Send messages
	-- TODO: Play sound
	-- TODO: Run sirens
	
	-- Turn all lights on
	lights_on()
	
	-- Open all blinds
	blinds_open()
	
	start_timer(TIMER_ALARM,'run_reset')
end

-- Run delayed reset
function run_reset()
	luup.log("[MyHome] Reset alarm")
	
	-- TODO: Stop sirens
	-- TODO: Disarm remote alarm manager
	
	set_status_away()
end

-- Run delayed away
function run_away()
	luup.log("[MyHome] Perform away")
  	for index,device in pairs(DEVICES.SEC_SENSOR) do
		luup.call_action(SID_SecuritySensor1, SENSOR_ACTION_ARM, {newArmedValue = 1}, device)
  	end
end

-- Set status
function set_status(new_status)
	local current_status = luup.variable_get(SERVICE_ID, "Status", SELF)
  	if (tonumber(new_status) ~= tonumber(current_status)) then
  		luup.log("[MyHome] Set status to " .. new_status)
   		luup.variable_set(SERVICE_ID, "Status", new_status, SELF)
   		return true
   	end
   	return false
end

-- Callback handling
function watch_callback(lul_device, lul_service, lul_variable, lul_value_old, lul_value_new)
	luup.log("[MyHome] Watched variable changed: " .. lul_device .. " " .. lul_service .. " " .. lul_variable .. " from " .. lul_value_old .. " to " .. lul_value_new)
	
	local data = {}
    data = read_config(SELF)
	
	if (tonumber(data.status) >= 0 and tonumber(data.status) < STATUS.INTRUSION) then
	  	luup.log("[MyHome] Callback data.status=" .. data.status , 100)
	 	check_tripped()
	end
end

function check_tripped()
  	local status = tonumber(read_or_init(SELF,SERVICE_ID, "Status",0))
  	local armed_tripped = 0
  
  	if (status > 0 and status < STATUS.INTRUSION) then
    	local data = {}
    	data = read_config(SELF)
  
    	-- Check Sensors Tripped Status
		for index,device in pairs(DEVICES.SEC_SENSOR) do
	  		local sensor_tripped = luup.variable_get(SID_SECURITYSENSOR,SENSOR_VARIABLE_TRIPPED, device)
		  	if sensor_tripped == "1" then
		    	local sensor_armed = luup.variable_get(SID_SECURITYSENSOR,SENSOR_VARIABLE_ARMED, device)
		    	if sensor_armed == "1" then
		    		-- Check Immediate Alarm 
		    		for j in pairs(DEVICES.SEC_SENSOR_IMMEDIATE) do
		    			local compare = DEVICES.SEC_SENSOR_IMMEDIATE[i]
		    			if compare == device then
		    				run_alarm(SELF, lul_settings)
		    				return
		    			end
		    		end
			  		armed_tripped = armed_tripped + 1
			  		
			  		-- TODO: Turn on lights in same room if luminosity low
				end
		  	end
		end
		if armed_tripped > 0 then
		  	run_intrusion(SELF, lul_settings)
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

function read_or_init(service_id, name, device, default)
	local value = luup.variable_get(service_id,name, device)
 	if (value == nil) then
   		value = default
   		luup.variable_set(service_id,name,value,device)
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

function devices_get(type,room)
	devices = {}
	
	for device_id, device_data in pairs(luup.devices) do
		local match = true
		if type ~= nil and device_data.device_type ~= type then
			match = false
		end
		if room ~= nil and device_data.room_num ~= room then
			match = false
		end
		if match == true then
			luup.log("FOUND DEVICE" .. device_data.device_type .. " - "..device_id)
			table.insert(devices,tonumber(device_id))
		end
	end
	
	return devices
end

-- 
-- TIMER
-- Various actors functions
--

function tick()
  	-- Timer may have been cancelled or forced.
  	-- If so, break out.
  	local counting = read_or_init(SERVICE_ID,"Counting",SELF, 0)
  	if (counting == "0") then
    	return false
  	end
  	if (update_remaining()) then
    	-- Timer is still underway.
    	return luup.call_delay("tick", 1, "") == 0
  	end
  	
  	-- Timer has completed.
  	set_if_changed(SERVICE_ID, "DueTimestamp", "", SELF)
  	set_if_changed(SERVICE_ID, "Remaining", 0, SELF)
  	set_if_changed(SERVICE_ID, "Counting", 0, SELF)
  
  	local timer_action = read_or_init(SERVICE_ID,"TimerAction",SELF, "")
  	if timer_action == "run_away" then
    	run_away()
  	elseif timer_action == "run_alarm" then
    	run_alarm()
   	elseif timer_action == "run_reset" then
   		run_reset()
  	end
  
  	return true
end

function start_timer(duration,action)
  	local counting = read_or_init(SERVICE_ID,"Counting",SELF, 0)
  	if (counting == "1") then
    	return false
  	end
  	set_if_changed(SERVICE_ID, "TimerDuration", duration, SELF)
  	return start_timer_always(action)
end

function start_timer_always(action)
	set_if_changed(SERVICE_ID, "TimerAction", action, SELF)
	
  	local counting = read_or_init(SERVICE_ID,"Counting",SELF, 0)
  	local duration = read_or_init(SERVICE_ID,"TimerDuration",SELF, 30)
  	local due_timestamp = os.time() + duration
  	set_if_changed(SERVICE_ID, "DueTimestamp", due_timestamp, SELF)
  	update_remaining()
  	set_if_changed(SERVICE_ID, "Counting", 1, SELF)
  	return luup.call_delay("tick", 1, "") == 0
end

function update_remaining()
  	local due_timestamp = read_or_init(SERVICE_ID,"DueTimestamp",SELF, "")
  	local remaining = tonumber(due_timestamp) - os.time()
  	if (remaining < 0) then remaining = 0 end
  	set_if_changed(SERVICE_ID, "Remaining", remaining, SELF)
  	return remaining > 0
end

function cancel_timer()
  	local counting = read_or_init(SERVICE_ID,"Counting",SELF, 0)
  	if (counting == "0") then
    	return false
  	end
  	set_if_changed(SERVICE_ID, "Counting", 0, SELF)
  	set_if_changed(SERVICE_ID, "DueTimestamp", "", SELF)
  	set_if_changed(SERVICE_ID, "Remaining", 0, SELF)
  	return true
end

-- 
-- DEVICE
-- Various device functions
--

function blind_partial(device,percentage)
	device = tonumber(device)
	if percentage == nil then
		percentage = BLINDS_PARTIAL
	end
	percentage = tonumber(percentage)

    luup.log("[MyHome] Moving blind " .. device .. " to " .. percentage .. "%")
    
    local status_key = "BlindStatus" .. device
    
    local blind_position_device = luup.variable_get(SID_DIMMING,"LoadLevelStatus",device)
    local blind_position_service = read_or_init(SERVICE_ID,status_key,SELF, blind_position_device)
    local blind_position = blind_position_service
    
    if (blind_position_device == 0 and blind_position_service ~= 0) then
    	blind_position = 0
    	luup.variable_set(SERVICE_ID, status_key, 0, SELF)
    end
    
    local blind_time = math.floor(( blind_position - percentage ) / 100 * TIMER_BLINDS)
	percentage = math.floor(( 1 -( blind_time / TIMER_BLINDS)) * 100)
	
	if percentage ~= blind_position then
	    luup.call_action(SID_DIMMING, "SetLoadLevelTarget", { newLoadlevelTarget = percentage }, device)
	    luup.variable_set(SERVICE_ID, status_key, percentage, SELF)
	
	    if blind_time < 0 then
			blind_time = blind_time * -1
	        luup.call_action(SID_WINDOWCOVERING, "Up", {}, device)
	    elseif blind_time > 0 then
	        luup.call_action(SID_WINDOWCOVERING, "Down", {}, device)
	    end
	
		luup.call_delay("blind_stop", blind_time, tostring(device))
	end
end

function blind_stop(device) 
	device = tonumber(device)
    luup.log("[MyHome] Stoping blind " .. device)
    luup.call_action(SID_WINDOWCOVERING, "Stop", {}, device)
end

function blind_open(device) 
	device = tonumber(device)
    luup.log("[MyHome] Opening blind " .. device)
    luup.call_action(SID_DIMMING, "SetLoadLevelTarget", { newLoadlevelTarget = 100 }, device)
    luup.variable_set(SERVICE_ID, "BlindStatus" .. device, 100, SELF)
end

function light_off(device)
	device = tonumber(device)
	luup.log("[MyHome] Turn light off " .. device)
	luup.call_action(SID_SWITCHPOWER, "SetTarget", { newTargetValue = 0 }, device)
end

-- 
-- ACTORS
-- Various actor functions
--

-- Periodic automator run
function run_automator()
	local data = read_config(SELF)
	
	if data.status == STATUS.AWAY then
		
		lights_random()
		blinds_temperature()
		windows_temperature()
		
	elseif data.status ==STATUS.VACATION then
	
		lights_random()
		
	elseif data.status == STATUS.HOME then
	
	 	--blinds_wakeup()
		thermostats_auto()
		blinds_temperature() -- ???
		windows_temperature()
		
	end
end

function weather_status()
	local weather = {}
	
	weather.condition 	= "POOR"
	weather.temperature = tonumber(luup.variable_get(SID_TEMPERATURE,"CurrentTemperature",DEVICES.TMP_SENSOR_OUTSIDE))
	weather.wind_speed 	= tonumber(luup.variable_get(SID_WEATHER,"WindSpeed",DEVICES.TMP_SENSOR_OUTSIDE))
	--weather.rain		= luup.variable_get(SID_SECURITYSENSOR,"CurrentTemperature",DEVICES.RAIN_SENSOR)
	
	local weather = luup.variable_get(SID_WEATHER,"ConditionGroup",DEVICES.WEATHER)
	for index, value in ipairs(WEATHER) do
		if value.value[weather] then
			luup.log("[MyHome] Check weather " .. weather .. " is " .. value.key)
			weather.condition = value.key
			return weather
		end
	end
	
	return weather
end

function blinds_temperature()
	local inside_temperature 	= luup.variable_get(SID_TEMPERATURE,"CurrentTemperature",DEVICES.TEMP_SENSOR_INSIDE)
	local weather_status 		= weather_status()
	local blinds_auto 			= read_or_init(SERVICE_ID,"BlindStatusAuto",SELF, 0)
	
	-- check weather
	if (inside_temperature >= TEMPERATURE.MAX and weather_status.condition == "FAIR") then
		-- check time
		local time = os.date('*t')
		if (time.hour >= 10 and time.hour <= 14) then
			-- check if blinds are already down
			if blinds_auto == 0 then
				luup.variable_set(SERVICE_ID,"BlindStatusAuto",1,SELF)
				for device in pairs(DEVICES.BLINDS_SOUTH) do
					blind_partial(device)
				end
			end
			return
		end
	end
	
	if blinds_auto == 1 then
		luup.variable_set(SERVICE_ID,"BlindStatusAuto",0,SELF)
		for device in pairs(DEVICES.BLINDS_SOUTH) do
			blind_open(device)
		end
	end
end

function blinds_wakeup()
	-- TODO: Move blinds up on weekdays at 7
end

function lights_random()
	local lights = devices_get(DID_LIGHT,nil)
	
	-- Get lights and return if some light is already running
	for index,device in pairs(lights) do
    	local light_status = luup.variable_get(SID_SWITCHPOWER,"Status",device)
    	if tonumber(light_status) == 1 then
			return	    	
    	end
	end
	
	luup.log("RANDOM".. table.getn(lights))

    local time = os.date('*t')
	if (luup.is_night() and time.hour >= 18 and time.hour <= 22) then

		device_index 	= math.floor(math.random(0,table.getn(lights)))
		device_on		= math.floor(math.random(10))
		device_time 	= math.floor(math.random(60,900))
	luup.log("RANDOM Index:".. device_index.. " On:"..device_on.." Time: " ..device_time)
		if (device_on == 1) then
			log("[MyHome] Turn random light on " .. device)
			local device = lights[device_index]
			luup.call_action(SID_SWITCHPOWER, "SetTarget", { newTargetValue = 1 }, device)
			luup.call_delay("light_off", device_time, device)
		end
	else
		lights_set(0)
	end
end

function lights_set(target)
	for index,device in pairs(devices_get(DID_LIGHT,nil)) do
		luup.call_action(SID_SWITCHPOWER, "SetTarget", { newTargetValue = target }, tonumber(device))
	end
end

function windows_temperature()
	local inside_temperature	= tonumber(luup.variable_get(SID_TEMPERATURE,"CurrentTemperature",DEVICES.TEMP_SENSOR_INSIDE))
	local weather_status		= weather_status()
	local windows_auto 			= read_or_init(SERVICE_ID,"WindowsStatusAuto",SELF, 0)
	
	-- check weather, wind
	if (inside_temperature >= TEMPERATURE.MAX and inside_temperature > weather.temperature and (weather_status.condition == "FAIR" or weather_status.condition == "NORMAL") and weather_status.wind_speed < WIND_SPEED_LIMIT) then
		-- check if blinds are already down
		if windows_auto == 0 then
			luup.variable_set(SERVICE_ID,"WindowsStatusAuto",1,SELF)
			for index,device in pairs(DEVICES.BLINDS_SOUTH) do
				blind_partial(device,BLINDS_PARTIAL)
			end
		end
		return
	end
	
	if windows_auto == 1 then
		luup.variable_set(SERVICE_ID,"WindowsStatusAuto",0,SELF)
		for index,device in pairs(DEVICES.BLINDS_SOUTH) do
			blind_open(device)
		end
	end
end

function windows_close()
	-- TODO
	--for index,device in pairs(DEVICES.WINDOWS) do
	--end
end

function thermostats_set()
	-- TODO
end

function thermostats_auto(temperature)
	-- TODO
end
