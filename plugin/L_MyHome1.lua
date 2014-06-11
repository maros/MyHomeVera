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

-- luup.call_action("urn:upnp-k1-com:serviceId:MyHome1", "lights_set", { target = 1}, 62)
-- luup.call_action("urn:upnp-k1-com:serviceId:MyHome1", "lights_random",{}, 62)
-- luup.call_action("urn:upnp-k1-com:serviceId:MyHome1","device_move",{ device = 30, percentage = 80 },62)

function Set(list)
	local set = {}
	for _, l in ipairs(list) do set[l] = true end
	return set
end

local SELF						= nil

local STATUS					= {}
STATUS.CHANGING					= -1
STATUS.HOME						= 0
STATUS.AWAY						= 10
STATUS.VACATION					= 20
STATUS.INTRUSION				= 90
STATUS.ALARM					= 100

local TIMER						= {}
TIMER.WINDOWS					= 30
TIMER.BLINDS					= 17	-- timer for blinds to move fully up/down
TIMER.AWAY						= 300	-- timer for alarm to be armed after leaving
TIMER.INTRUSION					= 60	-- timer for alarm to be disarmed after coming
TIMER.ALARM						= 600	-- timer for alarm

local BLINDS					= {}
BLINDS.PARTIAL					= 20
BLINDS.TEMPERATURE_OUTSIDE		= 22
BLINDS.LOCATION					= {}
BLINDS.LOCATION.south			= {}
BLINDS.LOCATION.south.START		= 10
BLINDS.LOCATION.south.END		= 15
BLINDS.LOCATION.livingroom		= {}
BLINDS.LOCATION.livingroom.START= 18
BLINDS.LOCATION.livingroom.END	= 20

local TEMPERATURE				= {}
TEMPERATURE.MAX					= 25
TEMPERATURE.VACATION			= 15
TEMPERATURE.LOW					= 18
TEMPERATURE.DEFAULT				= 20

local WEATHER					= {}
WEATHER.POOR					= Set { "chanceflurries", "chancesleet", "chancesnow", "chancetstorms", "flurries","sleet","rain","snow","tstorms","unknown" }
WEATHER.NEUTRAL					= Set { "cloudy", "fog", "mostlycloudy", "chancerain" }
WEATHER.FAIR					= Set { "clear", "hazy", "mostlysunny", "partlysunny", "partlycloudy" }

local ALARM_SECRET				= 'TODO CHANGE'
local ALARM_SERVER				= 'http:://alarm.k-1.com'
local WIND_SPEED_LIMIT			= 5
local LUMINOSITY_LEVEL			= 4
local SUNOFFSET					= 45 * 60

local DEVICES					= {
	[20]							= {
		["class"]						= "SecSensor"
	},
	[35]							= {
		["class"]						= "SecSensor",
		["immediate"]					= true
	},
	[63]							= {
		["class"]						= "SecSensor",
		["trigger"]						= 57
	},
	[64]							= {
		["class"]						= "TempSensor",
		["location"]					= "inside"
	},	
	[68]							= {
		["class"]						= "TempSensor",
		["location"]					= "outside"
	},
	[65]							= {
		["class"]						= "LightSensor",
	},
	[67]							= {
		["class"]						= "Weather"
	},
	[21]							= {
		["class"]						= "Blinds",
		["location"]					= "south",
		["position"]					= "lower",
		["timer"]						= TIMER.BLINDS
	},
	[22]							= {
		["class"]						= "Blinds",
		["location"]					= "south",
		["position"]					= "lower",
		["timer"]						= TIMER.BLINDS
	},
	[23]							= {
		["class"]						= "Blinds",
		["location"]					= "south",
		["position"]					= "lower",
		["timer"]						= TIMER.BLINDS
	},
	[24]							= {
		["class"]						= "Blinds",
		["location"]					= "south",
		["position"]					= "lower",
		["timer"]						= TIMER.BLINDS
	},
	[28]							= {
		["class"]						= "Blinds",
		["location"]					= "south",
		["position"]					= "upper",
		["timer"]						= TIMER.BLINDS
	},
	[29]							= {
		["class"]						= "Blinds",
		["location"]					= "south",
		["position"]					= "upper",
		["timer"]						= TIMER.BLINDS
	},
	[32]							= {
		["class"]						= "Blinds",
		["location"]					= "north",
		["position"]					= "lower",
		["timer"]						= TIMER.BLINDS
	},
	[33]							= {
		["class"]						= "Blinds",
		["location"]					= "north",
		["position"]					= "upper",
		["wakeup"]						= true,
		["timer"]						= TIMER.BLINDS
	},
	[26]							= {
		["class"]						= "Blinds",
		["location"]					= "north",
		["position"]					= "lower",
		["timer"]						= TIMER.BLINDS
	},
	[30]							= {
		["class"]						= "Blinds",
		["location"]					= "north",
		["position"]					= "upper",
		["related"]						= 81,
		["timer"]						= TIMER.BLINDS
	},
	[25]							= {
		["class"]						= "Blinds",
		["location"]					= "north",
		["position"]					= "lower",
		["timer"]						= TIMER.BLINDS
	},
	[31]							= {
		["class"]						= "Blinds",
		["location"]					= "north",
		["position"]					= "upper",
		["timer"]						= TIMER.BLINDS
	},
	[73]							= {
		["class"]						= "Presence"
	},
	[74]							= {
		["class"]						= "LockBlinds"
	},
	[75]							= {
		["class"]						= "LockWindows"
	},
	[76]							= {
		["class"]						= "LockAll"
	},
	[83]							= {
		["class"]						= "RainSensor"
	},
--	[84]							= {
--		["class"]						= "SecSensor",
--		["immediate"]					= true
--	},
	[80]							= {
		["class"]						= "LockLights"
	},
	[81]							= {
		["class"]						= "Window",
		["related"]						= 30,
		["timer"]						= TIMER.WINDOWS
--	},
--	[82]							= {
--		["class"]						= "Window",
	}
}

local SID_SELF					= "urn:upnp-k1-com:serviceId:MyHome1"
local SID_WINDOWCOVERING		= "urn:upnp-org:serviceId:WindowCovering1"
local SID_SWITCHPOWER			= "urn:upnp-org:serviceId:SwitchPower1"
local SID_DIMMING				= "urn:upnp-org:serviceId:Dimming1"
local SID_TEMPERATURE			= "urn:upnp-org:serviceId:TemperatureSensor1"
local SID_SECURITYSENSOR		= "urn:micasaverde-com:serviceId:SecuritySensor1"
local SID_WEATHER				= "urn:upnp-micasaverde-com:serviceId:Weather1"
local SID_VSWITCH				= "urn:upnp-org:serviceId:VSwitch1"
local SID_LUMINSOITY			= "urn:micasaverde-com:serviceId:LightSensor1"

local DID_LIGHT					= "urn:schemas-upnp-org:device:BinaryLight:1"
local DID_DOORSENSOR			= "urn:schemas-micasaverde-com:device:DoorSensor:1"
local DID_WINDOWCOVERING		= "urn:schemas-micasaverde-com:device:WindowCovering:1"

-- 
-- FUNCTION
-- Init Device
--
function initialize(lul_device)
	SELF = lul_device
	
	luup.log("[MyHome] Initialize MyHome")
	
	local data = read_config()
	
	luup.log("[MyHome] Fixup device status")
	if data.status == 0 then
		if data.presence == 0 then
			luup.log("[MyHome] Detected presence mismatch (set to home)")
			luup.call_action(SID_SWITCHPOWER, "SetTarget", { newTargetValue = 1 }, data.device_presence)
			luup.call_action(SID_VSWITCH, "SetTarget", { newTargetValue = 1 }, data.device_presence)
		end
	else
		if data.presence == 1 then
			luup.log("[MyHome] Detected presence mismatch (set to away)")
			luup.call_action(SID_SWITCHPOWER, "SetTarget", { newTargetValue = 0 }, data.device_presence)
			luup.call_action(SID_VSWITCH, "SetTarget", { newTargetValue = 0 }, data.device_presence)
		end
	end
	
	luup.log("[MyHome] Watch Security sensors")
	for index,device in pairs(devices_search({ ["class"] = "SecSensor" })) do
		-- luup.variable_watch("watch_callback", SID_SECURITYSENSOR, "Armed", device)
		luup.variable_watch("watch_alarm_callback", SID_SECURITYSENSOR, "Tripped", device)
		if device_attr(device,"trigger") ~= nil then
			luup.variable_watch("watch_light_callback", SID_SECURITYSENSOR, "Tripped", device)
		end
	end
	
	-- luup.variable_watch("watch_presence", SID_SWITCHPOWER, nil, data.device_presence)
	-- luup.variable_watch("watch_presence", SID_VSWITCH, nil, data.device_presence)
	
	-- Close windows on rain
	luup.variable_watch("windows_close", SID_SECURITYSENSOR, "Tripped", device_search_single({ ["class"] = "RainSensor" }))
	
	-- Re-start countdown
	tick()
	
	run_automator()
	
	luup.log("[MyHome] Finished Initialize")
	
	return true
end

function watch_presence(lul_device, lul_service, lul_variable, lul_value_old, lul_value_new)
	luup.log("[MyHome] Presence set via VSwitch")
	if lul_value_old ~= lul_value_new then
		if lul_value_new == 1 then
			set_status_home(lul_device)
		elseif lul_value_new == 0 then
			set_status_away(lul_device)
		end
	end
end	

-- Manually set status = Home
function set_status_home(lul_device)
	if (set_status(STATUS.HOME)) then
		set_if_changed(SID_SELF, "StatusLabel", "Somebody is home", SELF)
		
		-- Cancel timer
		cancel_timer()
		
		-- Disarm remote alarm
		remote_call('reset',{ message = 'Somebody came home' })
		
		-- Disarm sensors immediately
		for index,device in pairs(devices_search({ ["class"] = "SecSensor" })) do
			luup.call_action(SID_SECURITYSENSOR, "SetArmed", {newArmedValue = 0}, device)
		end
	end
end

-- Manually set status = Away
function set_status_away(lul_device)
	if (set_status(STATUS.AWAY)) then
		set_if_changed(SID_SELF, "StatusLabel", "Nobody is home", SELF)
		
		-- Set thermostats to low setting
		thermostats_set(TEMPERATURE.LOW)
		
		-- Turn all lights off
		lights_set(0)
		
		start_timer(TIMER.AWAY,'run_away')
	end
end

-- Manually set status = Vacation
function set_status_vacation(lul_device)
	if (set_status(STATUS.VACATION)) then
		set_if_changed(SID_SELF, "StatusLabel", "Gone on vacation", SELF)

		-- Set thermostats to vacation setting
		thermostats_set(TEMPERATURE.VACATION)
		
		-- Close all windows
		windows_close()
		
		-- Turn all lights off
		lights_set(0)

		start_timer(TIMER.AWAY,'run_away')
	end
end

-- Intrusion detected
function run_intrusion(lul_device,message)
	luup.log("[MyHome] Intrusion was detected")
	luup.variable_set(SID_SELF, "AlarmMessage", message, SELF)
	start_timer(TIMER.INTRUSION,"run_alarm")
	remote_call('intrusion',{ message = message, timer = TIMER.INTRUSION })	
end

-- Run delayed alarm
function run_alarm(lul_device)
	luup.log("[MyHome] ALARM!!!")
	
	-- TODO: Run sirens
	
	-- Turn all lights on
	lights_set(1)
	
	-- Closing all lower blinds 
	for index,device in pairs(devices_search({ ["class"] = "Blinds", ["position"] = "lower" })) do
		device_move(device,0)
	end
	
	-- Open all upper blinds 
	for index,device in pairs(devices_search({ ["class"] = "Blinds", ["position"] = "upper" })) do
		device_move(device,100)
	end
	
	-- Close all windows
	windows_close()
	
	start_timer(TIMER.ALARM,"run_reset")
end

-- Run delayed reset
function run_reset()
	luup.log("[MyHome] Reset alarm")
	
	-- TODO: Stop sirens
	
	lights_set(0)
	
	set_status_away(SELF)
end

-- Run delayed away
function run_away()
	luup.log("[MyHome] Perform away")
	for index,device in pairs(devices_search({ ["class"] = "SecSensor" })) do
		luup.call_action(SID_SECURITYSENSOR, "SetArmed", {newArmedValue = 1}, device)
	end
end

-- Set status
function set_status(new_status)
	local data = read_config()
	if (tonumber(new_status) ~= data.status) then
		luup.log("[MyHome] Set status to " .. new_status)
		local presence = 0
		if new_status == 0 then
			presence = 1
		end
		
		-- Set presence status
		luup.log("[MyHome] Set presence to " .. presence .. " for " .. data.device_presence)
		luup.call_action(SID_SWITCHPOWER, "SetTarget", { newTargetValue = presence }, data.device_presence)
		luup.call_action(SID_VSWITCH, "SetTarget", { newTargetValue = presence }, data.device_presence)
		
		luup.variable_set(SID_SELF,"AlarmMessage","", SELF)
		luup.variable_set(SID_SELF, "Status", new_status, SELF)
		return true
	end
	return false
end

-- Callback handling
function watch_alarm_callback(lul_device, lul_service, lul_variable, lul_value_old, lul_value_new)
	luup.log("[MyHome] Watched alarm variable changed: " .. lul_device .. " " .. lul_service .. " " .. lul_variable .. " from " .. lul_value_old .. " to " .. lul_value_new)

	local data			= read_config()
	local alarm_message	= ""

	if (data.status >= 0 and data.status < STATUS.INTRUSION) then
		luup.log("[MyHome] Callback data.status=" .. data.status , 100)
		local armed_tripped = 0

		-- Check Sensors Tripped Status
		for index,device in pairs(devices_search({ ["class"] = "SecSensor" })) do
			if sensor_tripped == "1" then
				local sensor_armed = luup.variable_get(SID_SECURITYSENSOR,"Armed", device)
				if sensor_armed == "1" then
					local message = 'Sensor '.. luup.devices[device].name .. ' tripped'
					-- Check Immediate Alarm 
					if device_attr(device,"immediate") == true then
						run_alarm(SELF, lul_settings)
						-- Call alarm
						remote_call("run",{ message = message })
						return
					else
						alarm_message = alarm_message .. message .. ","
					end
					armed_tripped = armed_tripped + 1
				end
			end
		end
		if armed_tripped > 0 then
			luup.variable_set(SID_SELF,"AlarmMessage",alarm_message, SELF)
			run_intrusion(SELF, alarm_message)
		end
	end
end

function watch_light_callback(lul_device, lul_service, lul_variable, lul_value_old, lul_value_new)
	luup.log("[MyHome] Watched light variable changed: " .. lul_device .. " " .. lul_service .. " " .. lul_variable .. " from " .. lul_value_old .. " to " .. lul_value_new)

	local data				= read_config()
	local triggered_device	= device_attr(lul_device,"trigger")
	
	if triggered_device == nil or data.lock_lights == 1 then
		luup.log("[MyHome] Not triggering")
		return
	end
	
	for index,device in pairs(devices_search({ ["device_type"] = DID_LIGHT })) do
		local lighton = luup.variable_get(SID_SWITCHPOWER,"Status",device)
		lighton = tonumber(lighton)
		if lighton == 1 then
			return
		end
	end
	
	local status_key		= "LightStatus" .. triggered_device
	local light_status		= read_or_init(SID_SELF,status_key,SELF,0)
	local current_status	= luup.variable_get(SID_SWITCHPOWER,"Status",triggered_device)
	local luminosity		= luup.variable_get(SID_LUMINSOITY,"CurrentLevel",device_search_single({ ["class"] = "LightSensor" }))
	local daynight_status	= daynight_status()
	luminosity				= tonumber(luminosity)
	current_status			= tonumber(current_status)
	light_status			= tonumber(light_status)
	lul_value_old			= tonumber(lul_value_old)
	lul_value_new			= tonumber(lul_value_new)
	
	if (current_status == 0 and lul_value_old == 0 and lul_value_new == 1 and luminosity <= LUMINOSITY_LEVEL and daynight_status == "night") then
		device_on(triggered_device)
	elseif (current_status == 1 and lul_value_old == 1 and lul_value_new == 0 and light_status == 1) then
		device_off(triggered_device)
	end
end

-- 
-- HELPER
--

local function read_config()

	local data = {}
	data.status				= read_or_init(SID_SELF, "Status", SELF, 0)
	data.statuslabel		= read_or_init(SID_SELF, "StatusLabel", SELF, "Disarmed")
	
	data.device_presence	= device_search_single({ ["class"] = "Presence" })
	data.device_lock_windows= device_search_single({ ["class"] = "LockWindows" })
	data.device_lock_blinds	= device_search_single({ ["class"] = "LockBlinds" })
	data.device_lock_lights	= device_search_single({ ["class"] = "LockLights" })
	data.device_lock_all	= device_search_single({ ["class"] = "LockAll" })
	
	data.device_presence	= tonumber(data.device_presence)
	data.device_lock_windows= tonumber(data.device_lock_windows)
	data.device_lock_blinds = tonumber(data.device_lock_blinds)
	data.device_lock_all	= tonumber(data.device_lock_all)

	data.presence			= luup.variable_get(SID_VSWITCH,"Status",data.device_presence)
	data.lock_windows		= luup.variable_get(SID_VSWITCH,"Status",data.device_lock_windows)
	data.lock_blinds		= luup.variable_get(SID_VSWITCH,"Status",data.device_lock_blinds)
	data.lock_lights		= luup.variable_get(SID_VSWITCH,"Status",data.device_lock_lights)
	data.lock_all			= luup.variable_get(SID_VSWITCH,"Status",data.device_lock_all)
	
	data.presence			= tonumber(data.presence)
	data.status				= tonumber(data.status)
	data.lock_windows		= tonumber(data.lock_windows)
	data.lock_lights		= tonumber(data.lock_lights)
	data.lock_blinds		= tonumber(data.lock_blinds)
	data.lock_all			= tonumber(data.lock_all)
	
	if data.lock_all == 1 then
		data.lock_blinds = 1
		data.lock_windows = 1
		data.lock_lights = 1
	end
	
	return data
end

local function read_or_init(service_id, name, device, default)
	local value = luup.variable_get(service_id,name, device)
	if (value == nil or value == "") then
		value = default
		luup.variable_set(service_id,name,value,device)
	end
	return value
end

local function set_if_changed(service_id, name, value, device)
	local curValue = luup.variable_get(service_id, name, device)
	if ((value ~= curValue) or (curValue == nil)) then
		luup.variable_set(service_id, name, value, device)
		return true
	else
		return false
	end
end

local function device_attr(device,attr)
	if DEVICES[device] == nil then
		return
	else
		return DEVICES[device][attr]
	end
end

local function device_search_single(search)
	local result = devices_search(search)
	if table.getn(result) > 0 then
		return result[1]
	end
	return
end

local function devices_search(search)
	devices = {}
	
	for device_id, device_data in pairs(luup.devices) do
		local match = true
		
		for search_key, search_value in pairs(search) do
			if match == true then
				local device_attr = DEVICES[device_id]
				
				if device_attr[search_key] =~ nil then
					local compare_value = device_attr[search_key]
					if compare_value ~= search_value then
						match = false
					end
				else
					local compare_value = device_data[search_key]
					if compare_value == nil or compare_value ~= search_value then
						match = false
					end
				end
			end
		end
		if match == true then
			--luup.log("[MyHome] Found Device " .. device_data.device_type .. " - "..device_id)
			table.insert(devices,tonumber(device_id))
		end
	end
	
	if table.getn(devices) == 0 then
		luup.log("[MyHome] Could not find any devices")
	end
	
	return devices
end

-- 
-- TIMER
-- Various actors functions
--

local function tick()
	-- Timer may have been cancelled or forced.
	-- If so, break out.
	local counting = read_or_init(SID_SELF,"Counting",SELF, 0)
	if (counting == "0") then
		return false
	end
	if (update_remaining()) then
		-- Timer is still underway.
		return luup.call_delay("tick", 1, "") == 0
	end
	
	-- Timer has completed.
	set_if_changed(SID_SELF, "DueTimestamp", "", SELF)
	set_if_changed(SID_SELF, "Remaining", 0, SELF)
	set_if_changed(SID_SELF, "Counting", 0, SELF)

	local timer_action = read_or_init(SID_SELF,"TimerAction",SELF, "")
	if timer_action == "run_away" then
		run_away()
	elseif timer_action == "run_alarm" then
		run_alarm()
	elseif timer_action == "run_reset" then
		run_reset()
	end

	return true
end

local function start_timer(duration,action)
	local counting = read_or_init(SID_SELF,"Counting",SELF, 0)
	if (counting == "1") then
		return false
	end
	set_if_changed(SID_SELF, "TimerDuration", duration, SELF)
	return start_timer_always(action)
end

local function start_timer_always(action)
	set_if_changed(SID_SELF, "TimerAction", action, SELF)
	
	local counting = read_or_init(SID_SELF,"Counting",SELF, 0)
	local duration = read_or_init(SID_SELF,"TimerDuration",SELF, 30)
	local due_timestamp = os.time() + duration
	set_if_changed(SID_SELF, "DueTimestamp", due_timestamp, SELF)
	update_remaining()
	set_if_changed(SID_SELF, "Counting", 1, SELF)
	return luup.call_delay("tick", 1, "") == 0
end

function update_remaining()
	local due_timestamp = read_or_init(SID_SELF,"DueTimestamp",SELF, "")
	local remaining = tonumber(due_timestamp) - os.time()
	if (remaining < 0) then remaining = 0 end
	set_if_changed(SID_SELF, "Remaining", remaining, SELF)
	return remaining > 0
end

local function cancel_timer()
	local counting = read_or_init(SID_SELF,"Counting",SELF, 0)
	if (counting == "0") then
		return false
	end
	set_if_changed(SID_SELF, "Counting", 0, SELF)
	set_if_changed(SID_SELF, "DueTimestamp", "", SELF)
	set_if_changed(SID_SELF, "Remaining", 0, SELF)
	return true
end

-- 
-- DEVICE
-- Various device functions
--

function device_position_get(device)
	device					= tonumber(device)
	local position_key		= "DevicePosition" .. device
	local position_service	= luup.variable_get(SID_SELF,status_key,SELF)
	local position_device	= luup.variable_get(SID_DIMMING,"LoadLevelStatus",device)
	position_service		= tonumber(position_service)
	position_device			= tonumber(position_device)
	
	if (position_device == 0 and position_device ~= 0) then
		position_device = 0
		luup.variable_set(SID_SELF, position_key, 0, SELF)
	end
	
	return position_device
end

function device_position_set(device,percentage)
	device		= tonumber(device)
	percentage	= tonumber(percentage)
	luup.call_action(SID_DIMMING, "SetLoadLevelTarget", { newLoadlevelTarget = percentage }, device)
	luup.variable_set(SID_SELF, "DevicePosition" .. device, percentage, SELF)
end

function device_stop(device) 
	device = tonumber(device)
	luup.log("[MyHome] Stoping device " .. device)
	luup.call_action(SID_WINDOWCOVERING, "Stop", {}, device)
end

function device_open(device) 
	device = tonumber(device)
	device_move(100)
end

function device_close(device)
	device = tonumber(device)
	device_move(0)
end

function device_move(device,percentage)
	device		= tonumber(device)
	percentage	= tonumber(percentage)
	
	-- Check related device
	local related = device_attr(device,"related")
	-- window 0-50 - blind 0-100
	-- window 50-100 - blind max 50-100
	--if related != nil then 
	--	local related_position = device_position_get(related)
	--	if related_position > 50 and percentage < 60 then
	--		percentage = 60
	--	end
	--end

	luup.log("[MyHome] Moving device " .. device .. " to " .. percentage .. "%")
	
	if (percentage == 100 or percentage == 0) then
		device_position_set(device,percentage)
	else
		local blind_position	= device_position_get(device)
		local blind_time		= math.floor(( blind_position - percentage ) / 100 * TIMER.BLINDS)
		--percentage			= math.floor(( 1 -( blind_time / TIMER.BLINDS)) * 100)
		
		if blind_time ~= 0 and percentage ~= blind_position then
			device_position_set(device,percentage)
		
			if blind_time < 0 then
				blind_time = blind_time * -1
				luup.call_action(SID_WINDOWCOVERING, "Up", {}, device)
			elseif blind_time > 0 then
				luup.call_action(SID_WINDOWCOVERING, "Down", {}, device)
			end
		
			luup.call_delay("device_stop", blind_time, tostring(device))
		end
	end
end

function device_off(device)
	device = tonumber(device)
	luup.log("[MyHome] Turn light off " .. device)
	luup.call_action(SID_SWITCHPOWER, "SetTarget", { newTargetValue = 0 }, device)
	luup.variable_set(SID_SELF, "DeviceStatus" .. device, 0, SELF)
end

function device_on(device)
	device = tonumber(device)
	luup.log("[MyHome] Turn light on " .. device)
	luup.call_action(SID_SWITCHPOWER, "SetTarget", { newTargetValue = 1 }, device)
	luup.variable_set(SID_SELF, "DeviceStatus" .. device, 1, SELF)
end

-- 
-- ACTORS
-- Various actor functions
--

-- Periodic automator run
function run_automator()
	luup.log("[MyHome] Run automator")

	local data = read_config()
	
	-- Check status
	for index,device in pairs(devices_search({ ["device_type"] = DID_LIGHT })) do
		if luup.variable_get(SID_SWITCHPOWER,"Status",device) == 0 then
			set_if_changed(SID_SELF, "DeviceStatus" .. device, 0, SELF)
		end
	end
	
	if data.lock_all == 0 then
		luup.log("[MyHome] Run automator actions")
	
		if data.status == STATUS.AWAY then
			
			lights_random()
			blinds_temperature("south")
			blinds_temperature("livingroom")
			
			windows_temperature()
			
		elseif data.status ==STATUS.VACATION then
		
			lights_random()
			
		elseif data.status == STATUS.HOME then
		
			thermostats_auto()
			blinds_temperature("south")
			blinds_temperature("livingroom")
			windows_temperature()
			
		end
	else
		luup.log("[MyHome] Skip automator actions" .. data.lock_all)
	end
	
	luup.call_delay("run_automator", 180, "")
end

function temperature_inside()
	local device_temperature = device_search_single({ ["class"] = "TempSensor", ["location"] = "inside" })
	local temperature_inside = luup.variable_get(SID_TEMPERATURE,"CurrentTemperature",device_temperature)
	return tonumber(temperature_inside)
end

function weather_status()
	local device_weather		= device_search_single({ ["class"] = "Weather" })
	local device_temperature	= device_search_single({ ["class"] = "TempSensor", ["location"] = "outside" })
	local device_rain			= device_search_single({ ["class"] = "RainSensor" })
	
	local weather = {}
	weather.condition	= "POOR"
	weather.rain		= false
	weather.temperature = luup.variable_get(SID_TEMPERATURE,"CurrentTemperature",device_temperature)
	weather.temperature = tonumber(weather.temperature)
	weather.wind_speed	= luup.variable_get(SID_WEATHER,"WindSpeed",device_weather)
	weather.wind_speed	= tonumber(weather.wind_speed)
	weather.rain_sensor = luup.variable_get(SID_SECURITYSENSOR,"Tripped",device_rain)
	
	local condition = luup.variable_get(SID_WEATHER,"ConditionGroup",device_weather)
	
	for key, value in pairs(WEATHER) do
		if value[condition] ~= nil then
			weather.condition = key
		end
	end
	
	-- TODO
	if weather.condition == "POOR" or weather.rain_sensor == true then
		weather.rain = true
	end
	
	luup.log("[MyHome] Check weather is " .. weather.condition)
	return weather
end

function daynight_status()
	local sunset		= luup.sunset()
	local sunrise		= luup.sunrise()
	local now			= os.time()
	sunset				= sunset + SUNOFFSET
	sunrise				= sunrise - SUNOFFSET
	local sunset_prev	= sunset - (24 * 60 * 60)
	
	if sunrise < now or sunrise > sunset or sunset_prev > now then
		return "day"
	else
		return "night"
	end
end

function blinds_temperature(location)
	local data = read_config()
	
	if data.lock_blinds == 1 then
		luup.log("[MyHome] Blinds locked")
		return
	end
	
	local temperature_inside	= temperature_inside()
	local weather_status		= weather_status()
	local blinds_auto_key		= "BlindStatusAuto"..location
	local action				= "keep"
	local devices_blinds		= devices_search({ ["class"] = "Blinds", ["location"] = location })
	local blinds_auto			= read_or_init(SID_SELF,blinds_auto_key,SELF, 0)
	blinds_auto					= tonumber(blinds_auto)
	
	local time = os.date('*t')
	if (time.hour < BLINDS.LOCATION[location].START or time.hour > BLINDS.LOCATION[location].END or daynight_status() == "night") then
		if blinds_auto == 1 then
			luup.log("[MyHome] Opening ".. location.." blinds (time)")
			action = "open"
		end
	elseif (temperature_inside >= TEMPERATURE.MAX and weather_status.condition == "FAIR" and weather_status.temperature >= BLIND.TEMPERATURE_OUTSIDE) then
		if blinds_auto == 0 then
			luup.log("[MyHome] Closing ".. location.." blinds (time & temperature)")
			action = "close"
		end
	elseif (temperature_inside < (TEMPERATURE.MAX-1) or weather_status.condition ~= "FAIR") then
		if blinds_auto == 1 then
			luup.log("[MyHome] Opening ".. location.." blinds (temperature)")
			action = "open"
		end
	end
	
	if action == "open" then
		for index,device in pairs(devices_blinds) do
			device_open(device)
		end
		luup.variable_set(SID_SELF,blinds_auto_key,0,SELF)
	elseif action == "close" then
		for index,device in pairs(devices_blinds) do
			device_move(device,BLINDS.PARTIAL)
		end
		luup.variable_set(SID_SELF,blinds_auto_key,1,SELF)
	end
end

function blinds_wakeup()
	local data = read_config()
	
	if data.status == 0 and data.lock_blinds == 0 then
		luup.log("[MyHome] Opening blinds (wakeup)")
		for index,device in pairs(devices_search({ ["class"] = "Blinds", ["wakeup"] = true })) do
			device_open(device)
		end
	end
end

function lights_random()
	local data = read_config()
	
	if data.lock_lights == 1 then
		luup.log("[MyHome] Lights locked")
		return
	end
	
	local lights = devices_search({ ["device_type"] = DID_LIGHT })
	
	-- Get lights and return if some light is already running
	for index,device in pairs(lights) do
		local light_status = luup.variable_get(SID_SWITCHPOWER,"Status",device)
		if tonumber(light_status) == 1 then
			return
		end
	end
	
	local time = os.date('*t')
	if (daynight_status() == "night" and time.hour >= 18 and time.hour <= 22) then
	
		local device		= lights[math.random( #lights )]
		local device_on		= math.floor(math.random(5))
		local device_time	= math.floor(math.random(60,719))
		
		if (device_on == 1) then
			luup.log("[MyHome] Turn random light on " .. device)
			device_on(device)
			luup.call_delay("device_off", device_time, device)
		end
	else
		lights_set(0)
	end
end

function lights_set(target)
	target = tonumber(target)

	luup.log("[MyHome] Set all light to " .. target)
	for index,device in pairs(devices_search({ ["device_type"] = DID_LIGHT })) do
		if target == 1 then
			device_on(device)
		elseif target == 0 then
			device_off(device)
		end
	end
end

function windows_temperature()
	local data					= read_config()
	local weather_status		= weather_status()
	local temperature_inside	= temperature_inside()
	local windows_auto			= read_or_init(SID_SELF,"WindowsStatusAuto",SELF, 0)
	local action				= "keep"
	
	if weather_status.rain == true or data.status >= 20 or weather_status.wind_speed > WIND_SPEED_LIMIT then
		luup.log("[MyHome] Closing all windows (weather)")
		action = "close"
	elseif data.lock_windows == 0 then
		-- check temperature and wind
		if (temperature_inside >= TEMPERATURE.MAX and temperature_inside > (weather_status.temperature - 1) and windows_auto == 0) then
			luup.log("[MyHome] Opening all windows (temperature)")
			action = "open"
		elseif (windows_auto == 1 and temperature_inside < (TEMPERATURE.MAX-1)) then
			luup.log("[MyHome] Closing all windows (temperature)")
			action = "close"
		end
	end
	
	if action == "close" then
		luup.variable_set(SID_SELF,"WindowsStatusAuto",0,SELF)
		windows_close()
	elseif action == "open" then
		luup.variable_set(SID_SELF,"WindowsStatusAuto",1,SELF)
		windows_open()
	end
end

function windows_close()
	for index,device in pairs(devices_search({ ["class"] = "Window" })) do
		device_close(device)
	end
end

function windows_open()
	for index,device in pairs(devices_search({ ["class"] = "Window" })) do
		device_open(device)
	end
end

function thermostats_set(temperature)
	-- TODO handle thermostats
end

function thermostats_auto()
	-- TODO handle thermostats
end

function remote_call(action,params)
-- TODO notify alarm manager
--	local http		= require("socket.http")
--	local sha1		= require("sha1")
--	http.TIMEOUT	= 10
--	--message			= urlencode(message)
--	
--	-- TODO handle params
--	local url		= "https://" .. ALARM_SERVER .. "/alarm/" .. action .. "?message=" .. message .. "&time=" .. os.time()
--	local signature = sha1.hmac_sha1(ALARM_SECRET,url)
--
--	luup.log("[MyHome] Calling remote alarm " .. action)
--
--	local result, status = http.request{
--		url		= url,
--		method	= 'POST',
--		headers	= {
--			["X-HomelyAlarm-Signature"] = signature
--		}
--	}
--	
--	if status != 200 then
--		luup.log("[MyHome] Failed remote alarm " .. action .. " (" .. status .. ") : " .. result)
--	end
end

