module ("myutils", package.seeall);

blinds_time = 17 -- time for blinds to move fully up/down
presence_device = 60



function partial_blind(device,percentage)
    luup.log ("Moving blind " .. device .. " to " .. percentage .. "%")
    
    local blind_status = luup.variable_get("urn:upnp-org:serviceId:Dimming1","LoadLevelStatus",device);

    local blind_time = ( blind_status - percentage ) / 100 * blinds_time;

    luup.call_action("urn:upnp-org:serviceId:Dimming1", "SetLoadLevelTarget", { newLoadlevelTarget = percentage }, device)

    blind_time = math.floor(blind_time)
    if blind_time < 0 then
		blind_time = blind_time * -1
        luup.call_action("urn:upnp-org:serviceId:WindowCovering1", "Up", {}, device)
    elseif blind_time > 0 then
        luup.call_action("urn:upnp-org:serviceId:WindowCovering1", "Down", {}, device)
    end

    luup.log("TIMING: " .. device .." - ".. blind_time)

	luup.call_delay("stop_blind", blind_time, device)
end



function random_lights(devices)
    local presence_status = luup.variable_get("urn:schemas-upnp-org:device:VSwitch:1","Status",presence_device)
    local time = os.date('*t');
    
	if presence_status == 0 && luup.is_night() && time.hour < 23 then
		device_index 	= math.ceil(math.random(0,table.getn(devices)))
		device_on		= math.floor(math.random(10))
		device_time 	= math.floor(math.random(60,900))
		if (device_on == 1) then
			local device = devices[device_index]
			luup.call_action("urn:upnp-org:serviceId:SwitchPower1", "SetTarget", { newTargetValue = 1 }, device)
			luup.call_delay("turn_off_light", device_time, device)
		end
	end
end