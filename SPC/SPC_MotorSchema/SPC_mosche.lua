-- Variabili Globali
MAX_STEPS = 1200 -- Tmax = 120 s
MAX_VELOCITY = 16
L_AXIS = robot.wheels.axis_length
local vector = require "vector"
n_steps = 0
black = true

-- Main Section -- 
-- Funzioni base: init, step, reset, destroy -- 

-- Funzione init, parte solo al lancio del controller
function init()
	log("Start "..robot.id)
end

-- Funzione step viene usata ogni time-tick
function step()
	wheel_v = {left = 0, right = 0}
	if n_steps < MAX_STEPS then
		n_steps = n_steps + 1
		log(robot.id)
		sendInfo()
		v_tot = vector.vec2_polar_sum(vector.vec2_polar_sum (randomForce(),friendsHelp()),collisionAvoid())
		wheel_v = speedFromForce(v_tot)
		wheel_v = velocity_check(wheel_v)
		robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
	end	
end

-- Funzione reset
function reset()
	n_steps = 0
	black = true
end

-- Funzione destroy
function destroy()
	-- non usato
end

--Sezione Schemas --

-- Comportamento: manda informazioni ai compagni
function sendInfo()
	robot.range_and_bearing.set_data(1,zoneDetect())
end

-- Comportamento: Muoversi casualmente
function randomForce()
	v = {length = 0, angle = 0}
	if zoneDetect() == 0 and helperCheck()== false then 
		v.length = robot.random.uniform(0,30)
		v.angle = robot.random.uniform(-3.14,3.14)
	end
	if (black) then
		v.length = robot.random.uniform(0,30)
		v.angle = robot.random.uniform(-3.14,3.14)
	end
	return v
end

-- Comportamento: muoviti verso il robottino più vicino che si trova in una zona non grigia
function friendsHelp()
	v = {length = 0, angle = 0}
	if zoneDetect() == 0 and helperCheck() then
		friend = closedFriend()
		delta = 0 
		if (friend.color == 1) then  -- se il robottino aiutante è sulla zona nera, non gli vado addosso ma mi muovo a 10 gradi da lui
			delta = 0.2
		end
		v.length = robot.range_and_bearing[friend.index].range
		v.angle = robot.range_and_bearing[friend.index].horizontal_bearing  + delta
	end
	return v
end

-- Comportamento: collission avoid
function collisionAvoid()
	avoidanceForce = { length = 0, angle = 0}
	if proxCheck() and zoneDetect() ~= 1 then
		for i = 1,#robot.proximity do
			v = {length = 0, angle = 0}
			v.length = - 20 * robot.proximity[i].value 
			v.angle = robot.proximity[i].angle
			avoidanceForce = vector.vec2_polar_sum(avoidanceForce, v)
		end
	end
    return avoidanceForce
end

-- Perceptual Schema -- 

-- Controllo del sensore del terreno
function zoneDetect()
	if robot.ground[1] == 0 then
		return 1
	end
	if robot.ground[1] == 1 then
		black = false 
		return 2
	end
	black = false
	return 0
end

-- Controllo sensori di vicinanza
function proxCheck()
	for i=1,#robot.proximity do
		if robot.proximity[i].value > 0 then
			return true
		end
	end
	return false
end

-- Controllo di sensore di comunicazione
function helperCheck()
	for i=1,#robot.range_and_bearing do
			if robot.range_and_bearing[i].data[1] > 0 then
				return true
			end
	end
	return false
end


--Sezione Helper --
-----------------------------------------

-- Seleziona l'amico più vicino 
function closedFriend()
	min_range = 80
	robInfo = {index = 0, color = 0}
	for i=1,#robot.range_and_bearing do
			if robot.range_and_bearing[i].data[1] > 0 then
				if robot.range_and_bearing[i].range < min_range then
					min_range =robot.range_and_bearing[i].range
					robInfo.index = i
					robInfo.color = robot.range_and_bearing[i].data[1]
				end
			end
	end
	return robInfo
end

-- Funzione che trasforma la forza passata in velocità delle ruote
function speedFromForce(f)
	wheel_v = {left = 0, right = 0}
    wheel_v.left = f.length - L_AXIS*f.angle/2
    wheel_v.right = f.length + L_AXIS*f.angle/2

    return wheel_v
end

-- Funzione ricorsiva che permette di stampare le tabelle
function tprint (tbl, indent)
	if not indent then indent = 0 end
	for k, v in pairs(tbl) do
	  formatting = string.rep("  ", indent) .. k .. ": "
	  if type(v) == "table" then
		print(formatting)
		tprint(v, indent+1)
	  else
		print(formatting)
		print(v)
	  end
	end
end

-- Funzione che gestisce la velocità massima delle ruote
function velocity_check(v)
	if (math.abs(v.right) > MAX_VELOCITY or math.abs(v.left) > MAX_VELOCITY)  then
		if (math.abs(v.right) >= math.abs(v.left)) then
			v.left = 16*v.left /v.right
			v.right = 16
			
		else 
			v.right = 16*v.right/v.left
			v.left  = 16
		end
	end

	return v
end



