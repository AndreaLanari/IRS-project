-- Variabili Globali
MAX_STEPS = 1200 -- Tmax = 120 s
MAX_VELOCITY = 16
L_AXIS = robot.wheels.axis_length
local vector = require "vector"
n_steps = 0
epuck_id= 0 -- durante l'init viene settato un id numerico per la comunicazione
black = 1 -- se è 1 vuol dire che il robot è stato creato sulla patch nera, quindi deve uscire e rientrare
color_friends = 0

-- Main Section -- 

function init()
	id_string = string.sub(robot.id,7)
	epuck_id = tonumber(id_string)
end

-- Funzione step che viene effettuata ad ogni iterazione
function step()
	wheel_v = {left = 0, right = 0}
	if n_steps < MAX_STEPS then
		n_steps = n_steps + 1
		--- sense --
		patch_color = groundCheck()
		friends = searchTheCloseFriend()
		prox = proxCheck()
		--- plan ---
			if patch_color ~= nil then
				 -- countFriends(patch_color)
				if patch_color == 2 then
					wheel_v = whitePatch()
				else
					wheel_v = blackPatch()	
				end
			else
				if friends[1] ~= 0 then
					if friends[2] == 2 then
						wheel_v = whiteHelp(friends[1])
					else
						if prox then
							wheel_v = collisionAvoid()
						else 
							wheel_v = blackHelp(friends[1])
						end
					end
				else
					if prox then
						wheel_v = collisionAvoid()
					else 
						wheel_v = randomWalk()
					end
				end
			end
		--]]
		
		wheel_v = velocity_check(wheel_v)
		--- act ---
		robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
	end
	--[[if (first == 1) then
		pointCounter()
	end	]]	
end

-- funzione di reset
function reset()
	n_steps = 0
	epuck_id= 0
	black = 1
end

-- funzione di destroy
function destroy()
	-- nothing to do
end

--Sezione Comportamenti --
-- Comportamento se si trova nella patch bianca
function whitePatch()
	v={left = 0 ,right = 0}
	callFriends(2)
	v = collisionAvoid()

	return v
end

-- Comportamento se si trova nella patch nera
function blackPatch(friends)
	v={left = 0, right = 0}
	if black == 1 then 
		v = randomWalk()
	end
	callFriends(1)
	return v
end

-- Comportamento se sta andando verso la patch bianca
function whiteHelp(i)
	wheel_v = {left = 0, right = 0}
	x_angle = robot.range_and_bearing[i].horizontal_bearing
	r = robot.range_and_bearing[i].range
	wheel_v.left = r - L_AXIS*x_angle/2
	wheel_v.right = r + L_AXIS*x_angle/2
	return wheel_v
end

-- COmportamento se sta andando verso la patch nera
function blackHelp(i)
	wheel_v = { left = 0, right = 0}
	x_angle = robot.range_and_bearing[i].horizontal_bearing + 0.175
	r = robot.range_and_bearing[i].range
	wheel_v.left = r - L_AXIS*x_angle/2
	wheel_v.right = r + L_AXIS*x_angle/2
	return wheel_v
end

-- Movimento casuale
function randomWalk()
	wheel_v={left = 0, right = 0}
	wheel_v.left = robot.random.uniform(0,MAX_VELOCITY)
	wheel_v.right = robot.random.uniform(0,MAX_VELOCITY)
	return wheel_v
end

--Sezione Helper --

--Funzione che chiama gli altri robot verso la zona designata
function callFriends(color,n_friends)
		robot.range_and_bearing.set_data(color,1)
end

-- Funzione che controlla il colore del terreno sotto il robot
function groundCheck()
	if robot.ground[1] == 0 then 
		return 1
	end
	if robot.ground[1] == 1 then
		black = 0
		return 2
	end
	black = 0
	return nil
end

-- Cerca l'amico più vicino a patto che il gruppo di amici nella patch non sia già completo
function searchTheCloseFriend()
	tmp = 80
	info = {0,0}
	for i=1,#robot.range_and_bearing do
		for j=1,2 do
			if ( robot.range_and_bearing[i].data[j] > 0 and robot.range_and_bearing[i].range < tmp and color_friends < 10 ) then
				tmp = robot.range_and_bearing[i].range
				info[1] = i
				info[2] = j
			end
		end
	end
	return info
end


-- Funzione che trasforma la forza passata in velocità delle ruote
function speedFromForce(f)
	wheel_v = {left = 0, right = 0}
	pol_v = vector.cart_to_polar(f)
    wheel_v.left = pol_v.length - L_AXIS*pol_v.angle/2
    wheel_v.right = pol_v.length + L_AXIS*pol_v.angle/2

    return wheel_v
end

-- Funzione che valuta se ci sono oggetti vicino al robot
function proxCheck()
	for i=1,#robot.proximity do
		if robot.proximity[i].value > 0 then
			return true
		end
	end
	return false
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

-- Comportamento se ha un oggetto vicino
function collisionAvoid()
	avoidanceForce = { x = 0, y = 0}
    for i = 1,#robot.proximity do
        v = -20 * robot.proximity[i].value 
        a = robot.proximity[i].angle

        sensorForce = {x = v * math.cos(a), y = v * math.sin(a)}
        avoidanceForce.x = avoidanceForce.x + sensorForce.x
        avoidanceForce.y = avoidanceForce.y + sensorForce.y
    end
    return speedFromForce(avoidanceForce)
end