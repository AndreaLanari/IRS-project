-- Variabili Globali
MAX_STEPS = 1200 -- Tmax = 120 s
MAX_VELOCITY = 16
L_AXIS = robot.wheels.axis_length
local vector = require "vector"
n_steps = 0
safe = 0 -- 0 se non è dentro il riparo, 1 se è al riparo
burned = 0 -- 0 se non è abbastanza vicino alla luce, 1 se lo è

-- Main Section -- 

function init()
	log("Start "..robot.id)
end

-- Funzione step che viene effettuata ad ogni iterazione
function step()
	wheel_v = {left = 0, right = 0}
	if n_steps < MAX_STEPS then
		log(robot.id)
		n_steps = n_steps + 1
		--- sense ---
		amISafe()
		light = lightCheck()
		prox = proxCheck()
		helped = listenToFriends()
		--- plan ---
		if safe == 1 then
			helpFriendsBh()
			log("Aiuto i miei amici a trovare il riparo")
			parkMe()
		else
			if safe == 0 then
				if prox then
					log("Comportamento: cerco di evitare i vari ostacoli")
					wheel_v = collisionAvoidBh()
				else
					if (burned == 0) then
						if (light) then
							log( "Comportamento: Vado verso la luce")
							wheel_v = lightFinderBh()
						else
							log ( "Comportamento: Vago, non ho riferimenti")
							wheel_v = randomWalkBh()
						end
					else
						if helped ~= 0 then
							log("Comportamento: mi stanno indicando il rifugio!")
							wheel_v = findingShelterBh(helped)
						else
							log ( "Comportamento: sono bruciato scappo al riparo")
							wheel_v = burnedBh()
						end
					end
				end
			end
			
		end
		--- act ---
		wheel_v = velocity_check(wheel_v)
		robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
	end
end

-- Funzione reset
function reset()
	n_steps = 0
	safe = 0 
	burned = 0
end

--  Funzione destroy
function destroy()
   -- non implementata
end

--Sezione Comportamenti -- 

-- Comportamento: Parcheggiami; se sono salvo mi posizione in un punto della shelter dove non creo disagio
function parkMe()
	force = { x = 0, y = 0}
	if proxCheck() then
		for i = 1,#robot.proximity do
			v = 8 * robot.proximity[i].value 
			a = robot.proximity[i].angle

			sensorForce = {x = v * math.cos(a), y = v * math.sin(a)}
			force.x = force.x + sensorForce.x
			force.y = force.y + sensorForce.y
		end
	else
		force.y = 15
	end
	return speedFromForce(force)
end

-- Comportamento: Controlla se è al sicuro, se lo è setta il suo stato in safe = 1
function amISafe()
	if robot.ground[1] == 1 then 
		log("Sono Salvo")
		safe = 1
		return true
	end
	return false
end

-- Comportamento: ascolta gli altri robot intorno a lui
function listenToFriends()
	for i=1,#robot.range_and_bearing do
		if robot.range_and_bearing[i].data[1] == 1 then
			return i
		end
	end
	return 0
end

-- Comportamento: Se il robot si trova al sicuro, comunica agl'altri la sua posizione
function helpFriendsBh()
	robot.range_and_bearing.set_data(1,1)
end

-- Comportamento: Se il suo stato di burn = 1, va in dirzione opposta alla luce alla ricerca del riparo
function findingShelterBh(i)
	v = {left = 0, right = 0}
	x_angle = robot.range_and_bearing[i].horizontal_bearing
	r = robot.range_and_bearing[i].range
	v.left = r - L_AXIS*x_angle/2
	v.right = r + L_AXIS*x_angle/2
	return v
end

-- Comportamento: antifototassi che si attiva nel momento in cui è arrivato abbastanza vicino alla luce
function burnedBh()
	ph_v = phototaxis(-1)
	force = ph_v[1]
	return speedFromForce(force)
end

-- Comportamento: phototassi
function lightFinderBh()
	ph_v = phototaxis(1)
	force = ph_v[1]
	tot_light = ph_v[2]
	if (tot_light > 4.5) then
		burned = 1
		force = {x = 0, y = 0}	
	end
	return speedFromForce(force)
end

-- Comportamento: Se non "vede la luce", il robot parte con una random walk.
function randomWalkBh()
	move_vector = {left = 0, right = 0}
	move_vector.left= robot.random.uniform(0 ,MAX_VELOCITY)
	move_vector.right = robot.random.uniform(0 ,MAX_VELOCITY)
	return move_vector
end

-- Comportamento: avoid collision
function collisionAvoidBh()
	avoidanceForce = { x = 0, y = 0}
    for i = 1,#robot.proximity do
        v = -15 * robot.proximity[i].value 
        a = robot.proximity[i].angle

        sensorForce = {x = v * math.cos(a), y = v * math.sin(a)}
        avoidanceForce.x = avoidanceForce.x + sensorForce.x
        avoidanceForce.y = avoidanceForce.y + sensorForce.y
    end
    return speedFromForce(avoidanceForce)
end


-- Sezione helper -- 
---------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------------------

-- Funzione che calcola la il vettore direzione per la fototassi se delta = 1, per l'anti fototassi se delte = -1
function phototaxis(delta)
	tot_light = 0
	force = {x = 0, y = 0}
	for i = 1,#robot.light do
		v = 15 * robot.light[i]
        a = (i-1)*0.79
        sensorForce = {x = v * math.cos(a), y = v * math.sin(a)}
        force.x = force.x + sensorForce.x
		force.y = force.y + sensorForce.y
		tot_light = robot.light[i]+ tot_light
	end
	force.y = force.y * delta
	return {force,tot_light}
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
		if robot.proximity[i].value > 0.2 then
			return true
		end
	end
	return false
end

-- Funzione che valuta se il robot "vede" la luce
function lightCheck()
	tot_light = 0
	flag = false
	for i=1,#robot.light do
		tot_light = tot_light + robot.light[i]
		if robot.light[i] > 0.0 then
			flag = true	
		end
	end
	if tot_light < 0.7 then 
		burned = 0
	end
	return flag
end

-- Funzione che controlla se non vengono superati i limiti fisici di velocità
function velocity_check(v)
	if (math.abs(v.right) > MAX_VELOCITY or math.abs(v.left) > MAX_VELOCITY)  then
		if (math.abs(v.right) >= math.abs(v.left)) then
			v.left = MAX_VELOCITY*v.left /v.right
			v.right = MAX_VELOCITY
			
		else 
			v.right = MAX_VELOCITY*v.right/v.left
			v.left  = MAX_VELOCITY
		end
	end
	return v
end