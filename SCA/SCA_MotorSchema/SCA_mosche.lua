-- Variabili Globali
MAX_STEPS = 1200 -- Tmax = 120 s
MAX_VELOCITY = 16
L_AXIS = robot.wheels.axis_length
local vector = require "vector"
n_steps = 0
burned = 1 -- 0 se non ha raggiunto il muro luce, 1 se sta andando verso il riparo

-- Main Section -- 

function init()
	log("Start "..robot.id)
end

-- Funzione step che viene effettuata ad ogni iterazione
function step()
	v_tot = {x = 0, y=0}
	if n_steps < MAX_STEPS then
		log(robot.id)
		n_steps = n_steps + 1
		sendInfo()
		friend_v = friendSchema()
		light_v = lightSchema()
		prox_v = proxSchema()
		rand_v = randomForce()
		v_tot.x = friend_v.x + light_v.x + prox_v.x + rand_v.x
		v_tot.y = friend_v.y + light_v.y + prox_v.y + rand_v.y
		wheel_v = speedFromForce(v_tot)
		wheel_v = velocity_check(wheel_v)
		robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
	end
end

-- Funzione di reset
function reset()
	n_steps = 0 
	burned = 1
end

-- Funzione di reset
function destroy()
   -- //
end

--Sezione Schemas --

-- Comportamento: Parcheggiami; se sono salvo mi posizione in un punto della shelter dove non creo disagio
function proxSchema()
	f= { x = 0, y = 0}
	sign = groundCheck()
	if proxCheck() then
		for i = 1,#robot.proximity do
			v = sign * 10 * robot.proximity[i].value 
			a = robot.proximity[i].angle
			sensorForce = {x = v * math.cos(a), y = v * math.sin(a)}
			force.x = force.x + sensorForce.x
			force.y = force.y + sensorForce.y
		end
	else
		if sign == 1 then
			force.y = 15
		end
	end
	return force
end


-- Comportamento: Se il robot si trova al sicuro, comunica agl'altri la sua posizione
function sendInfo()
	robot.range_and_bearing.set_data(1,groundCheck())
end

-- MotorSchema legato alla comunicazione con gli altri robot
function friendSchema()
	v_cart = {x = 0, y = 0}
	v_polar = {length = 0, angle = 0}
	helper = friendsCheck()
	if (helper ~= 0 and burned == -1 and groundCheck() == -1) then 
		v_polar.length = robot.range_and_bearing[helper].range
		v_polar.angle = robot.range_and_bearing[helper].horizontal_bearing
		v_cart = vector.polar_to_cart(v_polar)
	end
	return v_cart
end

-- Motorschema legato alla luce
function lightSchema()
	force = {x =0,y=0}
		if groundCheck() == -1 and lightCheck() > 0.2 then 
			sign = burned	
			ph_v = phototaxis(sign)
		end
		return force
end

-- Motorschema legato ad una forza casuale (si attiva solo se tutte le altre forze sono nulle)
function randomForce()
	v = {x = 0, y = 0}
	if (groundCheck() == -1 and lightCheck() <= 0.2) then 
		v.x= robot.random.uniform(-40 ,40)
		v.y = robot.random.uniform(-40 ,40)
	end
	return v
end

--Sezione perception --
-------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------

-- Funzione che valuta se ci sono oggetti vicino al robot
function proxCheck()
	for i=1,#robot.proximity do
		if robot.proximity[i].value > 0.2 then
			return true
		end
	end
	return false
end

-- Funzione che percepisce l'intensità della luce
function lightCheck()
	tot_light = 0
	for i=1,#robot.light do
		tot_light = tot_light + robot.light[i]
	end
	log(tot_light)
	if tot_light < 0.7 then
		burned = 1
	end
	if tot_light > 4.5 then
		burned = -1
	end
	return tot_light
end

-- Funzione per il check del colore del terreno
function groundCheck()
	if robot.ground[1] == 1 then 
		return 1
	end
	return -1
end

-- Funzione per il check di presenza di amici arrivati al riparo
function friendsCheck()
	for i=1,#robot.range_and_bearing do
		if robot.range_and_bearing[i].data[1] == 1 then
			return i
		end
	end
	return 0
end

-- Sezione helper -- 
---------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------------------

-- Funzione che calcola la il vettore direzione per la fototassi se sign = 1, per l'anti fototassi se sign = -1
function phototaxis(sign)
	force = {x = 0, y = 0}
	for i = 1,#robot.light do
		v = 10 * robot.light[i]
        a = (i-1)*0.79
        sensorForce = {x = v * math.cos(a), y = v * math.sin(a)}
        force.x = force.x + sensorForce.x
		force.y = force.y + sensorForce.y
	end
	force.y = force.y*sign
	return force
end 

-- Funzione che trasforma la forza passata in velocità delle ruote
function speedFromForce(f)
	wheel_v = {left = 0, right = 0}
	pol_v = vector.cart_to_polar(f)
    wheel_v.left = pol_v.length - L_AXIS*pol_v.angle/2
    wheel_v.right = pol_v.length + L_AXIS*pol_v.angle/2

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

-- Funzione che salvaguardia la velocità massima del robot
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
