-- Import
local vector = require "vector"
local luabt = require('luabt')

-- Variabili Globali
MAX_STEPS = 1200 -- Tmax = 120 s
MAX_VELOCITY = 16
L_AXIS = robot.wheels.axis_length
n_steps = 0
burned = 1
ground = 0
helper = 0
tot_light = 0
subs_bt = nil

-- Funzione init, parte solo al lancio del controller
function init()
   log("Start "..robot.id)
   subs_bt = luabt.create(root_node)
end

-- Funzione step viene usata ogni time-tick
function step()
   if n_steps < MAX_STEPS then
      subs_bt()
      n_steps = n_steps +1
   end
end

-- Funzione reset
function reset()
	n_steps = 0
   subs_bt = luabt.create(root_node)
   n_steps = 0
   burned = 1
   ground = 0
   helper = 0
   tot_light = 0

end

-- Funzione destroy
function destroy()
	-- non usato
end


-- Sezione Albero Comportamentali --
---------------------------------

-- Condition Leaves

-- Nodo condizione per la prossimità
function proxCheck()
	for i=1,#robot.proximity do
      if robot.proximity[i].value > 0.2 then
			return false,true
		end
   end
	return false,false
end

-- Nodo condizione per la luce
function lightCheck()
   tot_light = 0
	for i=1,#robot.light do
		tot_light = tot_light + robot.light[i]
	end
	if tot_light < 0.7 then
		burned = 1
	end
	if tot_light > 4.5 then
		burned = -1
	end
   if tot_light > 0.2 then
      return false,true
   end
   return false,false
end

-- Nodo condizione per il colore del pavimento
function groundCheck()
	if robot.ground[1] == 1 then 
      ground = 1
      return false,true
   end
   ground = -1
	return false,false
end

-- Nodo condizione per la comunicazione con gli altri robot
function friendCheck()
	for i=1,#robot.range_and_bearing do
		if robot.range_and_bearing[i].data[1] == 1 then
         helper = i
         return false,true
		end
	end
	return false,false
end

-- Action leaves

--Nodo azione per l'invio della comunicazione
function sendInfo()
   robot.range_and_bearing.set_data(1,ground)
   return false,false
end

-- Nodo azione per il movimento verso altri robot al riparo
function friendHelpBh()
   f = {length = 0, angle = 0}
	f.length = robot.range_and_bearing[helper].range
   f.angle = robot.range_and_bearing[helper].horizontal_bearing
   wheel_v = speedFromForce(f)
   set_velocity(wheel_v)
	return true
end

-- Nodo azione per la phototassi/antiphototassi, in base al segno
function lightBh()
	sign = burned	
	force = {x = 0, y = 0}
	for i = 1,#robot.light do
		v = 20 * robot.light[i]
      a = (i-1)*0.79
      sensorForce = {x = v * math.cos(a), y = v * math.sin(a)}
      force.x = force.x + sensorForce.x
		force.y = force.y + sensorForce.y
	end
   force.y = force.y*sign
   wheel_v = speedFromForce(vector.cart_to_polar(force))
   set_velocity(wheel_v)
	return true
end

-- Nodo azione per le ricerca/evitamento di collissioni, in base al segno
function proxBh()
   sign = ground
   force = { x = 0, y = 0 }
	for i = 1,#robot.proximity do
			v = sign*10 * robot.proximity[i].value 
			a = robot.proximity[i].angle
			sensorForce = {x = v * math.cos(a), y = v * math.sin(a)}
			force.x = force.x + sensorForce.x
			force.y = force.y + sensorForce.y
   end
   wheel_v = speedFromForce(vector.cart_to_polar(force))
   set_velocity(wheel_v)
	return true
end

-- Nodo azione che fa muovere il robot casualmente
function randomBh()
   wheel_v = {left= 0, right = 0}
	wheel_v.left = robot.random.uniform(0, MAX_VELOCITY)
	wheel_v.right = robot.random.uniform(0, MAX_VELOCITY)
   robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
	return true
end

-- Inner Nodes

-- Nodo: se vedo sufficiente luce, seguo/scappo dalla luce
light_node = {
   type = "sequence",
   children = {
      lightCheck,
      lightBh
   }
}


-- Nodo: se qualcuno mi comunica la posizione del riparo, 
-- cerco di raggiungerlo
help_node = {
   type = "sequence",
   children = {
      friendCheck,
      friendHelpBh
   }
}

-- Nodo: collision avoid se non è al sicuro, 
-- attractive force se è al sicuro
proximity_node = {
   type = "sequence",
   children = {
      proxCheck,
      proxBh
   }
}

-- Nodo: se si trova nel rifugio, posiziona al meglio il robot
park_node = {
   type = "selector",
   children = {
      proximity_node,
      function()
         robot.wheels.set_velocity(MAX_VELOCITY,MAX_VELOCITY)
         return true
      end
   }
}

-- Nodo : zona sicura
shelter_node = {
   type = "sequence",
   children = {
      groundCheck,
      park_node
   }
}

-- Radice
root_node = {
   type = "selector",
   children = {
      sendInfo,
      shelter_node,
      proximity_node,
      help_node,
      light_node,
      randomWalk
   }
}



-- Sezione Helper --

-- Funzione che pone dei limiti di velocità alle ruote, nel momento in cui si eccede la velocità massima consentita
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

-- Funzione che controlla se la velocità settata è accettabile,
-- per poi muovere gli attuatori
function set_velocity(wheel_v)
   wheel_v = velocity_check(wheel_v)
   robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
end

-- Funzione che trasforma la forza passata in velocità delle ruote
function speedFromForce(f)
	wheel_v = {left = 0, right = 0}
    wheel_v.left = f.length - L_AXIS*f.angle/2
    wheel_v.right = f.length + L_AXIS*f.angle/2
    return wheel_v
end


