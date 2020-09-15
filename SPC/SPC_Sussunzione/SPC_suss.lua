-- Import
local vector = require "vector"
local luabt = require('luabt')

-- Variabili Globali
MAX_STEPS = 1200 -- Tmax = 120 s
MAX_VELOCITY = 16
L_AXIS = robot.wheels.axis_length
n_steps = 0
black = true
zone = 0
friend_info = {index = 0, color = 0}
subs_bt = nil

-- Funzione init, parte solo al lancio del controller
function init()
   log("Start "..robot.id)
   subs_bt = luabt.create(root_node)
end

-- Funzione step viene usata ogni time-tick
function step()
	wheel_v = {left = 0, right = 0}
   if n_steps < MAX_STEPS then
      subs_bt()
      n_steps = n_steps +1
   end
end

-- Funzione reset
function reset()
	n_steps = 0
   black = true
   subs_bt = luabt.create(root_node)
end

-- Funzione destroy
function destroy()
	-- non usato
end

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

-- Funzione che cerca il robot più vicino che si trova in una zona bianca o nera
function closerFriend()
   log("prova closer")
	min_range = 80
	for i=1,#robot.range_and_bearing do
			if robot.range_and_bearing[i].data[1] > 0 then
				if robot.range_and_bearing[i].range < min_range then
					min_range =robot.range_and_bearing[i].range
					friend_info.index = i
					friend_info.color = robot.range_and_bearing[i].data[1]
				end
			end
	end
	return false,true
end

-- Funzione che trasforma il vettore movimento in velocità
function speedFromForce(f)
	wheel_v = {left = 0, right = 0}
   wheel_v.left = f.length - L_AXIS*f.angle/2
   wheel_v.right = f.length + L_AXIS*f.angle/2
   return wheel_v
end

-- Sezione Albero Comportamentali --
---------------------------------

-- Sezione condition leaves-- 

-- Nodo condizione per il colore del pavimento(zone)
function zoneDetect()
   if robot.ground[1] == 0 then
      zone = 1
		return false,true
	end
   if robot.ground[1] == 1 then
      zone = 2
		black = false 
		return false,true
	end
   black = false
   zone = 0
	return false,false
end

-- Nodo condizione sensori di vicinanza
function proxCheck()
	for i=1,#robot.proximity do
		if robot.proximity[i].value > 0 then
			return false,true
		end
	end
	return false,false
end

-- Nodo condizione per la  comunicazione
function helperCheck()
	for i=1,#robot.range_and_bearing do
         if robot.range_and_bearing[i].data[1] > 0 then
				return false,true
			end
	end
	return false,false
end

-- Sezione action leaves 

--Nodo azione che invia l'informazioni riguardo la zona nel quale è il robot
function sendInfo()
   robot.range_and_bearing.set_data(1,zone)
   return false,false
end

-- Nodo azione che fa muovere verso il robottino più vicino che si trova in una zona non grigia
function friendHelp()
   v = {length = 0, angle = 0}
	delta = 0 
	if (friend_info.color == 1) then  -- se il robottino aiutante è sulla zona nera, non gli vado addosso ma mi muovo a 10 gradi da lui
		delta = 0.2
   end
   v.length = robot.range_and_bearing[friend_info.index].range
	v.angle = robot.range_and_bearing[friend_info.index].horizontal_bearing  + delta
   wheel_v =  speedFromForce(v)
   wheel_v = velocity_check(wheel_v)
   robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
   return true
end

-- Nodo azione che evita le collissioni
function collisionAvoid()
   avoidanceForce = { length = 0, angle = 0}
	for i = 1,#robot.proximity do
		v = {length = 0, angle = 0}
		v.length = - 20 * robot.proximity[i].value 
		v.angle = robot.proximity[i].angle
		avoidanceForce = vector.vec2_polar_sum(avoidanceForce, v)
	end
   wheel_v =speedFromForce(avoidanceForce)
   wheel_v = velocity_check(wheel_v)
   robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
   return true
end

-- Nodo azione per il Random Walk
function randomWalk()
	wheel_v={left = 0, right = 0}
	wheel_v.left = robot.random.uniform(0,MAX_VELOCITY)
   wheel_v.right = robot.random.uniform(0,MAX_VELOCITY)
   robot.wheels.set_velocity(wheel_v.left,wheel_v.right)
   return true
end

-- Nodo azione che ferma il robot
function stop_walking()
   robot.wheels.set_velocity(0,0)
   return true
end

-- Inner Node

-- Nodo: collision avoid
proximity_node = {
   type = "sequence",
   children = {
      proxCheck,
      collisionAvoid
   }
}

-- Nodo: vado verso un robot che si trova sulla zona bianca
whiteFriend_node = {
   type = "sequence",
   children = {
      function()
         log("whitefriend")
         if friend_info.color == 2 then
            return false,true
         else
            return false,false
         end
      end,
      friendHelp
   }

}

-- Nodo: vado verso un robot che si trova nella zona nera
blackFriend_node = {
   type = "sequence",
   children = {
      function()
         log("blackfriend")
         if friend_info.color == 1 then
            return false,true
         else
            return false,false
         end
      end,
      {
         type = "selector",
         children = 
         {
            proximity_node,
            friendHelp
         }
      }
      
   }
}

--Nodo: seleziona verso quale robot si sta andando
selectFriend_node = {
   type = "selector",
   children = 
   {
      whiteFriend_node,
      blackFriend_node
   }
}

-- Nodo: se c'è qualcuno che lo sta "aiutando", va verso l'aiutante più vicino
helped_node = {
   type = "sequence",
   children = {
      helperCheck,
      closerFriend,
      selectFriend_node,

   }
}

-- Nodo: zona bianca
white_node = {
   type = "sequence",
   children = {
      zoneDetect,
      function ()
         if zone == 2 then
            return false,true
         else
            return false, false
         end
      end,
      { 
         type = "selector",
         children = {
         proximity_node,
         stop_walking
         }
      }
   }
}

-- Nodo : zona nera
black_node = {
   type = "sequence",
   children = {
         zoneDetect,
         function ()
            if zone == 1 then
               return false,true
            else
               return false, false
            end
         end,
      {
         type = "selector",
         children = {
            {
               type = "sequence",
               children = {
                  function()
                     if black then
                        return false,true
                     else
                        return false,false
                     end
                     
                  end,
                  randomWalk
               }},
            stop_walking
         }
      }     
   }
}

-- Nodo: è quello principale che permette la selezione di un comportamento
root_node = {
   type = "selector",
   children = {
      sendInfo,
      white_node,
      black_node,
      helped_node,
      proximity_node,
      randomWalk
   }
}

