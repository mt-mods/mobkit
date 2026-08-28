local abs = math.abs
local pi = math.pi
local ceil = math.ceil
local random = math.random
local sqrt = math.sqrt
local max = math.max

local abr = tonumber(core.get_mapgen_setting("active_block_range")) or 3

local neighbors = {
	{ x = 1, z = 0 },
	{ x = 1, z = 1 },
	{ x = 0, z = 1 },
	{ x = -1, z = 1 },
	{ x = -1, z = 0 },
	{ x = -1, z = -1 },
	{ x = 0, z = -1 },
	{ x = 1, z = -1 },
}

function mobkit.dir2neighbor(dir)
	dir.y = 0
	dir = vector.round(vector.normalize(dir))
	for k, v in ipairs(neighbors) do
		if v.x == dir.x and v.z == dir.z then
			return k
		end
	end
	return 1
end

function mobkit.neighbor_shift(neighbor, shift) -- int shift: minus is left, plus is right
	return (8 + neighbor + shift - 1) % 8 + 1
end

function mobkit.is_neighbor_node_reachable(self, neighbor) -- todo: take either number or pos
	local offset = neighbors[neighbor]
	local pos = mobkit.get_stand_pos(self)
	local tpos = mobkit.get_node_pos(mobkit.pos_shift(pos, offset))
	local recursteps = ceil(self.jump_height) + 1
	local height, liquidflag = mobkit.get_terrain_height(tpos, recursteps)

	if height and abs(height - pos.y) <= self.jump_height then
		tpos.y = height
		height = height - pos.y

		-- don't cut corners
		if neighbor % 2 == 0 then -- diagonal neighbors are even
			local n2 = neighbor - 1 -- left neighbor never < 0
			offset = neighbors[n2]
			local t2 = mobkit.get_node_pos(mobkit.pos_shift(pos, offset))
			local h2 = mobkit.get_terrain_height(t2, recursteps)
			if h2 and h2 - pos.y > 0.02 then
				return
			end
			n2 = (neighbor + 1) % 8 -- right neighbor
			offset = neighbors[n2]
			t2 = mobkit.get_node_pos(mobkit.pos_shift(pos, offset))
			h2 = mobkit.get_terrain_height(t2, recursteps)
			if h2 and h2 - pos.y > 0.02 then
				return
			end
		end

		-- check headroom
		local foot = mobkit.get_node_pos(pos)
		local head = mobkit.get_node_pos(vector.new(pos.x, pos.y + self.height, pos.z))
		local target_head = mobkit.get_node_pos(vector.new(tpos.x, tpos.y + self.height, tpos.z))
		local target_high = vector.new(target_head.x, max(target_head.y, head.y), target_head.z)
		if target_high.y > foot.y then -- If head in next node above, else no point checking headroom
			local target_foot = mobkit.get_node_pos(tpos)
			local low = vector.new(foot.x, target_foot.y + 1, foot.z)

			local nodes = mobkit.get_nodes_in_area(low, target_high, true)

			for p, node in pairs(nodes) do
				if node.name == "ignore" or node.walkable then
					return
				end
			end
		end

		return height, tpos, liquidflag
	else
		return
	end
end

function mobkit.get_next_waypoint(self, tpos)
	local pos = mobkit.get_stand_pos(self)
	local dir = vector.direction(pos, tpos)
	local neighbor = mobkit.dir2neighbor(dir)
	local function update_pos_history(self2, pos2)
		table.insert(self2.pos_history, 1, pos2)
		if #self2.pos_history > 2 then
			table.remove(self2.pos_history, #self2.pos_history)
		end
	end
	local nogopos = self.pos_history[2]

	local height, pos2, liquidflag = mobkit.is_neighbor_node_reachable(self, neighbor)
	if height and not liquidflag and not (nogopos and mobkit.isnear2d(pos2, nogopos, 0.1)) then
		local heightl = mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, -1))
		if heightl and abs(heightl - height) < 0.001 then
			local heightr = mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, 1))
			if heightr and abs(heightr - height) < 0.001 then
				dir.y = 0
				local dirn = vector.normalize(dir)
				local npos = mobkit.get_node_pos(mobkit.pos_shift(pos, neighbors[neighbor]))
				local factor = abs(dirn.x) > abs(dirn.z) and abs(npos.x - pos.x) or abs(npos.z - pos.z)
				pos2 = mobkit.pos_shift(pos, { x = dirn.x * factor, z = dirn.z * factor })
			end
		end
		update_pos_history(self, pos2)
		return height, pos2
	else
		for i = 1, 3 do
			-- scan left
			local liq
			height, pos2, liq =
				mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, -i * self.path_dir))
			if height and not liq and not (nogopos and mobkit.isnear2d(pos2, nogopos, 0.1)) then
				update_pos_history(self, pos2)
				return height, pos2
			end
			-- scan right
			height, pos2, liq =
				mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, i * self.path_dir))
			if height and not liq and not (nogopos and mobkit.isnear2d(pos2, nogopos, 0.1)) then
				update_pos_history(self, pos2)
				return height, pos2
			end
		end
		--scan rear
		height, pos2, liquidflag = mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, 4))
		if height and not liquidflag and not (nogopos and mobkit.isnear2d(pos2, nogopos, 0.1)) then
			update_pos_history(self, pos2)
			return height, pos2
		end
	end
	-- stuck condition here
	table.remove(self.pos_history, 2)
	self.path_dir = self.path_dir * -1 -- subtle change in pathfinding
end

function mobkit.get_next_waypoint_fast(self, tpos, nogopos)
	local pos = mobkit.get_stand_pos(self)
	local dir = vector.direction(pos, tpos)
	local neighbor = mobkit.dir2neighbor(dir)
	local height, pos2, liquidflag = mobkit.is_neighbor_node_reachable(self, neighbor)

	if height and not liquidflag then
		local fast = false
		local heightl = mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, -1))
		if heightl and abs(heightl - height) < 0.001 then
			local heightr = mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, 1))
			if heightr and abs(heightr - height) < 0.001 then
				fast = true
				dir.y = 0
				local dirn = vector.normalize(dir)
				local npos = mobkit.get_node_pos(mobkit.pos_shift(pos, neighbors[neighbor]))
				local factor = abs(dirn.x) > abs(dirn.z) and abs(npos.x - pos.x) or abs(npos.z - pos.z)
				pos2 = mobkit.pos_shift(pos, { x = dirn.x * factor, z = dirn.z * factor })
			end
		end
		return height, pos2, fast
	else
		for i = 1, 4 do
			-- scan left
			local liq
			height, pos2, liq = mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, -i))
			if height and not liq then
				return height, pos2
			end
			-- scan right
			height, pos2, liq = mobkit.is_neighbor_node_reachable(self, mobkit.neighbor_shift(neighbor, i))
			if height and not liq then
				return height, pos2
			end
		end
	end
end

function mobkit.goto_next_waypoint(self, tpos)
	local height, pos2 = mobkit.get_next_waypoint(self, tpos)

	if not height then
		return false
	end

	if height <= 0.01 then
		local yaw = self.object:get_yaw()
		local tyaw = core.dir_to_yaw(vector.direction(self.object:get_pos(), pos2))
		if abs(tyaw - yaw) > 1 then
			mobkit.lq_turn2pos(self, pos2)
		end
		mobkit.lq_dumbwalk(self, pos2)
	else
		mobkit.lq_turn2pos(self, pos2)
		mobkit.lq_dumbjump(self, height)
	end
	return true
end

----------------------------
-- BEHAVIORS
----------------------------
-- LOW LEVEL QUEUE FUNCTIONS
----------------------------

function mobkit.lq_turn2pos(self, tpos)
	local func = function(self2)
		local pos = self2.object:get_pos()
		return mobkit.turn2yaw(self2, core.dir_to_yaw(vector.direction(pos, tpos)))
	end
	mobkit.queue_low(self, func)
end

function mobkit.lq_idle(self, duration, anim)
	anim = anim or "stand"
	local init = true
	local func = function(self2)
		if init then
			mobkit.animate(self2, anim)
			init = false
		end
		duration = duration - self.dtime
		if duration <= 0 then
			return true
		end
	end
	mobkit.queue_low(self, func)
end

function mobkit.lq_dumbwalk(self, dest, speed_factor)
	local timer = 3 -- failsafe
	speed_factor = speed_factor or 1
	local func = function(self2)
		mobkit.animate(self2, "walk")
		timer = timer - self2.dtime
		if timer < 0 then
			return true
		end

		local pos = mobkit.get_stand_pos(self2)
		local y = self2.object:get_velocity().y

		if mobkit.is_there_yet2d(pos, core.yaw_to_dir(self2.object:get_yaw()), dest) then
			--		if mobkit.isnear2d(pos,dest,0.25) then
			if not self2.isonground or abs(dest.y - pos.y) > 0.1 then -- prevent uncontrolled fall when velocity too high
				--			if abs(dest.y-pos.y) > 0.1 then	-- isonground too slow for speeds > 4
				self2.object:set_velocity({ x = 0, y = y, z = 0 })
			end
			return true
		end

		if self2.isonground then
			local dir =
				vector.normalize(vector.direction({ x = pos.x, y = 0, z = pos.z }, { x = dest.x, y = 0, z = dest.z }))
			dir = vector.multiply(dir, self2.max_speed * speed_factor)
			--			self.object:set_yaw(core.dir_to_yaw(dir))
			mobkit.turn2yaw(self, core.dir_to_yaw(dir))
			dir.y = y
			self2.object:set_velocity(dir)
		end
	end
	mobkit.queue_low(self, func)
end

-- initial velocity for jump height h, v= a*sqrt(h*2/a) ,add 20%
function mobkit.lq_dumbjump(self, height, anim)
	anim = anim or "stand"
	local jump = true
	local func = function(self2)
		local yaw = self2.object:get_yaw()
		if self2.isonground then
			if jump then
				mobkit.animate(self, anim)
				local dir = core.yaw_to_dir(yaw)
				dir.y = -mobkit.gravity * sqrt((height + 0.35) * 2 / -mobkit.gravity)
				self2.object:set_velocity(dir)
				jump = false
			else -- the eagle has landed
				return true
			end
		else
			local dir = core.yaw_to_dir(yaw)
			local vel = self.object:get_velocity()
			if self2.lastvelocity.y < 0.9 then
				dir = vector.multiply(dir, 3)
			end
			dir.y = vel.y
			self2.object:set_velocity(dir)
		end
	end
	mobkit.queue_low(self, func)
end

function mobkit.lq_jumpout(self)
	local phase = 1
	local func = function(self2)
		local vel = self2.object:get_velocity()
		if phase == 1 then
			vel.y = vel.y + 5
			self2.object:set_velocity(vel)
			phase = 2
		else
			if vel.y < 0 then
				return true
			end
			local dir = core.yaw_to_dir(self.object:get_yaw())
			dir.y = vel.y
			self2.object:set_velocity(dir)
		end
	end
	mobkit.queue_low(self, func)
end

function mobkit.lq_freejump(self)
	local phase = 1
	local func = function(self2)
		local vel = self2.object:get_velocity()
		if phase == 1 then
			vel.y = vel.y + 6
			self2.object:set_velocity(vel)
			phase = 2
		else
			if vel.y <= 0.01 then
				return true
			end
			local dir = core.yaw_to_dir(self2.object:get_yaw())
			dir.y = vel.y
			self2.object:set_velocity(dir)
		end
	end
	mobkit.queue_low(self, func)
end

function mobkit.lq_jumpattack(self, height, target)
	local init = true
	local tgtbox = target:get_properties().collisionbox
	local func = function(self2)
		if not mobkit.is_alive(target) then
			return true
		end
		if self2.isonground then
			if init then -- collision bug workaround
				local dir = core.yaw_to_dir(self2.object:get_yaw())
				dir = vector.multiply(dir, 6)
				dir.y = -mobkit.gravity * sqrt(height * 2 / -mobkit.gravity)
				self2.object:set_velocity(dir)
				mobkit.make_sound(self2, "charge")
				init = false
			else
				mobkit.lq_idle(self2, 0.3)
				return true
			end
		else
			local tgtpos = target:get_pos()
			local pos = self2.object:get_pos()
			-- calculate attack spot
			local yaw = self2.object:get_yaw()
			local dir = core.yaw_to_dir(yaw)
			local apos = mobkit.pos_translate2d(pos, yaw, self2.attack.range)

			if mobkit.is_pos_in_box(apos, tgtpos, tgtbox) then --bite
				target:punch(self2.object, 1, self2.attack)
				-- bounce off
				local vy = self2.object:get_velocity().y
				self2.object:set_velocity({ x = dir.x * -3, y = vy, z = dir.z * -3 })
				-- play attack sound if defined
				mobkit.make_sound(self2, "attack")
				return true
			end
		end
	end
	mobkit.queue_low(self, func)
end

function mobkit.lq_fallover(self)
	local zrot = 0
	local init = true
	local func = function(self2)
		if init then
			local vel = self2.object:get_velocity()
			self2.object:set_velocity(mobkit.pos_shift(vel, { y = 1 }))
			mobkit.animate(self2, "stand")
			init = false
		end
		zrot = zrot + pi * 0.05
		local rot = self2.object:get_rotation()
		self2.object:set_rotation({ x = rot.x, y = rot.y, z = zrot })
		if zrot >= pi * 0.5 then
			return true
		end
	end
	mobkit.queue_low(self, func)
end
-----------------------------
-- HIGH LEVEL QUEUE FUNCTIONS
-----------------------------

function mobkit.dumbstep(self, height, tpos, speed_factor, idle_duration)
	if height <= 0.001 then
		mobkit.lq_turn2pos(self, tpos)
		mobkit.lq_dumbwalk(self, tpos, speed_factor)
	else
		mobkit.lq_turn2pos(self, tpos)
		mobkit.lq_dumbjump(self, height)
	end
	idle_duration = idle_duration or 6
	mobkit.lq_idle(self, random(ceil(idle_duration * 0.5), idle_duration))
end

function mobkit.hq_roam(self, prty)
	local func = function(self2)
		if mobkit.is_queue_empty_low(self) and self.isonground then
			local neighbor = random(8)

			local height, tpos, liquidflag = mobkit.is_neighbor_node_reachable(self, neighbor)
			if height and not liquidflag then
				mobkit.dumbstep(self2, height, tpos, 0.3)
			end
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_follow0(self, tgtobj) -- probably delete this one
	local func = function(self2)
		if not tgtobj then
			return true
		end
		if mobkit.is_queue_empty_low(self2) and self2.isonground then
			local pos = mobkit.get_stand_pos(self2)
			local opos = tgtobj:get_pos()
			if vector.distance(pos, opos) > 3 then
				local neighbor = mobkit.dir2neighbor(vector.direction(pos, opos))
				if not neighbor then
					return true
				end --temp debug
				local height, tpos = mobkit.is_neighbor_node_reachable(self2, neighbor)
				if height then
					mobkit.dumbstep(self2, height, tpos)
				else
					for i = 1, 4 do --scan left
						height, tpos = mobkit.is_neighbor_node_reachable(self2, (8 + neighbor - i - 1) % 8 + 1)
						if height then
							mobkit.dumbstep(self2, height, tpos)
							break
						end --scan right
						height, tpos = mobkit.is_neighbor_node_reachable(self2, (neighbor + i - 1) % 8 + 1)
						if height then
							mobkit.dumbstep(self2, height, tpos)
							break
						end
					end
				end
			else
				mobkit.lq_idle(self2, 1)
			end
		end
	end
	mobkit.queue_high(self, func, 0)
end

function mobkit.hq_follow(self, prty, tgtobj)
	local func = function(self2)
		if not mobkit.is_alive(tgtobj) then
			return true
		end
		if mobkit.is_queue_empty_low(self2) and self2.isonground then
			local pos = mobkit.get_stand_pos(self2)
			local opos = tgtobj:get_pos()
			if vector.distance(pos, opos) > 3 then
				mobkit.goto_next_waypoint(self2, opos)
			else
				mobkit.lq_idle(self2, 1)
			end
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_goto(self, prty, tpos)
	local func = function(self2)
		if mobkit.is_queue_empty_low(self2) and self.isonground then
			local pos = mobkit.get_stand_pos(self2)
			if vector.distance(pos, tpos) > 3 then
				mobkit.goto_next_waypoint(self2, tpos)
			else
				return true
			end
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_runfrom(self, prty, tgtobj)
	local init = true
	local timer = 6
	local func = function(self2)
		if not mobkit.is_alive(tgtobj) then
			return true
		end
		if init then
			timer = timer - self2.dtime
			if timer <= 0 or vector.distance(self2.object:get_pos(), tgtobj:get_pos()) < 8 then
				mobkit.make_sound(self2, "scared")
				init = false
			end
			return
		end

		if mobkit.is_queue_empty_low(self2) and self2.isonground then
			local pos = mobkit.get_stand_pos(self2)
			local opos = tgtobj:get_pos()
			if vector.distance(pos, opos) < self2.view_range * 1.1 then
				local tpos = { x = 2 * pos.x - opos.x, y = opos.y, z = 2 * pos.z - opos.z }
				mobkit.goto_next_waypoint(self2, tpos)
			else
				self2.object:set_velocity({ x = 0, y = 0, z = 0 })
				return true
			end
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_hunt(self, prty, tgtobj)
	local func = function(self2)
		if not mobkit.is_alive(tgtobj) then
			return true
		end
		if mobkit.is_queue_empty_low(self2) and self2.isonground then
			local pos = mobkit.get_stand_pos(self2)
			local opos = tgtobj:get_pos()
			local dist = vector.distance(pos, opos)
			if dist > self2.view_range then
				return true
			elseif dist > 3 then
				mobkit.goto_next_waypoint(self2, opos)
			else
				mobkit.hq_attack(self2, prty + 1, tgtobj)
			end
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_warn(self, prty, tgtobj)
	local timer = 0
	local tgttime = 0
	local init = true
	local func = function(self2)
		if not mobkit.is_alive(tgtobj) then
			return true
		end
		if init then
			mobkit.animate(self2, "stand")
			init = false
		end
		local pos = mobkit.get_stand_pos(self2)
		local opos = tgtobj:get_pos()
		local dist = vector.distance(pos, opos)

		if dist > 11 then
			return true
		elseif dist < 4 or timer > 12 then -- too close man
			--			mobkit.clear_queue_high(self)
			mobkit.remember(self2, "hate", tgtobj:get_player_name())
			mobkit.hq_hunt(self2, prty + 1, tgtobj) -- priority
		else
			timer = timer + self2.dtime
			if mobkit.is_queue_empty_low(self2) then
				mobkit.lq_turn2pos(self2, opos)
			end
			-- make noise in random intervals
			if timer > tgttime then
				mobkit.make_sound(self2, "warn")
				-- if self.sounds and self.sounds.warn then
				-- core.sound_play(self.sounds.warn, {object=self.object})
				-- end
				tgttime = timer + 1.1 + random() * 1.5
			end
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_die(self)
	local timer = 5
	local start = true
	local func = function(self2)
		if start then
			mobkit.lq_fallover(self2)
			self2.logic = function(self3) end -- brain dead as well
			start = false
		end
		timer = timer - self.dtime
		if timer < 0 then
			self2.object:remove()
		end
	end
	mobkit.queue_high(self, func, 100)
end

function mobkit.hq_attack(self, prty, tgtobj)
	local func = function(self2)
		if not mobkit.is_alive(tgtobj) then
			return true
		end
		if mobkit.is_queue_empty_low(self2) then
			local pos = mobkit.get_stand_pos(self2)
			--			local tpos = tgtobj:get_pos()
			local tpos = mobkit.get_stand_pos(tgtobj)
			local dist = vector.distance(pos, tpos)
			if dist > 3 then
				return true
			else
				mobkit.lq_turn2pos(self, tpos)
				local midpoint = mobkit.get_box_height(tgtobj) * 0.5
				if tpos.y + midpoint > pos.y then
					mobkit.lq_jumpattack(self, tpos.y + midpoint - pos.y, tgtobj)
				else
					mobkit.lq_dumbwalk(self2, mobkit.pos_shift(tpos, { x = random() - 0.5, z = random() - 0.5 }))
				end
			end
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_liquid_recovery(self, prty) -- scan for nearest land
	local radius = 1
	local yaw = 0
	local func = function(self2)
		if not self2.isinliquid then
			return true
		end
		local pos = self2.object:get_pos()
		local vec = core.yaw_to_dir(yaw)
		local pos2 = mobkit.pos_shift(pos, vector.multiply(vec, radius))
		local height, liquidflag = mobkit.get_terrain_height(pos2)
		if height and not liquidflag then
			mobkit.hq_swimto(self2, prty, pos2)
			return true
		end
		yaw = yaw + pi * 0.25
		if yaw > 2 * pi then
			yaw = 0
			radius = radius + 1
			if radius > self2.view_range then
				self2.hp = 0
				return true
			end
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_swimto(self, prty, tpos)
	local box = self.object:get_properties().collisionbox
	local cols = {}
	local func = function(self2)
		if not self2.isinliquid then
			if self2.isonground then
				return true
			end
			return false
		end

		local pos = mobkit.get_stand_pos(self2)
		local y = self2.object:get_velocity().y
		local pos2d = { x = pos.x, y = tpos.y, z = pos.z }
		local dir = vector.normalize(vector.direction(pos2d, tpos))
		local yaw = core.dir_to_yaw(dir)

		if mobkit.timer(self2, 1) then
			cols = mobkit.get_box_displace_cols(pos, box, dir, 1)
			for _, p in ipairs(cols[1]) do
				p.y = pos.y
				local h = mobkit.get_terrain_height(p)
				if h and h > pos.y and self2.isinliquid then
					mobkit.lq_freejump(self2)
					break
				end
			end
		elseif mobkit.turn2yaw(self2, yaw) then
			dir.y = y
			self2.object:set_velocity(dir)
		end
	end
	mobkit.queue_high(self, func, prty)
end

---------------------
-- AQUATIC
---------------------

-- MACROS
local function aqua_radar_dumb(pos, yaw, range, reverse)
	range = range or 4

	local function okpos(p)
		local node = mobkit.nodeatpos(p)
		if node then
			if node.drawtype == "liquid" then
				local nodeu = mobkit.nodeatpos(mobkit.pos_shift(p, { y = 1 }))
				local noded = mobkit.nodeatpos(mobkit.pos_shift(p, { y = -1 }))
				if (nodeu and nodeu.drawtype == "liquid") or (noded and noded.drawtype == "liquid") then
					return true
				else
					return false
				end
			else
				local h = mobkit.get_terrain_height(p)
				if h then
					local node2 = mobkit.nodeatpos({ x = p.x, y = h + 1.99, z = p.z })
					if node2 and node2.drawtype == "liquid" then
						return true, h
					end
				else
					return false
				end
			end
		else
			return false
		end
	end

	local fpos = mobkit.pos_translate2d(pos, yaw, range)
	local ok, h = okpos(fpos)
	if not ok then
		local ffrom, fto, fstep
		if reverse then
			ffrom, fto, fstep = 3, 1, -1
		else
			ffrom, fto, fstep = 1, 3, 1
		end
		for i = ffrom, fto, fstep do
			ok, h = okpos(mobkit.pos_translate2d(pos, yaw + i, range))
			if ok then
				return yaw + i, h
			end
			ok, h = okpos(mobkit.pos_translate2d(pos, yaw - i, range))
			if ok then
				return yaw - i, h
			end
		end
		return yaw + pi, h
	else
		return yaw, h
	end
end

function mobkit.is_in_deep(target)
	if not target then
		return false
	end
	local nodepos = mobkit.get_stand_pos(target)
	local node1 = mobkit.nodeatpos(nodepos)
	nodepos.y = nodepos.y + 1
	local node2 = mobkit.nodeatpos(nodepos)
	nodepos.y = nodepos.y - 2
	local node3 = mobkit.nodeatpos(nodepos)
	if
		node1
		and node2
		and node3
		and node1.drawtype == "liquid"
		and (node2.drawtype == "liquid" or node3.drawtype == "liquid")
	then
		return true
	end
end

-- HQ behaviors

function mobkit.hq_aqua_roam(self, prty, speed)
	local tyaw = self.object:get_yaw()
	local init = true
	local prvscanpos = { x = 0, y = 0, z = 0 }
	local center = self.object:get_pos()
	local func = function(self2)
		if init then
			mobkit.animate(self2, "def")
			init = false
		end
		local pos = mobkit.get_stand_pos(self2)
		local yaw = self2.object:get_yaw()
		local scanpos = mobkit.get_node_pos(mobkit.pos_translate2d(pos, yaw, speed))
		if not vector.equals(prvscanpos, scanpos) then
			prvscanpos = scanpos
			local nyaw, height = aqua_radar_dumb(pos, yaw, speed, true)
			if height and height > pos.y then
				local vel = self2.object:get_velocity()
				vel.y = vel.y + 1
				self2.object:set_velocity(vel)
			end
			if yaw ~= nyaw then
				tyaw = nyaw
				mobkit.hq_aqua_turn(self2, prty + 1, tyaw, speed)
				return
			end
		end
		if mobkit.timer(self2, 1) then
			if vector.distance(pos, center) > abr * 16 * 0.5 then
				tyaw = core.dir_to_yaw(
					vector.direction(
						pos,
						{ x = center.x + random() * 10 - 5, y = center.y, z = center.z + random() * 10 - 5 }
					)
				)
			else
				if random(10) >= 9 then
					tyaw = tyaw + random() * pi - pi * 0.5
				end
			end
		end

		mobkit.turn2yaw(self2, tyaw, 3)
		--		local yaw = self.object:get_yaw()
		mobkit.go_forward_horizontal(self2, speed)
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_aqua_turn(self, prty, tyaw, speed)
	local func = function(self2)
		local finished = mobkit.turn2yaw(self2, tyaw)
		--		local yaw = self.object:get_yaw()
		mobkit.go_forward_horizontal(self2, speed)
		if finished then
			return true
		end
	end
	mobkit.queue_high(self, func, prty)
end

function mobkit.hq_aqua_attack(self, prty, tgtobj, speed)
	local tyaw = 0
	local prvscanpos = { x = 0, y = 0, z = 0 }
	local init = true
	local tgtbox = tgtobj:get_properties().collisionbox
	local func = function(self2)
		if not mobkit.is_alive(tgtobj) then
			return true
		end
		if init then
			mobkit.animate(self2, "fast")
			mobkit.make_sound(self2, "attack")
			init = false
		end
		local pos = mobkit.get_stand_pos(self2)
		local yaw = self2.object:get_yaw()
		local scanpos = mobkit.get_node_pos(mobkit.pos_translate2d(pos, yaw, speed))
		if not vector.equals(prvscanpos, scanpos) then
			prvscanpos = scanpos
			local nyaw, height = aqua_radar_dumb(pos, yaw, speed * 0.5)
			if height and height > pos.y then
				local vel = self2.object:get_velocity()
				vel.y = vel.y + 1
				self2.object:set_velocity(vel)
			end
			if yaw ~= nyaw then
				tyaw = nyaw
				mobkit.hq_aqua_turn(self2, prty + 1, tyaw, speed)
				return
			end
		end

		local tpos = tgtobj:get_pos()
		tyaw = core.dir_to_yaw(vector.direction(pos, tpos))
		mobkit.turn2yaw(self2, tyaw, 3)
		yaw = self2.object:get_yaw()
		if mobkit.timer(self2, 1) then
			if not mobkit.is_in_deep(tgtobj) then
				return true
			end
			local vel = self2.object:get_velocity()
			if tpos.y > pos.y + 0.5 then
				self2.object:set_velocity({ x = vel.x, y = vel.y + 0.5, z = vel.z })
			elseif tpos.y < pos.y - 0.5 then
				self2.object:set_velocity({ x = vel.x, y = vel.y - 0.5, z = vel.z })
			end
		end
		if mobkit.is_pos_in_box(mobkit.pos_translate2d(pos, yaw, self2.attack.range), tpos, tgtbox) then --bite
			tgtobj:punch(self2.object, 1, self2.attack)
			mobkit.hq_aqua_turn(self2, prty, yaw - pi, speed)
			return true
		end
		mobkit.go_forward_horizontal(self2, speed)
	end
	mobkit.queue_high(self, func, prty)
end
