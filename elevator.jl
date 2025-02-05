using Plots
using DataStructures
default(show=true)

struct ElevatorProblem
	number_of_floors
	outer_button_press_probability
end

struct ElevatorState
	current_floor
	inner_buttons
	up_outer_buttons
	down_outer_buttons
end

struct ApproxAboveBelowElevatorState
	current_floor
	inner_buttons
	ob_pressed_below
	ob_at_current_floor
	ob_pressed_above
end

struct ApproxDistanceElevatorState
	current_floor
	inner_buttons
	below_dist
	at_current_floor
	above_dist
end

function circle(x, y, r)
	phi = LinRange(0, 2*pi, 100)
	return x .+ r*sin.(phi), y .+ r*cos.(phi)
end

ElevatorState(problem::ElevatorProblem) = ElevatorState(1, zeros(problem.number_of_floors), zeros(problem.number_of_floors), zeros(problem.number_of_floors))

function elevator_simulator(problem::ElevatorProblem, state::ElevatorState, action)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons
	r = 0.0

	#Move up, down or stay!
	f = f + action

	#if you reach the boundaries, you actually cant go up, so you stay
	if f > num_f
		action = 0
		f = num_f
	elseif f < 1 
		action = 0
		f = 1
	end

	#If you are staying, first check if you let someone out and get a reward
	#Then if there is any up outer button presses and you are below the top floor, randomly choose a new floor above you to go to
	#Do the same for the  down outer button presses
	if action == 0
		r = ib[f]
		ib[f] = 0

		if (uob[f] == 1) && (f < num_f)
			ib[f + rand(1:(num_f-f))] = 1
		end

		if (dob[f] == 1) && (f > 1)
			ib[f - rand(1:(f-1))] = 1
		end

		uob[f] = 0
		dob[f] = 0
	end
    # [1, 2, 3, 4], f=3 ->  [1, 2, 4]
	for jj in filter(fl -> (fl != f), 2:num_f)
		dob[jj] = 1*((dob[jj] == 1) || (rand() < prob_obp))
	end
	for jj in filter(fl -> (fl != f), 1:(num_f-1))
		uob[jj] = 1*((uob[jj] == 1) || (rand() < prob_obp))
	end

	return ElevatorState(f, ib, uob, dob), r
end




function first_come_first_serve_heuristic(problem::ElevatorProblem, state::ElevatorState, destination, inner_queue, outer_queue)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons
	d = destination
	floors = 1:num_f

	#Combine both outer up and down buttons into a single outer button vector
	ob = map(((u, d),) -> 1*((u == 1) || (d == 1)), collect(zip(uob,dob)))

	new_ib_q = Queue{Int}()
	for node in inner_queue
		if ib[node] == 1
			enqueue!(new_ib_q, node)
		end 
	end 

	for fl in floors 
		if (ib[fl] == 1) && !(fl in new_ib_q)
			enqueue!(new_ib_q, fl)
		end 
	end 

	inner_queue = deepcopy(new_ib_q)

	new_ob_q = Queue{Int}() 
	for node in outer_queue
		if ob[node] == 1
			enqueue!(new_ob_q, node)
		end 
	end 

	for fl in floors 
		if (ob[fl] == 1) && !(fl in new_ob_q)
			enqueue!(new_ob_q, fl)
		end 
	end 
	outer_queue = deepcopy(new_ob_q)
	
	if f==d 	#If you are on the floor that is your destination, stop and let that person out
		d = 0
		return 0, 0, inner_queue, outer_queue 
	end 
	#If the floor you are on, currently has an inner button pressed, then you should stop and let that person out




	#If there are no buttons pressed, just wait at your current floor
	if !(length(inner_queue) > 0) && !(length(outer_queue)>0)
		a = 0
		d = 0
		return a, d, inner_queue, outer_queue
	end

	#If you don't have a destination, first check if any of the internal buttons are pressed and go to the closest floor
	#If no internal buttons are pressed, check if any external buttons are pressed and go to the closest floor
	#Otherwise, generally head towards destination, picking up people who are also heading to destination
	if d==0
		if length(inner_queue)>0
			d = first(inner_queue)
		else
			d = first(outer_queue)
		end
		a = (d > f)*2 - 1
	else	
		if f in inner_queue 
			a = 0 
		elseif d > f
			if uob[f] == 1
				a = 0
			else
				a = 1
			end
		else 
			d < f
			if dob[f] == 1
				a = 0
			else 
				a = -1
			end
		end
	end


	return a, d, inner_queue, outer_queue
end


function shortest_distance_heuristic(problem::ElevatorProblem, state::ElevatorState, destination)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons
	d = destination
	floors = 1:num_f

	#Combine both outer up and down buttons into a single outer button vector
	ob = map(((u, d),) -> 1*((u == 1) || (d == 1)), collect(zip(uob,dob)))
	
#	#If the floor you are on, currently has an inner button pressed, then you should stop and let that person out
#	if f in [fl for (fl, b) in zip(floors, ib) if b > 0]
#		if f == d
#			d = 0 
#		end
#		return 0, d
#	end

	if f==d
		return 0, 0
	end

	#If there are no buttons pressed, just wait at your current floor
	if !(any(ib.>0) || any(ob.>0))
		a = 0
		d = 0
		return a, d
	end

	#If you don't have a destination, first check if any of the internal buttons are pressed and go to the closest floor
	#If no internal buttons are pressed, check if any external buttons are pressed and go to the closest floor
	#Otherwise, generally head towards destination, picking up people who are also heading to destination
	if d==0
		if any(ib.>0)
			viable_floors = [fl for (fl, b) in zip(floors, ib) if b > 0]
			d = viable_floors[argmin(abs.(viable_floors.-f))]
		else 
			viable_floors = [fl for (fl, b) in zip(floors, ob) if b > 0]
			d = viable_floors[argmin(abs.(viable_floors.-f))]
		end
		a = (d > f)*2 - 1
	else
		if f in [fl for (fl, b) in zip(floors, ib) if b > 0]
			a = 0
		elseif d > f
			if uob[f] == 1
				a = 0
			else
				a = 1
			end
		else 
			if dob[f] == 1
				a = 0
			else 
				a = -1
			end
		end
	end

	return a, d
end

function plot_elevator_state(problem::ElevatorProblem, state::ElevatorState)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons
	plot(xlims = [-0.5, 0.5 + 0.5], ylims=[-0.5, num_f+0.5])

	ib_radius = 1/(2*num_f)
	ib_offset = floor(num_f/2)

	uob_radius = 1/4
	uob_offset = 3/4

	dob_radius = 1/4
	dob_offset = 1/4

	for ii in 1:num_f
		if f == ii
			plot!(circle(0, ii, 0.5), xlims=[-0.5, 0.5 + 0.5], ylims=[0.5, num_f+0.5], seriestype=[:shape,], lw=0.5, c=:black, linecolor =:black, legend=false, fillalpha=0.5, aspect_ratio=1)
		else
			if ib[ii]  == 1
				plot!(circle(0, ii, 0.5), xlims=[-0.5, 0.5 + 0.5], ylims=[0.5, num_f+0.5], seriestype=[:shape,], lw=1, c=:blue, linecolor =:black, legend=false, fillalpha=0.5, aspect_ratio=1)
			else
				plot!(circle(0, ii, 0.5), xlims=[-0.5, 0.5 + 0.5], ylims=[0.5, num_f+0.5], seriestype=[:shape,], lw=1, c=:blue, linecolor =:black, legend=false, fillalpha=0.0, aspect_ratio=1)
			end
		end

		if uob[ii] == 1
			plot!(circle(0.75, (ii-0.5) + uob_offset, uob_radius), xlims=[-0.5, 0.5 + 0.5], ylims=[0.5, num_f+0.5], seriestype=[:shape,], lw=0.5, c=:red, linecolor =:black, legend=false, fillalpha=0.75, aspect_ratio=1)
		else
			plot!(circle(0.75, (ii-0.5) + uob_offset, uob_radius), xlims=[-0.5, 0.5 + 0.5], ylims=[0.5, num_f+0.5], seriestype=[:shape,], lw=0.5, c=:black, linecolor =:black, legend=false, fillalpha=0.0, aspect_ratio=1)
		end

		if dob[ii] == 1
			plot!(circle(0.75, (ii-0.5) + dob_offset, dob_radius), xlims=[-0.5, 0.5 + 0.5], ylims=[0.5, num_f+0.5], seriestype=[:shape,], lw=0.5, c=:red, linecolor =:black, legend=false, fillalpha=0.75, aspect_ratio=1)
		else
			plot!(circle(0.75, (ii-0.5) + dob_offset, dob_radius), xlims=[-0.5, 0.5 + 0.5], ylims=[0.5, num_f+0.5], seriestype=[:shape,], lw=0.5, c=:black, linecolor =:black, legend=false, fillalpha=0.0, aspect_ratio=1)
		end
	end
end

function plot_elevator_state(problem::ElevatorProblem, state::ElevatorState, destination, action)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons
	a = action
	d = destination

	action_labels = ["d", "s", "u"]

	plot_elevator_state(problem, state)
	if d > 0
		quiver!([0], [f], quiver=([0], [d-f]))
	end
	annotate!(0, f, text(action_labels[a+2], :white, :middle, 10))
end

function ob_abovebelow_project(problem::ElevatorProblem, state::ElevatorState)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons

	floors = 1:num_f

	ob = map(((u, d),) -> 1*((u == 1) || (d == 1)), collect(zip(uob,dob)))
	above = any([fl for (fl, b) in zip(floors, ob) if b > 0].>f)*1.0
	below = any([fl for (fl, b) in zip(floors, ob) if b > 0].<f)*1.0
	at_current_floor = any([fl for (fl, b) in zip(floors, ob) if b > 0].==f)*1.0

	return ApproxAboveBelowElevatorState(f, ib, below, at_current_floor, above)
end

function ob_distance_project(problem::ElevatorProblem, state::ElevatorState)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons

	floors = 1:num_f

	ob = map(((u, d),) -> 1*((u == 1) || (d == 1)), collect(zip(uob,dob)))

	viable_floors_above = [fl for (fl, b) in zip(floors, ob) if (b > 0) && (fl > f)]
	if length(viable_floors_above) > 0
		above_floor_dist = abs(viable_floors_above[1]-f)
	else
		above_floor_dist = 0
	end
	viable_floors_below = [fl for (fl, b) in zip(floors, ob) if (b > 0) && (fl < f)]
	if length(viable_floors_below) > 0
		below_floor_dist = abs(viable_floors_below[end]-f)
	else
		below_floor_dist = 0
	end

	at_current_floor = any([fl for (fl, b) in zip(floors, ob) if b > 0]==f)*1.0

	return ApproxDistanceElevatorState(f, ib, below_floor_dist, at_current_floor, above_floor_dist)
end

animate_this = false
Q_learning = false

num_f = 5

EP  = ElevatorProblem(num_f, 0.05)
elv = ElevatorState(EP)
a   = 0
d   = 0
inner_queue = Queue{Int}()
outer_queue = Queue{Int}()
tot_r = 0
	#Approx Distance
	#convert_state = LinearIndices((num_f, 2^(num_f), num_f, 2, num_f))
	#state_dimension = convert_state[num_f, 2^(num_f), num_f, 2,num_f]
		#Approx Up Down
convert_state = LinearIndices((num_f, 2^(num_f), 2, 2, 2))
state_dimension = convert_state[num_f, 2^(num_f), 2, 2,2]
Q_current = zeros(state_dimension,3)

α = 0.35
γ = 0.7
prev_a = 0
prev_s = 1

if animate_this && !Q_learning
	anim = @animate for ii in 1:100
		global elv, r = elevator_simulator(EP, elv, a)
		global a, d, inner_queue, outer_queue = first_come_first_serve_heuristic(EP, elv, d, inner_queue, outer_queue)
		#global a, d = shortest_distance_heuristic(EP, elv, d)
		global tot_r += r
		println(a, " ", d, " ", elv, " ", tot_r, " ", r)
		plot_elevator_state(EP, elv, d, a)
	end 
	gif(anim, "test.gif", fps=3)
else
	for ii in 1:500000
		global elv, r = elevator_simulator(EP, elv, a)
		#global a, d = shortest_distance_heuristic(EP, elv, d)
		global a, d, inner_queue, outer_queue = first_come_first_serve_heuristic(EP, elv, d, inner_queue, outer_queue)
		global tot_r += r

		if Q_learning 
			approx_elv = ob_abovebelow_project(EP, elv)
			f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons, approx_elv.ob_pressed_above, approx_elv.ob_at_current_floor, approx_elv.ob_pressed_below
			ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

			#approx_elv = ob_distance_project(EP, elv)
			#f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons,	approx_elv.above_dist, approx_elv.at_current_floor, approx_elv.below_dist
			#ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

			s = convert_state[f, ib_int + 1, Int(ab+1), Int(at+1), Int(bl+1)]

		#println(prev_s, " ", prev_a, " ",  r, " ", s)

			global Q_max = maximum(Q_current[s, :]) 
			global Q_current[prev_s, prev_a+2] = Q_current[prev_s, prev_a+2] + α*(r + γ*Q_max - Q_current[prev_s, prev_a+2])
			 #is determining the next a part of the exploration? can i just do that randomly 
			global prev_a = a
			global prev_s = s
		end


		#println(approx_elv)
		#println(convert_state[f, ib_int + 1, Int(ab+1), Int(at+1), Int(bl+1)])

		#println("cycle: ", ii)
		#println("--------------------------")
		#println("action: ", a+2, "\nstate: ", elv, "\nreward: ", r)
		#println("--------------------------")
	end
	println("Final Score: ", tot_r)
	for ii in 1:1500000
		global elv, r = elevator_simulator(EP, elv, a)
		global a = rand(1:3) - 2
		approx_elv = ob_abovebelow_project(EP, elv)
		f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons, approx_elv.ob_pressed_above, approx_elv.ob_at_current_floor, approx_elv.ob_pressed_below
		ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

		#approx_elv = ob_distance_project(EP, elv)
		#f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons,	approx_elv.above_dist, approx_elv.at_current_floor, approx_elv.below_dist
		#ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

		s = convert_state[f, ib_int + 1, Int(ab+1), Int(at+1), Int(bl+1)]

		#println(prev_s, " ", prev_a, " ",  r, " ", s)

		if Q_learning 
			global Q_max = maximum(Q_current[s, :]) 
			global Q_current[prev_s, prev_a+2] = Q_current[prev_s, prev_a+2] + α*(r + γ*Q_max - Q_current[prev_s, prev_a+2])
			 #is determining the next a part of the exploration? can i just do that randomly 
		end
		global tot_r += r

		global prev_a = a
		global prev_s = s
	end
end

println(Q_current)
Q_prev = deepcopy(Q_current)
readline()

println("Final Score: ", tot_r)
elv = ElevatorState(EP)
tot_r = 0
prev_f = 1
f_repeats = 0
use_Q = true

cheating = zeros((1000000), 1)

if Q_learning
	for ii in 1:1000000
		if use_Q
			approx_elv = ob_abovebelow_project(EP, elv)
			f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons, approx_elv.ob_pressed_above, approx_elv.ob_at_current_floor, approx_elv.ob_pressed_below
			ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])
			
			#approx_elv = ob_distance_project(EP, elv)
			#f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons,	approx_elv.above_dist, approx_elv.at_current_floor, approx_elv.below_dist
			#ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

			s = convert_state[f, ib_int + 1, Int(ab+1), Int(at+1), Int(bl+1)]

			global a = argmax(Q_current[s, :]) - 2
			if f_repeats > 3
				global a = rand(1:3) - 2
				global use_Q = false
				global d = 0
			end
			global elv, r = elevator_simulator(EP, elv, a)

			approx_elv = ob_abovebelow_project(EP, elv)
			f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons, approx_elv.ob_pressed_above, approx_elv.ob_at_current_floor, approx_elv.ob_pressed_below
			ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

			#approx_elv = ob_distance_project(EP, elv)
			#f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons,	approx_elv.above_dist, approx_elv.at_current_floor, approx_elv.below_dist
			#ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

			sp = convert_state[f, ib_int + 1, Int(ab+1), Int(at+1), Int(bl+1)]

			if f_repeats > 3
				global Q_max = maximum(Q_current[sp, :]) 
				global Q_current[s, a+2] = Q_current[s, a+2] + α*(r + γ*Q_max - Q_current[s, a+2])
			end
			global tot_r +=r
			#println("cycle: ", ii)
			#println("--------------------------")
			#println("action: ", a+2, "\nstate: ", elv, "\nreward: ", r)
			#println("--------------------------")
			if f == prev_f
				global f_repeats += 1
			else
				global f_repeats = 0
			end
			global prev_f = f
			global cheating[ii] = 0
		else
			global elv, r = elevator_simulator(EP, elv, a)
			global a, d = shortest_distance_heuristic(EP, elv, d)
			global tot_r += r
			global f_repeats = f_repeats - 1
			if f_repeats <= 1 #Some weird issue with how globals are updated!
				global use_Q = true
			end
			global cheating[ii] = 1
		end
	end
	println("Final Score: ", tot_r)
	println(sum(cheating)/1000000)
end
#println(Q_current -Q_prev)
if Q_learning && animate_this
	elv = ElevatorState(EP)
	tot_r = 0
	prev_f = 1
	f_repeats = 0
	use_Q = true
	anim = @animate for ii in 1:100
		if use_Q
			approx_elv = ob_abovebelow_project(EP, elv)
			f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons, approx_elv.ob_pressed_above, approx_elv.ob_at_current_floor, approx_elv.ob_pressed_below
			ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])
			
			#approx_elv = ob_distance_project(EP, elv)
			#f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons,	approx_elv.above_dist, approx_elv.at_current_floor, approx_elv.below_dist
			#ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

			s = convert_state[f, ib_int + 1, Int(ab+1), Int(at+1), Int(bl+1)]

			global a = argmax(Q_current[s, :]) - 2
			if f_repeats > 3
				global a = rand(1:3) - 2
				global use_Q = false
				global d = 0
			end
			global elv, r = elevator_simulator(EP, elv, a)

			approx_elv = ob_abovebelow_project(EP, elv)
			f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons, approx_elv.ob_pressed_above, approx_elv.ob_at_current_floor, approx_elv.ob_pressed_below
			ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

			#approx_elv = ob_distance_project(EP, elv)
			#f, ib, ab, at, bl = approx_elv.current_floor, approx_elv.inner_buttons,	approx_elv.above_dist, approx_elv.at_current_floor, approx_elv.below_dist
			#ib_int = sum([Int(2^(i-1) * b) for (i, b) in zip(1:num_f, ib)])

			sp = convert_state[f, ib_int + 1, Int(ab+1), Int(at+1), Int(bl+1)]

			if f_repeats > 3
				global Q_max = maximum(Q_current[sp, :]) 
				global Q_current[s, a+2] = Q_current[s, a+2] + α*(r + γ*Q_max - Q_current[s, a+2])
			end
			global tot_r +=r
			#println("cycle: ", ii)
			#println("--------------------------")
			#println("action: ", a+2, "\nstate: ", elv, "\nreward: ", r)
			#println("--------------------------")
			if f == prev_f
				global f_repeats += 1
			else
				global f_repeats = 0
			end
			global prev_f = f
		else
			global elv, r = elevator_simulator(EP, elv, a)
			global a, d = shortest_distance_heuristic(EP, elv, d)
			global tot_r += r
			global f_repeats = f_repeats - 1
			if f_repeats <= 1 #Some weird issue with how globals are updated!
				global use_Q = true
			end
		end
		plot_elevator_state(EP, elv, 0, a)
	end 
	gif(anim, "test.gif", fps=3)
end

