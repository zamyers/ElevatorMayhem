using Plots
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
	ob_pressed_above
	ob_pressed_below
	ob_at_current_floor
end

struct ApproxDistanceElevatorState
	current_floor
	inner_buttons
	above_dist
	below_dist
	ob_at_current_floor
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

function shortest_distance_heuristic(problem::ElevatorProblem, state::ElevatorState, destination)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons
	d = destination
	floors = 1:num_f

	#Combine both outer up and down buttons into a single outer button vector
	ob = map(((u, d),) -> 1*((u == 1) || (d == 1)), collect(zip(uob,dob)))
	
	#If the floor you are on, currently has an inner button pressed, then you should stop and let that person out
	if f in filter(((fl, b),) ->(b==1), collect(zip(floors, ib)))
		return 0, d
	end

	#If you are on the floor that is your destination, stop and let that person out
	if f==d
		d = 0
		return 0, d
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
		if d > f
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
	above = any([fl for (fl, b) for zip(floors, ob) if b > 0].>f)
	below = any([fl for (fl, b) for zip(floors, ob) if b > 0].<f)
	at_current_floor = any([fl for (fl, b) for zip(floors, ob) if b > 0]==f)

	return ApproxAboveBelowElevatorState(f, ib, above, below, at_current_floor)
end

function ob_distance_project(problem::ElevatorProblem, state::ElevatorState)
	num_f, prob_obp = problem.number_of_floors, problem.outer_button_press_probability
	f, ib, uob, dob = state.current_floor, state.inner_buttons, state.up_outer_buttons, state.down_outer_buttons

	floors = 1:num_f

	ob = map(((u, d),) -> 1*((u == 1) || (d == 1)), collect(zip(uob,dob)))

	viable_floors_above = [fl for (fl, b) in zip(floors, uob) if (b > 0) && (fl > f)]
	above_floor_dist = abs(viable_floors_above[1]-f)

	viable_floors_below = [fl for (fl, b) in zip(floors, dob) if (b > 0) && (fl < f)]
	below_floor_dist = abs(viable_floors_below[1]-f)

	at_current_floor = any([fl for (fl, b) for zip(floors, ob) if b > 0]==f)

	return ApproxDistanceElevatorState(f, ib, above_floor_dist, below_floor_dist, at_current_floor)
end

animate_this = false

EP  = ElevatorProblem(5, 0.025)
elv = ElevatorState(EP)
a   = 0
d   = 0
tot_r = 0

if animate_this
	anim = @animate for ii in 1:100
		global elv, r = elevator_simulator(EP, elv, a)
		global a, d = shortest_distance_heuristic(EP, elv, d)
		global tot_r += r
		println(a, " ", d, " ", elv, " ", tot_r, " ", r)
		plot_elevator_state(EP, elv, d, a)
	end 
	gif(anim, "test.gif", fps=3)
else
	for ii in 1:1000
		global elv, r = elevator_simulator(EP, elv, a)
		global a, d = shortest_distance_heuristic(EP, elv, d)
		global tot_r += r
		println("cycle: ", ii)
		println("--------------------------")
		println("action: ", a+2, "\nstate: ", elv, "\nreward: ", r)
		println("--------------------------")
	end
end



