extends LBFGSBSolver

var t_pose : Array[Transform3D] # The T-pose skeleton
var current_pose : Array[Transform3D] # The current pose skeleton
var many_bone_ik: ManyBoneIK3D = null
var humanoid: SkeletonProfileHumanoid = SkeletonProfileHumanoid.new()

func _ready():
	var skeleton: Skeleton3D = %GeneralSkeleton
	t_pose = get_rest_pose(skeleton)
	current_pose = get_pose(skeleton)
	many_bone_ik = get_parent()

func _process(_delta):
	var skeleton: Skeleton3D = %GeneralSkeleton
	minimize_pose(skeleton)

func get_rest_pose(skeleton: Skeleton3D) -> Array:
	var pose: Array[Transform3D]
	for humanoid_i: int in range(humanoid.bone_size):
		var i: int = skeleton.find_bone(humanoid.get_bone_name(humanoid_i))
		if i == -1:
			continue
		var transform = skeleton.get_bone_global_rest(i)
		pose.append(transform)
	return pose

func get_pose(skeleton: Skeleton3D) -> Array:
	var pose: Array[Transform3D]
	for humanoid_i: int in range(humanoid.bone_size):
		var i: int = skeleton.find_bone(humanoid.get_bone_name(humanoid_i))
		if i == -1:
			continue
		var transform = skeleton.get_bone_global_pose(i)
		pose.append(transform)
	return pose

func _call_operator(x: Array, gradient: Array) -> float:
	var skeleton: Skeleton3D = %GeneralSkeleton
	var cost = 0.0
	for humanoid_i: int in range(humanoid.bone_size):
		var i: int = skeleton.find_bone(humanoid.get_bone_name(humanoid_i))
		if i == -1:
			continue
		# Convert the flat array back into basis and translation
		var b = Basis(Vector3(x[i], x[i+1], x[i+2]), Vector3(x[i+3], x[i+4], x[i+5]), Vector3(x[i+6], x[i+7], x[i+8]))
		var t = Vector3(x[i+9], x[i+10], x[i+11])

		# Calculate the difference in rotation and position for each joint
		var rot_diff: float = t_pose[i/12].basis.get_euler().distance_to(b.get_euler())
		var pos_diff: float = t_pose[i/12].origin.distance_to(t)

		# Get the current twist value for the joint
		var current_rotation_twist_from = x[i+12]
		var current_rotation_twist_range = x[i+13]

		# Add the squared differences to the cost
		cost += rot_diff * rot_diff + pos_diff * pos_diff + current_rotation_twist_from * current_rotation_twist_range + current_rotation_twist_from * current_rotation_twist_range

		# Update the gradient
		for j in range(12): # Since there are 12 elements in a pose (9 for rotation, 3 for translation)
			if j < 9: # For basis elements
				var row = int(j / 3)
				var col = j % 3
				gradient[i+j] = 2.0 * (x[i+j] - b[row][col])
			elif j < 11: # For twist elements
				gradient[i+j] = 2.0 * x[i+j]
			else: # For translation elements
				gradient[i+j] = 2.0 * (x[i+j] - t[j-9])
	return cost


func minimize_pose(skeleton: Skeleton3D):
	var x = [] # Start with the current pose
	var count: int
	for humanoid_i: int in range(humanoid.bone_size):
		var i: int = skeleton.find_bone(humanoid.get_bone_name(humanoid_i))
		if i == -1:
			continue
		count = count + 1
		var transform = skeleton.get_bone_global_pose(i).orthonormalized()
		x.append(transform.basis.x.x)
		x.append(transform.basis.x.y)
		x.append(transform.basis.x.z)
		x.append(transform.basis.y.x)
		x.append(transform.basis.y.y)
		x.append(transform.basis.y.z)
		x.append(transform.basis.z.x)
		x.append(transform.basis.z.y)
		x.append(transform.basis.z.z)
		x.append(transform.origin.x)
		x.append(transform.origin.y)
		x.append(transform.origin.z)

		# Add the current twist value for the joint
		var constraint_i: int = many_bone_ik.find_constraint(skeleton.get_bone_name(i))
		var current_twist = many_bone_ik.get_kusudama_twist(constraint_i)
		if constraint_i == -1:
			x.append(0)
			x.append(TAU)
			continue
		x.append(current_twist.x)
		x.append(current_twist.y)

	var fx = 0.0 # Initial function value
	var lower_bound = [] # Lower bound constraints
	var upper_bound = [] # Upper bound constraints
	var max_distance = 10
	
	# Set the bounds for each joint
	for i in range(x.size()):
		if i == -1:
			continue
		if i % 14 < 9: # basis values
			lower_bound.append(0)
			upper_bound.append(1)
		elif i % 14 < 11: # twist values
			lower_bound.append(0)
			upper_bound.append(TAU)
		else: # translation values
			lower_bound.append(-max_distance) 
			upper_bound.append(max_distance)

	# Call the minimize method
	var result: Array = minimize(x, fx, lower_bound, upper_bound)[1]
	if result.size() < count * 14:
		print("Error: Not enough elements in result array")
		return
	var bone_data = []

	var result_i: int = 0
	# Update the current pose with the result
	for humanoid_i: int in range(humanoid.bone_size):
		var i: int = skeleton.find_bone(humanoid.get_bone_name(humanoid_i))
		if i == -1:
			continue
		var min_twist = Vector2(result[result_i*14+12], result[result_i*14+13])
		var constraint_i: int = many_bone_ik.find_constraint(skeleton.get_bone_name(i))
		many_bone_ik.set_kusudama_twist(constraint_i, min_twist)
		result_i = result_i + 1
	print(result)
