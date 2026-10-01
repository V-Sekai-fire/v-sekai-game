# Overlap test for SarAreaZone3D and SarAreaTeleport3D:
#   godot --headless --path . --script res://tests/sar_areas/sar_area_test.gd
extends SceneTree

class EntityStub:
	extends Node3D
	func get_game_entity() -> Node3D:
		return self

class ParentStub:
	extends Node3D
	var entity := EntityStub.new()
	func get_game_entity_interface() -> Object:
		return entity

var _det: SarCharacterSimulationAreaDetectorComponent3D
var _zone_a: SarAreaZone3D
var _zone_b: SarAreaZone3D
var _tele: SarAreaTeleport3D
var _teleports := 0
var _n := 0
var _fails := 0

func _box(a: Area3D, size: float) -> void:
	var s := CollisionShape3D.new()
	var b := BoxShape3D.new()
	b.size = Vector3.ONE * size
	s.shape = b
	a.add_child(s)

func _check(what: String, ok: bool) -> void:
	print("%s %s" % ["ok  " if ok else "FAIL", what])
	if not ok:
		_fails += 1

func _initialize() -> void:
	var root3d := Node3D.new()
	root.add_child(root3d)
	var parent := ParentStub.new()
	root3d.add_child(parent)
	parent.add_child(parent.entity)
	_det = SarCharacterSimulationAreaDetectorComponent3D.new()
	_box(_det, 1.0)
	parent.add_child(_det)
	_det.teleported.connect(func(_o): _teleports += 1)
	var z := SarZone.new()
	z.priority = 3
	_zone_a = SarAreaZone3D.new()
	_zone_a.zone = z
	_box(_zone_a, 4.0)
	root3d.add_child(_zone_a)
	_zone_b = SarAreaZone3D.new()
	_box(_zone_b, 4.0)
	root3d.add_child(_zone_b)
	_tele = SarAreaTeleport3D.new()
	_tele.target_node = Node3D.new()
	root3d.add_child(_tele.target_node)
	_tele.target_node.position = Vector3(10, 0, 0)
	_box(_tele, 4.0)
	_tele.position = Vector3(0, 50, 0)
	root3d.add_child(_tele)

func _physics_process(_dt: float) -> bool:
	_n += 1
	if _n == 10:
		_check("a zone with a SarZone registers on overlap", _det._active_zones.has(_zone_a))
		_check("control: a zone with no SarZone does not register", not _det._active_zones.has(_zone_b))
		_check("the highest-priority zone is the overlapped one", _det.get_highest_priority_active_zone() == _zone_a.zone)
		_det.get_parent().position = Vector3(0, 50, 0)
	if _n == 20:
		_check("leaving the zone unregisters it", _det._active_zones.is_empty())
		_check("entering a teleport area emits teleported once", _teleports == 1)
		print("AREA-TEST %d failed" % _fails)
		quit(1 if _fails else 0)
	return false
