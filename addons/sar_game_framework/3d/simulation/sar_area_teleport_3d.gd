extends Area3D
class_name SarAreaTeleport3D

## Where a character's area detector is sent when it enters this area.
@export var target_node: Node3D = null


func _ready() -> void:
	area_entered.connect(_on_area_entered)
	area_exited.connect(_on_area_exited)


func _on_area_entered(p_area: Area3D) -> void:
	if target_node and p_area is SarCharacterSimulationAreaDetectorComponent3D:
		p_area.entered_teleport(self)


func _on_area_exited(p_area: Area3D) -> void:
	if p_area is SarCharacterSimulationAreaDetectorComponent3D:
		p_area.exited_teleport(self)
