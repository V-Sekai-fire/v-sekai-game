extends Area3D
class_name SarAreaZone3D

## The zone a character's area detector is in while it overlaps this area.
@export var zone: SarZone = null


func _ready() -> void:
	area_entered.connect(_on_area_entered)
	area_exited.connect(_on_area_exited)


func _on_area_entered(p_area: Area3D) -> void:
	if zone and p_area is SarCharacterSimulationAreaDetectorComponent3D:
		p_area.entered_zone(self)


func _on_area_exited(p_area: Area3D) -> void:
	if p_area is SarCharacterSimulationAreaDetectorComponent3D:
		p_area.exited_zone(self)
