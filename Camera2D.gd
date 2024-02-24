extends Camera2D

@onready var main = $".."

@export var in_points: bool = false
@export var only_movement: bool = false

var mouse_pressed: bool = false

func _input(event) -> void:
	if event is InputEventMouseButton:
		mouse_pressed = event.is_pressed()
	check_movement(event)
	if only_movement:
		if Input.is_action_just_pressed("left_mouse"):
			main.OnClicked(get_global_mouse_position())
		return
	if event is InputEventMouseMotion and mouse_pressed:
		if Input.is_key_pressed(KEY_SHIFT):
			main.ComputeClosest(get_global_mouse_position())
		else:
			if in_points:
				main.DrawPixel(get_global_mouse_position())
	if Input.is_action_just_pressed("left_mouse"):
		if not in_points and not Input.is_key_pressed(KEY_SHIFT):
			main.AddPoint(get_global_mouse_position())
	if Input.is_action_just_released("left_mouse"):
		if not in_points and not Input.is_key_pressed(KEY_SHIFT):
			main.AddSecondPoint(get_global_mouse_position())

func _on_zoom_in_button_pressed():
	zoom *= 1.3

func _on_zoom_out_button_pressed():
	zoom /= 1.3

func check_movement(event) -> void:
	if event is InputEventKey:
		if Input.is_action_just_pressed("zoom_in"):
			zoom *= 1.3
		if Input.is_action_just_pressed("zoom_out"):
			zoom /= 1.3
		if Input.is_action_just_pressed("ui_cancel"):
			get_tree().change_scene_to_file("res://menu.tscn")
	if event is InputEventMouseMotion and mouse_pressed:
		if Input.is_key_pressed(KEY_CTRL):
			position -= event.relative / zoom.x
