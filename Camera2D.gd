extends Camera2D

@onready var main = $".."

var mouse_pressed: bool = false

func _input(event) -> void:
	if event is InputEventMouseButton:
		mouse_pressed = event.is_pressed()
	if event is InputEventMouseMotion and mouse_pressed:
		if Input.is_key_pressed(KEY_CTRL):
			position -= event.relative / zoom.x
		elif Input.is_key_pressed(KEY_SHIFT):
			main.ComputeClosest(get_global_mouse_position())
		else:
			main.DrawPixel(get_global_mouse_position())
	if event is InputEventKey:
		if Input.is_action_just_pressed("zoom_in"):
			zoom *= 1.3
		if Input.is_action_just_pressed("zoom_out"):
			zoom /= 1.3

func _on_zoom_in_button_pressed():
	zoom *= 1.3

func _on_zoom_out_button_pressed():
	zoom /= 1.3
