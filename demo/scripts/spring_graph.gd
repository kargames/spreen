extends Control


const DURATION := 2.0
const DT := 1.0 / 240.0
const GOAL := 1.0
const Y_MIN := -0.35
const Y_MAX := 1.55

const MARGIN := Vector2(56, 28)


var title := ""

var _series: Array[Dictionary] = []


func add_series(label: String, color: Color, damping_ratio: float, halflife: float) -> void:
	var probe := Node2D.new()

	add_child(probe)
	probe.position = Vector2.ZERO

	var spreen := SpreenTree.create_spreen(null)
	spreen.set_finishable(false)
	spreen.spreen_float(probe, "position:x", GOAL, damping_ratio, halflife)

	var points := PackedVector2Array()
	var t := 0.0
	while t <= DURATION:
		points.append(Vector2(t, probe.position.x))
		spreen.custom_step(DT)
		t += DT

	spreen.kill()
	probe.queue_free()

	_series.append({"label": label, "color": color, "points": points})
	queue_redraw()


func _plot_rect() -> Rect2:
	return Rect2(MARGIN, size - MARGIN * 2.0)


func _to_screen(p: Vector2) -> Vector2:
	var r := _plot_rect()
	return Vector2(
		r.position.x + (p.x / DURATION) * r.size.x,
		r.position.y + (1.0 - (p.y - Y_MIN) / (Y_MAX - Y_MIN)) * r.size.y
	)


func _draw() -> void:
	var r := _plot_rect()
	var font := ThemeDB.fallback_font
	var grid := Color(1, 1, 1, 0.07)
	var axis := Color(1, 1, 1, 0.25)

	draw_rect(r, Color(0.10, 0.11, 0.14), true)

	for v in [0.0, 0.5, 1.0, 1.5]:
		var y := _to_screen(Vector2(0.0, v)).y
		draw_line(Vector2(r.position.x, y), Vector2(r.end.x, y), grid, 1.0)
		draw_string(font, Vector2(6, y + 5), "%.1f" % v,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(1, 1, 1, 0.45))

	var t := 0.0
	while t <= DURATION:
		var x := _to_screen(Vector2(t, 0.0)).x
		draw_line(Vector2(x, r.position.y), Vector2(x, r.end.y), grid, 1.0)
		draw_string(font, Vector2(x - 10, r.end.y + 18), "%.1fs" % t,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(1, 1, 1, 0.45))
		t += 0.5

	var goal_y := _to_screen(Vector2(0.0, GOAL)).y
	draw_dashed_line(Vector2(r.position.x, goal_y), Vector2(r.end.x, goal_y), axis, 1.0, 6.0)

	for s in _series:
		var pts: PackedVector2Array = s["points"]
		var screen := PackedVector2Array()
		for p in pts:
			screen.append(_to_screen(p))
		if screen.size() > 1:
			draw_polyline(screen, s["color"], 2.0, true)

	var ly := r.position.y + 10.0
	for s in _series:
		draw_line(Vector2(r.end.x - 150, ly), Vector2(r.end.x - 128, ly), s["color"], 2.0)
		draw_string(font, Vector2(r.end.x - 120, ly + 4), s["label"],
				HORIZONTAL_ALIGNMENT_LEFT, -1, 12, s["color"])
		ly += 18.0

	if title != "":
		draw_string(font, Vector2(r.position.x, 20), title,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color(1, 1, 1, 0.8))


func _notification(what: int) -> void:
	if what == NOTIFICATION_RESIZED:
		queue_redraw()
