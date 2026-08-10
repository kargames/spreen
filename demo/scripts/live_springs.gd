extends Control


const MARKER_RADIUS := 11.0


var rows: Array[Dictionary] = []

var _goal_x := 0.0
var _left := true
var _elapsed := 0.0
var _auto_seconds := 5.0


func add_row(label: String, color: Color, damping_ratio: float, halflife: float) -> void:
  var marker := Node2D.new()
  add_child(marker)

  var spreen := SpreenTree.create_spreen(marker)
  spreen.set_finishable(false)
  var spreener := spreen.spreen_float(marker, "position:x", 0.0, damping_ratio, halflife)

  rows.append({
    "label": label,
    "color": color,
    "marker": marker,
    "spreener": spreener,
    "damping_ratio": damping_ratio,
    "halflife": halflife,
  })


func _ready() -> void:
  _retarget(true)


func _retarget(force: bool = false) -> void:
  var r := _track_rect()
  _left = not _left if not force else _left
  _goal_x = r.position.x + (50.0 if _left else r.size.x - 50.0)
  for row in rows:
    row["spreener"].update_goal(_goal_x)
  queue_redraw()


func _track_rect() -> Rect2:
  return Rect2(Vector2(24, 44), Vector2(maxf(size.x - 48.0, 32.0), maxf(size.y - 68.0, 32.0)))


func _row_y(i: int) -> float:
  var r := _track_rect()
  var step := r.size.y / float(maxi(rows.size(), 1))
  return r.position.y + step * (i + 0.5)


func _process(delta: float) -> void:
  _elapsed += delta
  if _elapsed >= _auto_seconds:
    _elapsed = 0.0
    _retarget()
  queue_redraw()


func _gui_input(event: InputEvent) -> void:
  if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
    _elapsed = 0.0
    _retarget()


func _draw() -> void:
  var font := ThemeDB.fallback_font
  var r := _track_rect()

  draw_string(font, Vector2(24, 26),
      "Click to flip the goal early",
      HORIZONTAL_ALIGNMENT_LEFT, -1, 13, Color(1, 1, 1, 0.6))

  for i in rows.size():
    var row := rows[i]
    var y := _row_y(i)

    draw_line(Vector2(r.position.x, y), Vector2(r.end.x, y), Color(1, 1, 1, 0.08), 2.0)
    draw_circle(Vector2(_goal_x, y), 4.0, Color(1, 1, 1, 0.28))

    var marker: Node2D = row["marker"]
    draw_circle(Vector2(marker.position.x, y), MARKER_RADIUS, row["color"])

    draw_string(font, Vector2(r.position.x, y - 18.0),
        "%s  (ratio %.2f, halflife %.2fs)" % [row["label"], row["damping_ratio"], row["halflife"]],
        HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(1, 1, 1, 0.55))


func _notification(what: int) -> void:
  if what == NOTIFICATION_RESIZED and is_inside_tree():
    _retarget(true)
