extends Control


const LINK_COUNT := 6
const HALFLIFE := 0.11
const DAMPING_RATIO := 0.85


var _links: Array[Dictionary] = []
var _trail: PackedVector2Array = []
var _placed := false


func _ready() -> void:
  for i in LINK_COUNT:
    var node := Node2D.new()
    add_child(node)

    var spreen := SpreenTree.create_spreen(node)
    spreen.set_finishable(false)
    var halflife := HALFLIFE * (1.0 + i * 0.55)
    var spreener := spreen.spreen_vector2(node, "position", Vector2.ZERO, DAMPING_RATIO, halflife)

    _links.append({
      "node": node,
      "spreener": spreener,
      "radius": 20.0 - i * 2.2,
      "color": Color.from_hsv(0.55 + i * 0.035, 0.55, 1.0),
    })


func _process(_delta: float) -> void:
  # size is not laid out yet during _ready, so snap the chain into place on the
  # first frame the panel has real dimensions.
  if not _placed and size.x > 1.0:
    _placed = true
    for link in _links:
      link["node"].position = size * 0.5
      link["spreener"].update_goal(size * 0.5)
    _trail.clear()

  var target := get_local_mouse_position()
  if not Rect2(Vector2.ZERO, size).has_point(target):
    target = size * 0.5

  for i in _links.size():
    var goal: Vector2 = target if i == 0 else _links[i - 1]["node"].position
    _links[i]["spreener"].update_goal(goal)

  if not _links.is_empty():
    _trail.append(_links[0]["node"].position)
    if _trail.size() > 90:
      _trail.remove_at(0)

  queue_redraw()


func _draw() -> void:
  if _trail.size() > 1:
    draw_polyline(_trail, Color(1, 1, 1, 0.10), 1.5, true)

  for i in range(_links.size() - 1, -1, -1):
    var link := _links[i]
    var node: Node2D = link["node"]
    if i > 0:
      var prev: Node2D = _links[i - 1]["node"]
      draw_line(node.position, prev.position, Color(1, 1, 1, 0.14), 2.0)
    draw_circle(node.position, link["radius"], link["color"])
