extends Control


const SpringGraph := preload("res://scripts/spring_graph.gd")
const LiveSprings := preload("res://scripts/live_springs.gd")
const CursorChase := preload("res://scripts/cursor_chase.gd")


const UNDER := Color("#ff6b6b")
const CRITICAL := Color("#4ecdc4")
const OVER := Color("#ffd166")
const EXTRA := Color("#a78bfa")


func _ready() -> void:
  var bg := ColorRect.new()
  bg.color = Color(0.07, 0.08, 0.10)
  bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
  add_child(bg)

  var tabs := TabContainer.new()
  tabs.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
  add_child(tabs)

  tabs.add_child(_damping_graph())
  tabs.add_child(_halflife_graph())
  tabs.add_child(_live_springs())
  tabs.add_child(_cursor_chase())


func _damping_graph() -> Control:
  var graph := SpringGraph.new()
  graph.name = "Damping Ratio"
  graph.title = "0.35s halflife, varying damping ratio"
  graph.add_series("0.25 under-damped", UNDER, 0.25, 0.35)
  graph.add_series("0.50 under-damped", EXTRA, 0.50, 0.35)
  graph.add_series("1.00 critically damped", CRITICAL, 1.00, 0.35)
  graph.add_series("2.50 over-damped", OVER, 2.50, 0.35)
  return graph


func _halflife_graph() -> Control:
  var graph := SpringGraph.new()
  graph.name = "Halflife"
  graph.title = "Critically damped, varying halflife"
  graph.add_series("0.08s snappy", UNDER, 1.0, 0.08)
  graph.add_series("0.20s", EXTRA, 1.0, 0.20)
  graph.add_series("0.45s", CRITICAL, 1.0, 0.45)
  graph.add_series("0.90s languid", OVER, 1.0, 0.90)
  return graph


func _live_springs() -> Control:
  var live := LiveSprings.new()
  live.name = "Live"
  live.add_row("under-damped", UNDER, 0.25, 0.7)
  live.add_row("slightly under-damped", EXTRA, 0.60, 0.7)
  live.add_row("critically damped", CRITICAL, 1.00, 0.7)
  live.add_row("over-damped", OVER, 2.50, 0.35)
  return live


func _cursor_chase() -> Control:
  var chase := CursorChase.new()
  chase.name = "Follow the Cursor"
  return chase
