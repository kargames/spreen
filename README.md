# Spreen

Spreen is a Godot module that provides a Tween-like API for animation and interpolation using springs as a base. Unlike `TRANS_SPRING` in the Tween library, these springs are defined by damping ratios and halflife. 

These help you make fantastic character and camera controllers, reacting to user input in very natural ways.

## The Basics

1. Compile Godot including this repository in a folder titled `spreen` in the default Godot modules directory or a directory defined with `custom_modules`. 

2. Create a Spreen object, optionally binding it to a node (in this case, `self`).

```gdscript
var spreen := SpreenTree.create_spreen(self)
```

3. Optionally set whether the Spreen can finish, typically you will not want to finish a controller spreen.

```gdscript
spreen.set_finishable(false)
```

4. Create and optionally save reference to a property tweener.

```gdscript
var transform_spreen := spreen.spreen_transform_3d(%Box, "global_transform", %Box.global_transform.looking_at(%Target.global_position), 0.5, 0.25)
```

5. Update the springs goal in real time.

```gdscript
transform_spreen.update_goal(%Box.global_transform.looking_at(%Target.global_position))
```

## Why Springs?

This entire module is based on the math and use-cases presented in [this fantastic article by The Orange Duck](https://theorangeduck.com/page/spring-roll-call). If that doesn't convince you, I don't know what will.

## Getting Involved

This module was recently re-written to match the style of Godot's Tween class, but this new version lacks the battle testing of its predecessor. If you find areas that could be improved, please let us know!

We primarily use modules at [KAR Games](https://kar.games) but would welcome a PR to allow this code to be shared as both a module and GDExtension. 

## Contributors

This module was originally written by [Guillaume Bailey](https://github.com/gbudee) and was re-architected for Godot by [Chris Ridenour](https://github.com/cridenour).