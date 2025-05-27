def can_build(env, platform):
   return True

def configure(env):
   pass

def get_doc_classes():
	return [
      "BasisSpreener",
      "FloatSpreener",
      "Spreen",
      "Spreener",
      "SpreenTree",
      "Transform2DSpreener",
      "Transform3DSpreener",
      "Vector2Spreener",
      "Vector3Spreener",
	]

def get_doc_path():
	return "doc_classes"