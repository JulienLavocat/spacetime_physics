module := "stdb-physics"

publish:
    spacetime publish -p spacetime_physics {{module}} -y -c

delete:
    spacetime delete {{module}} -y
