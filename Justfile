module := "stdb-physics"

publish:
    spacetime publish -p spacetime_physics {{module}} -y -c

delete:
    spacetime delete {{module}} -y

client:
    bevy run --bin client

bindings:
    spacetime generate --lang rust --out-dir ./client/src/module_bindings --project-path spacetime_physics

all: publish bindings client
