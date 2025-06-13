module := "stdb-physics"

publish:
    spacetime publish -p spacetime_physics {{module}} -y -c

delete:
    spacetime delete {{module}} -y

client:
    cargo run --bin client

bindings:
    spacetime generate --lang rust --out-dir ./client/src/module_bindings --project-path spacetime_physics

logs:
    spacetime logs {{module}}

sc: publish client

all: publish bindings client
